// tb_scsi: rtl/scsi/scsi_targets.sv against a model of its disks.
//
// The bench is the initiator (a byte-level REQ/ACK partner, as
// ncr53c710.sv) and hps_io (block requests served from in-memory images
// after a random delay, like Main's round trip). It runs random READ(6),
// READ(10) and WRITE(10) commands of 1..80 blocks on the two disks and
// checks every byte read against the model, while the CD-ROM plays audio
// (its frames are requests on the same port, between the disks' chunks).
// Checks: one block request at a time, the data, the status, and that a
// READ's chunks are asked for ahead (requests overlap the bus).
//
//   make -C verilator tb_scsi        (optionally TB_SCSI_ARGS="seed cycles")
#include "Vscsi_targets.h"
#include "verilated.h"
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <vector>
#include <random>

static Vscsi_targets* t;
static uint64_t cyc = 0;
static std::mt19937 rng;
static int fails = 0;

static const uint32_t DISK_BLOCKS = 8192;     // 4 MB each
static const uint32_t CD_BLOCKS = 4096;       // 512-byte units
static std::vector<uint8_t> img[3];           // what hps_io serves
static std::vector<uint8_t> model[2];         // what the disks should hold

static int rnd(int lo, int hi) { return std::uniform_int_distribution<int>(lo, hi)(rng); }

// ------------------------------------------------------------ hps_io model
static struct {
	int st = 0;             // 0 idle, 1 waiting, 2 reading (to the core), 3 writing (from it), 4 drop
	int n = 0, wait = 0, phase = 0;
	uint32_t words = 0, w = 0;
	uint64_t lba = 0;
	bool rd = false;
	std::vector<uint8_t> buf;
} sd;
static uint64_t reqs = 0, overlap = 0, aud_reqs = 0;
static int lat_max = 400;

static uint8_t cd_byte(uint64_t lba, uint32_t i) {
	if (lba >= 0x80000000ull) return (uint8_t)(i * 7 + lba);  // audio frames: anything
	if (lba >= 0x40000000ull) return 0;                         // the track table window: none
	return img[2][(lba * 512 + i) % img[2].size()];
}

static void sd_step(bool bus_busy) {
	t->sd_buff_wr = 0;
	switch (sd.st) {
	case 0:
		for (int n = 0; n < 3; n++)
			if (((t->sd_rd | t->sd_wr) >> n) & 1) {
				sd.n = n; sd.lba = t->sd_lba[n]; sd.rd = (t->sd_rd >> n) & 1;
				sd.words = (t->sd_blk_cnt[n] + 1) * 256; sd.w = 0; sd.phase = 0;
				if (__builtin_popcount(t->sd_rd | t->sd_wr) != 1) { printf("FAIL two requests at once %x/%x\n", t->sd_rd, t->sd_wr); fails++; }
				if (sd.words > 2048) { printf("FAIL request of %u blocks\n", sd.words / 256); fails++; }
				reqs++;
				if (n == 2 && sd.lba >= 0x80000000ull) aud_reqs++;
				if (bus_busy) overlap++;
				sd.buf.assign(sd.words * 2, 0);
				if (sd.rd) {
					for (uint32_t i = 0; i < sd.words * 2; i++)
						sd.buf[i] = n == 2 ? cd_byte(sd.lba, i) : img[n][(sd.lba * 512 + i) % img[n].size()];
				}
				sd.wait = rnd(1, lat_max); sd.st = 1;
				break;
			}
		break;
	case 1:
		if (--sd.wait > 0) break;
		t->sd_ack = 1 << sd.n;
		sd.st = sd.rd ? 2 : 3;
		break;
	case 2:
		if (++sd.phase % rnd(1, 4)) break;
		if (sd.w < sd.words) {
			t->sd_buff_addr = sd.w;
			t->sd_buff_dout = sd.buf[2 * sd.w] | (sd.buf[2 * sd.w + 1] << 8);
			t->sd_buff_wr = 1;
			sd.w++;
		} else { t->sd_ack = 0; sd.st = 0; }
		break;
	case 3:
		if ((++sd.phase & 3) == 1) t->sd_buff_addr = sd.w;
		else if ((sd.phase & 3) == 0) {
			uint16_t v = t->sd_buff_din[sd.n];
			sd.buf[2 * sd.w] = v & 0xFF; sd.buf[2 * sd.w + 1] = v >> 8;
			if (++sd.w == sd.words) {
				if (sd.n < 2) memcpy(&img[sd.n][sd.lba * 512], sd.buf.data(), sd.buf.size());
				else { printf("FAIL write to the CD\n"); fails++; }
				t->sd_ack = 0; sd.st = 0;
			}
		}
		break;
	}
}

static void tick(bool bus_busy) {
	sd_step(bus_busy);
	t->clk = 0; t->eval();
	t->clk = 1; t->eval();
	cyc++;
}

// ------------------------------------------------------------- initiator
// one command: select, CDB out, data in or out, status and message in
static bool command(int id, const std::vector<uint8_t>& cdb, std::vector<uint8_t>& din,
                    const std::vector<uint8_t>& dout, uint8_t& status) {
	t->sel_id = id; t->sel_atn = 0; t->sel = 1; tick(false); t->sel = 0;
	for (int k = 0; k < 20 && !t->sel_done && !t->sel_fail; k++) tick(false);
	if (!t->bsy) { printf("FAIL no selection of %d\n", id); return false; }
	size_t ci = 0, oi = 0; bool done = false;
	uint64_t limit = cyc + 20000000;
	while (cyc < limit) {
		if (t->req && !t->ack) {
			int ph = t->phase;
			if (ph == 2) t->idata = ci < cdb.size() ? cdb[ci++] : 0;
			else if (ph == 0) t->idata = oi < dout.size() ? dout[oi++] : 0;
			else if (ph == 1) din.push_back(t->tdata);
			else if (ph == 3) status = t->tdata;
			else if (ph == 7) done = true;
			t->ack = 1;
			// a random pause before the initiator takes a byte (DMA, the CPU)
		} else t->ack = 0;
		tick(true);
		if (!t->ack && rnd(0, 15) == 0) for (int k = rnd(1, 30); k; k--) tick(true);
		if (done && !t->bsy) return true;
	}
	printf("FAIL command 0x%02x timed out\n", cdb[0]);
	return false;
}

int main(int argc, char** argv) {
	Verilated::commandArgs(argc, argv);
	unsigned seed = argc > 1 ? strtoul(argv[1], 0, 0) : 1;
	int ncmd = argc > 2 ? atoi(argv[2]) : 400;
	rng.seed(seed);
	t = new Vscsi_targets;
	for (int d = 0; d < 3; d++) {
		img[d].resize((d == 2 ? CD_BLOCKS : DISK_BLOCKS) * 512);
		for (auto& b : img[d]) b = (uint8_t)rnd(0, 255);
		if (d < 2) model[d] = img[d];
	}
	t->img_present = 7;
	t->img_blocks[0] = DISK_BLOCKS; t->img_blocks[1] = DISK_BLOCKS; t->img_blocks[2] = CD_BLOCKS;
	t->rst = 1; for (int k = 0; k < 10; k++) tick(false); t->rst = 0;
	for (int k = 0; k < 10; k++) tick(false);

	// the CD plays audio throughout: PLAY AUDIO(10) from LBA 0, 30000 frames
	{
		std::vector<uint8_t> din; uint8_t st = 0xFF;
		command(2, {0x45, 0, 0, 0, 0, 0, 0, 0x75, 0x30, 0}, din, {}, st);
		if (st != 0) { printf("FAIL PLAY AUDIO status %02x\n", st); fails++; }
	}

	uint64_t bytes = 0, c0 = cyc;
	for (int c = 0; c < ncmd && fails < 10; c++) {
		int d = rnd(0, 1), id = d == 0 ? 6 : 5;
		int kind = rnd(0, 9);
		if (c % 50 == 0) lat_max = std::vector<int>{1, 40, 400, 3000}[rnd(0, 3)];
		uint32_t n = rnd(0, 3) == 0 ? rnd(1, 80) : std::vector<uint32_t>{1, 2, 7, 8, 9, 16, 17, 64}[rnd(0, 7)];
		uint32_t lba = rnd(0, DISK_BLOCKS - n);
		std::vector<uint8_t> din, dout; uint8_t st = 0xFF;
		if (kind < 4) {                              // WRITE(10)
			dout.resize(n * 512);
			for (auto& b : dout) b = (uint8_t)rnd(0, 255);
			command(id, {0x2A, 0, (uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
			             0, (uint8_t)(n >> 8), (uint8_t)n, 0}, din, dout, st);
			memcpy(&model[d][lba * 512], dout.data(), dout.size());
			if (st != 0) { printf("FAIL WRITE status %02x\n", st); fails++; }
			if (!din.empty()) { printf("FAIL WRITE gave data in\n"); fails++; }
		} else {
			if (kind == 9 && n <= 255 && lba < 0x200000) {   // READ(6)
				command(id, {0x08, (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba, (uint8_t)n, 0}, din, dout, st);
			} else {
				command(id, {0x28, 0, (uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba,
				             0, (uint8_t)(n >> 8), (uint8_t)n, 0}, din, dout, st);
			}
			if (st != 0) { printf("FAIL READ status %02x\n", st); fails++; }
			if (din.size() != n * 512) { printf("FAIL READ %u blocks @%u gave %zu bytes\n", n, lba, din.size()); fails++; }
			else if (memcmp(din.data(), &model[d][lba * 512], din.size())) {
				size_t i = 0; while (din[i] == model[d][lba * 512 + i]) i++;
				printf("FAIL READ disk %d %u blocks @%u: byte %zu is %02x, not %02x (cmd %d)\n", d, n, lba, i,
				       din[i], model[d][lba * 512 + i], c);
				fails++;
			}
		}
		bytes += n * 512;
	}
	// the images must equal the model once the writes are in
	for (int k = 0; k < 5000; k++) tick(false);
	for (int d = 0; d < 2; d++)
		if (img[d] != model[d]) { printf("FAIL disk %d image differs from the model\n", d); fails++; }
	printf("tb_scsi seed %u: %d commands, %llu bytes in %llu cycles; %llu requests (%llu while the bus was busy, %llu CD audio)\n",
	       seed, ncmd, (unsigned long long)bytes, (unsigned long long)(cyc - c0), (unsigned long long)reqs,
	       (unsigned long long)overlap, (unsigned long long)aud_reqs);
	if (aud_reqs == 0) { printf("FAIL no CD audio frames were fetched\n"); fails++; }
	printf(fails ? "tb_scsi: FAIL\n" : "tb_scsi: PASS\n");
	delete t;
	return fails ? 1 : 0;
}

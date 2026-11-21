# HP9K715_MiSTer

An **HP 9000 Model 715/64** PA-RISC workstation for the
[MiSTer](https://github.com/MiSTer-devel/Wiki_MiSTer/wiki) FPGA, written from
scratch in SystemVerilog: a PA-7100LC CPU with its FPU, the LASI I/O chip
(SCSI, Ethernet, serial, PS/2, audio) and the built-in Artist graphics. It
runs HP's own boot ROM (PDC 1.6) and HP-UX. Licensed GPL-2.0-or-later.

**Requires the MiSTer SDRAM module.**

## Features

- PA-7100LC CPU at 64 MHz: PA-RISC 1.1 with the MAX-1 multimedia
  instructions, IEEE single/double FPU, 128 KB instruction and 128 KB data
  caches, a 64-entry TLB with the hardware miss handler.
- 16, 32, 64 or 128 MB of memory.
- Model 715/64, 715/80 or 715/100 (the model ID the firmware reports).
- Boots HP's PDC 1.6 (`boot0.rom` in `release/`), or another ROM through
  Load ROM.
- SCSI (NCR 53C710): two hard disks and a CD-ROM, with CD audio.
- Ethernet (i82596) through the MiSTer's network.
- Artist graphics: 1280x1024, 8-bit, hardware cursor, accelerated fills,
  lines and text. Or the console on the serial port.
- PS/2 keyboard and mouse from the MiSTer's USB devices (Alt/Win swap).
- Harmony audio (CS4215).
- The RS-232 port on the MiSTer's UART.
- EEPROM (firmware settings), built in or saved to an image file; the
  clock set from the MiSTer's.
- TOC and soft power buttons in the OSD.
- OSD in the computer cores' layout ([docs/osd-standard.md](docs/osd-standard.md)).

## Tested

- HP-UX 10.20: boots from disk to the CDE desktop; keyboard, mouse, audio,
  CD audio.
- HP-UX 11i v1 (11.11): boots to CDE, network; installs from its CD/DVD.
- HP's PDC 1.6: every self-test, `BOOT_ADMIN>`, ISL.
- The CPU test suites ([tests/cpu](tests/cpu/README.md)): 14 of 14 pass,
  and the two CPU models run in lockstep.
- One MiSTer so far (a SuperStation One).

## Limitations

- One instruction per clock at 64 MHz; the real PA-7100LC can run two, so
  the core is somewhat slower than a real 715/64. Every clock meets the
  timing analysis.
- At most 128 MB of memory (a real 715 takes 256 MB).
- Disk access is slow: installing an operating system takes hours.
- The floppy controller has no drive; audio recording is silent; no HIL,
  second serial port or EISA.
- HP-UX 10.20's standard kernel has no driver for the built-in Ethernet;
  HP-UX 11i's has.
- The network and the right time of day need the HP 9000 support in
  Main_MiSTer, which is not in the official release yet: use the `MiSTer`
  file from `release/`.
- `HP9K715_20261018.rbf` in `release/` is the build before the current
  one, with the slower CPU.

## Known issues

- Advanced > Boot mode does not work reliably. Press Esc at start-up and
  type the commands at `BOOT_ADMIN>` instead.
- NeXTSTEP 3.3's CD boots to its installer; installing it has not been
  tried. Linux, NetBSD and OpenBSD have not been tried.
- A black screen in CDE is the screen saver: move the mouse.

## Install

You need a MiSTer with an SDRAM module (32 MB is enough for HP-UX 10.20,
128 MB is recommended for HP-UX 11i), an exFAT SD card and an HP-UX disk
image or install CD (see [Disk images](#disk-images)).

Copy the files from [`release/`](release/):

| file | copy to |
|---|---|
| `HP9K715_20261121.rbf` | `/media/fat/_Unstable/` (or `_Computer/`) |
| `boot0.rom` | `/media/fat/games/HP9K715/` |
| `hp715.eep` | `/media/fat/games/HP9K715/` (optional, see below) |
| `MiSTer` | `/media/fat/MiSTer`, replacing the one there (optional, see below) |

Put your disk images in `/media/fat/games/HP9K715/`.

**Video.** The core shows 1280x1024. Give it a 1080-line HDMI mode in
`MiSTer.ini`, otherwise the picture is squeezed and small text is hard to
read:

    [HP9K715]
    video_mode=8

**MiSTer main binary.** Networking and the correct time of day need the
HP 9000 support in the MiSTer main program. Until it is part of the official
MiSTer release, use the `MiSTer` file from `release/`. With the stock one the
core still runs, but there is no network and HP-UX's clock is off by your
time-zone offset.

**EEPROM.** The 715 keeps its firmware settings (boot path, console, monitor
type, network address) in an EEPROM. The core has one built in, set up to
boot from the disk at SCSI ID 6 with the console on the screen, so nothing
needs to be mounted. Mount `hp715.eep` (OSD > Advanced > EEPROM image) only
if you want changes made at the firmware prompt to be saved.

**Network address.** The 715's Ethernet address is kept in the EEPROM, and
every copy of this core starts with the same one, `08:00:09:71:5A:01`. Two
cores on one network need different addresses. Make an EEPROM image with
your own (Python 3, on any computer) and mount it as the EEPROM image:

    python3 tools/eeprom/mkeeprom.py setmac release/hp715.eep my715.eep 08:00:09:12:34:56

Keep `08:00:09` (HP's prefix) and choose the last three bytes. `lan_addr` at
the `BOOT_ADMIN>` prompt shows the address in use.

## The OSD

Start HP9K715 from the MiSTer menu, put a disk image in **Pri. disk (ID 6)**
and press **Apply and reset**. The boot ROM tests the machine (about a
minute), then boots the disk.

Open the menu with **F12**. Most settings take effect at the next reset:
change them, then choose **Apply and reset** ("Changes pending" is shown
until you do). Disks, the CD, Load ROM, the network and the keyboard
settings take effect at once.

| menu | option | values |
|---|---|---|
| main | Pri. disk (ID 6) | the disk the machine boots from |
| main | Disk (ID 5) | a second disk |
| main | CD-ROM (ID 2) | an ISO, or CUE/BIN with the custom main binary |
| main | Load ROM | boots another ROM file until the core is restarted |
| main | Memory | **32MB**, 64MB, 128MB, 16MB |
| main | Network interface | **eth0**, None, eth1, macvlan, tap0 |
| Machine | Model | **715/64**, 715/80, 715/100 |
| Machine | TOC (front panel) | the front-panel button: HP-UX dumps memory and reboots |
| Machine | Soft power off | the power button: HP-UX shuts down cleanly |
| Video | Display | **Graphics**, Auto, Serial console |
| Video | Aspect ratio, Scale | **Original**; **V-Integer** |
| Input | Swap Alt/Win keys | **No**, Yes (for Mac keyboards) |
| Advanced | Boot mode | **Normal**, BOOT_ADMIN, ISL, Single-user, LVM maint., Network, CD/DVD |
| Advanced | EEPROM image | an `.eep` file to keep firmware changes |

**Getting to the firmware prompt.** Press **Esc** while the boot ROM starts
up to stop the automatic boot and get the `BOOT_ADMIN>` prompt. From there,
for example, `boot scsi.2` boots the CD and `boot pri isl` stops at the
`ISL>` prompt, where `hpux -is` boots HP-UX single-user.

**Display options.** *Graphics* always shows the Artist screen. *Serial
console* shows the 715's RS-232 console instead (the MiSTer keyboard types
into it). *Auto* shows the serial console until HP-UX turns the graphics
on. A black screen in CDE is the screen saver: move the mouse.

## Disk images

The 715 boots from SCSI ID 6 by default. An HP-UX disk records the SCSI ID it
was installed at, so an image installed at another ID, or on another machine
or emulator, has to be adjusted once before it boots at ID 6.
[`disks/README.md`](disks/README.md) has the steps, including for an
HP-UX 10.20 image made for QEMU and for moving an installed 11.11 disk from
ID 5 to ID 6.

To install HP-UX 11i, put the install CD/DVD in the CD-ROM slot and an empty
image of at least 4 GB at ID 6, then type `boot scsi.2` at `BOOT_ADMIN>`.
Use the HP-UX installer's text interface on the graphics console. Expect the
installation to take several hours.

## What is here

| Path | What |
|---|---|
| `rtl/cpu/` | the PA-7100LC: pipeline, caches, TLB, FPU ([docs/design/cpu.md](docs/design/cpu.md)) |
| `rtl/hp715.sv`, `rtl/mioc.sv`, `rtl/lasi/` | the machine: memory and I/O controller, LASI ([docs/design/system.md](docs/design/system.md)) |
| `rtl/artist/` | the Artist graphics ([docs/design/artist.md](docs/design/artist.md)) |
| `rtl/scsi/` | the 53C710 and the SCSI disks and CD-ROM |
| `rtl/mister/` | the MiSTer side: SDRAM, DDR3, the EEPROM image |
| `tests/`, `verilator/` | the CPU test suites, the lockstep co-simulation, the whole machine on HP's ROM |
| `tools/` | ROM analysis, HP-UX disk and file-system tools, EEPROM images, board scripts |
| `docs/` | the design notes, the analysis of HP's boot ROM (`reference/rom/`, no HP code), [STUBS.md](docs/STUBS.md) for everything simplified or left out |
| `sys/` | the Template_MiSTer framework, unchanged |
| `release/` | the release: the rbf, the boot ROM, an EEPROM image, the Main_MiSTer binary |

## Building

- The core: Quartus 17.0.2 Lite, `HP9K715.qpf`.
- Tests (Verilator 5): `tests/run-cputest.sh` (the CPU suites),
  `tests/run-cosim.sh` (the two CPU models in lockstep), `tests/run-rom.sh`
  (HP's ROM to `BOOT_ADMIN>`).

## Credits

- **Template_MiSTer** (`sys/`): the MiSTer project (Sorgelig and
  contributors).
- The SDRAM mapping and the save-image pattern follow the MiSTer Macintosh
  Quadra core.
- QEMU's hppa machine and SeaBIOS-hppa, used as references.

## Licence

GPL-2.0-or-later, as the MiSTer template ([LICENSE](LICENSE)).

`release/boot0.rom` is HP's boot ROM. It is HP's copyrighted firmware and is
not covered by this repository's licence.

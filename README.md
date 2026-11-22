# HP9K715_MiSTer

This core turns a [MiSTer](https://github.com/MiSTer-devel/Wiki_MiSTer/wiki)
into an **HP 9000 Model 715/64**, HP's PA-RISC desktop workstation from the
mid-1990s. Everything is written from scratch in SystemVerilog: the PA-7100LC
processor and its floating-point unit, the LASI I/O chip (SCSI, Ethernet,
serial, PS/2, audio) and the Artist graphics built into the board. It runs
HP's own boot firmware (PDC 1.6) and boots HP-UX to the CDE desktop.

The core needs a MiSTer with an **SDRAM module**.

## What the core does

The processor is a PA-7100LC running at 64 MHz. It implements PA-RISC 1.1
with the MAX-1 multimedia instructions and an IEEE single- and
double-precision FPU. As in the real chip, it has 128 KB instruction and
128 KB data caches and a 64-entry TLB that refills itself in hardware.

Around it is the rest of the 715:

- 16, 32, 64 or 128 MB of memory.
- The model the firmware reports: 715/64, 715/80 or 715/100.
- SCSI (an NCR 53C710) with two hard disks and a CD-ROM drive, including
  CD audio.
- Ethernet (an Intel 82596), connected to the MiSTer's network.
- Artist graphics at 1280x1024 with 256 colours, a hardware cursor and the
  accelerated fills, lines and text HP-UX's X server uses. The console can
  also run on the serial port.
- A PS/2 keyboard and mouse, fed from the MiSTer's USB devices.
- Harmony audio with its CS4215 codec.
- The RS-232 port, wired to the MiSTer's UART.
- The EEPROM that holds the firmware settings, either built in or saved to
  an image file on the SD card. The real-time clock is set from the
  MiSTer's.
- The front-panel TOC button and the soft power button, both in the OSD.

The OSD follows the layout of the other MiSTer computer cores
([docs/osd-standard.md](docs/osd-standard.md)).

## What has been tested

HP-UX 10.20 boots from disk to the CDE desktop, with keyboard, mouse, audio
and CD audio working. HP-UX 11i v1 (11.11) boots to CDE with networking and
installs from its CD or DVD. HP's PDC 1.6 passes every one of its
self-tests and reaches the `BOOT_ADMIN>` prompt and ISL.

The CPU passes all 14 of its test suites ([tests/cpu](tests/cpu/README.md)),
and the pipelined core runs in lockstep with a simpler reference core.

So far the core has been tested on one MiSTer, a SuperStation One.

## Limitations

- The core issues one instruction per clock at 64 MHz. The real PA-7100LC
  can issue two, so the core is somewhat slower than a real 715/64.
- Memory tops out at 128 MB; a real 715 takes up to 256 MB.
- Disk access is slow, and installing an operating system takes hours.
- The floppy controller has no drive attached, audio recording is silent,
  and there is no HIL, second serial port or EISA.
- HP-UX 10.20's standard kernel has no driver for the built-in Ethernet.
  HP-UX 11i's does.
- Networking, CUE/BIN CD images and the right time of day need HP 9000
  support in the MiSTer main program, which is not in the official MiSTer
  release yet. With the stock main program the core still runs, but there
  is no network, the CD-ROM takes ISO images only, and HP-UX's clock is off
  by your time-zone offset.

## Known issues

- Advanced > Boot mode in the OSD is not reliable. Press Esc while the
  machine starts up and type the commands at `BOOT_ADMIN>` instead.
- NeXTSTEP 3.3's CD boots to its installer, but nobody has tried installing
  it yet. Linux, NetBSD and OpenBSD have not been tried at all.
- A black screen in CDE is just the screen saver: move the mouse.

## Installing

You need:

- A MiSTer with an SDRAM module. 32 MB is enough for HP-UX 10.20; 128 MB is
  recommended for HP-UX 11i.
- An exFAT SD card.
- HP's boot ROM for the 715/64, PDC revision 1.6. It is HP's copyrighted
  firmware, so it is not part of this repository; you have to supply your
  own copy. HP shipped Series 700 firmware on update tapes, and
  [`tools/hpfw/hpfw.py`](tools/hpfw/hpfw.py) can list and unpack such a tape
  image and check its ROM image's checksum.
- An HP-UX disk image or install CD (see [Disk images](#disk-images)).

Copy these files to the SD card:

| file | where it goes |
|---|---|
| `release/HP9K715_20261121.rbf` | `/media/fat/_Unstable/` (or `_Computer/`) |
| your PDC 1.6 ROM, named `boot0.rom` | `/media/fat/games/HP9K715/` |
| `release/hp715.eep` | `/media/fat/games/HP9K715/` (optional, see below) |

Put your disk images in `/media/fat/games/HP9K715/` as well.

**Video.** The core shows a 1280x1024 picture. Give it a 1080-line HDMI mode
in `MiSTer.ini`; otherwise the picture is squeezed and small text is hard to
read:

    [HP9K715]
    video_mode=8

**EEPROM.** The 715 keeps its firmware settings (boot path, console, monitor
type, network address) in an EEPROM. The core has one built in, set up to
boot from the disk at SCSI ID 6 with the console on the screen, so you do
not need to mount anything. Mount `hp715.eep` (OSD > Advanced > EEPROM image)
only if you want changes you make at the firmware prompt to be kept.

**Network address.** The Ethernet address also lives in the EEPROM, and every
copy of this core starts with the same one, `08:00:09:71:5A:01`. If you run
two cores on one network, they need different addresses. Make an EEPROM
image with your own address (Python 3, on any computer) and mount it as the
EEPROM image:

    python3 tools/eeprom/mkeeprom.py setmac release/hp715.eep my715.eep 08:00:09:12:34:56

Keep `08:00:09`, HP's prefix, and pick the last three bytes yourself. The
`lan_addr` command at the `BOOT_ADMIN>` prompt shows the address in use.

## Using the core

Start HP9K715 from the MiSTer menu, put a disk image in **Pri. disk (ID 6)**
and choose **Apply and reset**. The boot ROM tests the machine for about a
minute and then boots the disk.

Press **F12** to open the menu. Most settings take effect at the next reset:
change them, then choose **Apply and reset** ("Changes pending" is shown
until you do). Disks, the CD, Load ROM, the network and the keyboard
settings take effect immediately.

| menu | option | values |
|---|---|---|
| main | Pri. disk (ID 6) | the disk the machine boots from |
| main | Disk (ID 5) | a second disk |
| main | CD-ROM (ID 2) | an ISO image, or CUE/BIN with the HP 9000 main program |
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
| Advanced | EEPROM image | an `.eep` file that keeps firmware changes |

Defaults are in bold.

**The firmware prompt.** Press **Esc** while the boot ROM starts up to stop
the automatic boot and get the `BOOT_ADMIN>` prompt. From there,
`boot scsi.2` boots the CD, and `boot pri isl` stops at the `ISL>` prompt,
where `hpux -is` boots HP-UX in single-user mode.

**Display.** *Graphics* always shows the Artist screen. *Serial console*
shows the 715's RS-232 console instead, and the MiSTer keyboard types into
it. *Auto* shows the serial console until HP-UX switches the graphics on.

## Disk images

The 715 boots from the disk at SCSI ID 6. An HP-UX disk remembers the
hardware path of the disk it was installed on, so an image installed at
another SCSI ID, or on another machine or emulator, has to be adjusted once
before it will boot here. On the 715 the disk at ID 6 is `2/0/1.6.0`.

For HP-UX 10.x, [`tools/hpux/lvmbdra.py`](tools/hpux/lvmbdra.py) shows and
rewrites that path directly in the image:

    python3 tools/hpux/lvmbdra.py show hpux.img
    python3 tools/hpux/lvmbdra.py setpath hpux.img 2/0/1.6.0 --save bdra-orig.bin

HP-UX's own way works too: boot into LVM maintenance mode (`hpux -lm` at
the `ISL>` prompt, or Advanced > Boot mode > LVM maint.) and run
`lvlnboot -R`.

To install HP-UX 11i, put the install CD or DVD in the CD-ROM slot and an
empty image of at least 4 GB at ID 6, then type `boot scsi.2` at
`BOOT_ADMIN>`. Use the installer's text interface on the graphics console,
and expect the installation to take several hours.

## What is in this repository

| path | what |
|---|---|
| `rtl/cpu/` | the PA-7100LC: pipeline, caches, TLB, FPU |
| `rtl/hp715.sv`, `rtl/mioc.sv`, `rtl/lasi/` | the machine: the memory and I/O controller and LASI |
| `rtl/artist/` | the Artist graphics |
| `rtl/scsi/` | the 53C710 and the SCSI disks and CD-ROM |
| `rtl/mister/` | the MiSTer side: SDRAM, DDR3, the EEPROM image |
| `tests/`, `verilator/` | the CPU test suites, the lockstep co-simulation, and the whole machine simulated on HP's ROM |
| `tools/` | ROM analysis, HP-UX disk and file-system tools, EEPROM images, board scripts |
| `docs/` | the OSD layout, and an analysis of HP's boot ROM (`reference/rom/`, with no HP code in it) |
| `sys/` | the Template_MiSTer framework, unchanged |
| `release/` | the core (`.rbf`) and an EEPROM image |

## Building

The core builds with Quartus 17.0.2 Lite from `HP9K715.qpf`.

The tests run under Verilator 5:

- `tests/run-cputest.sh` runs the CPU test suites.
- `tests/run-cosim.sh` runs the two CPU cores in lockstep and compares
  every instruction.
- `tests/run-rom.sh` runs HP's boot ROM on the simulated machine up to
  `BOOT_ADMIN>`. It needs your own copy of the ROM.

## Credits

- **Template_MiSTer** (`sys/`) comes from the MiSTer project (Sorgelig and
  contributors).
- The SDRAM mapping and the save-image handling follow the MiSTer Macintosh
  Quadra core.
- QEMU's hppa machine and SeaBIOS-hppa served as references.

## Licence

GPL-2.0-or-later, the same as the MiSTer template ([LICENSE](LICENSE)).
HP's firmware and HP-UX are HP's copyrighted software and are not covered by
this licence.

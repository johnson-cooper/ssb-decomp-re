# Super Smash Bros. 64 — native PlayStation 2 port

This directory tree adds a **native** PS2 build of the SSB64 decompilation. The
game's C code is compiled for the Emotion Engine with the PS2Build SDK; there
is no N64 CPU/RSP/RDP emulation. libultra is replaced by a thin PS2 platform
layer, and F3DEX2 display lists are translated to GS primitives at runtime.

The N64 matching build is unaffected: all PS2 code lives under `ps2/`, and
the few edits to game sources are behind `#ifdef PLATFORM_PS2` (or are
`AVOID_UB` fixes that are no-ops for the matching build).

Current state is tracked in [PS2_PORT_STATUS.md](PS2_PORT_STATUS.md).

## Legal

- **No game data is distributed.** `baserom.us.z64`, ROM dumps, the
  extracted `assets/`/`relocAssets/` trees, generated `build/` sources and the
  asset pack `SSB64.DAT` are all git-ignored and must never be committed. You
  need your own cartridge dump; the pack is built from it locally.
- The upstream decomp repository currently has **no LICENSE file**. Until it
  has one, this port's code (which is derived from it) carries no licence
  grant either; do not redistribute builds. Third-party components used at
  build/run time: ps2sdk, gsKit, dmaKit and the IOP modules shipped by
  PS2Build, each under its own open-source licence (see the packages). `ps2/ps2_linkfile.ld` is derived from the
  ps2sdk default linkfile (AFL 2.0, credited in the file).

## Building

Requirements: the PS2Build SDK (`ps2build` on PATH), Python 3, and for asset preparation the decomp's own
extraction prerequisites (GNU make, splat; see README.md). IDO is **not**
needed.

```bash
ps2/tools/prepare_assets.sh          # once per ROM / asset change -> ps2/build/bin/SSB64.DAT
cd ps2 && ps2build build             # -> ps2/build/bin/ssb64.elf
```

The normal `ps2build build` is the production configuration. For a hardware-debug
build, add `- PS2_DEBUG=1` to the `defines` of the `ssb_platform` target in
`ps2/ps2.yaml`. That enables the Select + R3 debug overlay, Select + L3
frame-clear diagnostic, Select + R1 row-coverage capture, and extra framebuffer
scan-out diagnostics. The final audio path uses the dedicated `ssb_audio` IOP
server rather than SDR/sdrdrv.

Copy `ssb64.elf` and `SSB64.DAT` to the same directory and run the ELF.
The boot layer derives the launch/data device from `argv[0]` and supports
`host:`, generic `massN:` BDM mounts, explicit USB, internal ATA/exFAT BDM,
MX4SIO, iLink, MMCE, APA/PFS HDD, UDPBD, UDPFS, memory card and `cdrom0:`.

Launches from `massN:` work from any launcher (OPL, RiptOPL, wLaunchELF,
launcHER, ...) and do not depend on the launcher's IOP surviving: whether it
does is launcher-specific (RiptOPL's "Reboot IOP", for example, leaves a bare
ROM IOP). The port halts any USB host controller the launcher left running,
resets the IOP exactly once, loads its own BDM + USB stack and finds the
`massN:` slot that holds the same relative path to `SSB64.DAT`. If the pack is
not on USB after 5 s, MX4SIO, iLink and ATA are added to the live stack
without another reset. Explicit transport identities such as `usb0:`,
`ata0:`, `mx4sio0:`, `ilink0:` and `udpbd:` are rebuilt the same way and
resolved to the `massN:` filesystem of that transport.

The asset pack can also live on a different device from the ELF:

```text
ssb64.elf --data=usb0:/SSB64/
ssb64.elf --data=mmce0:/SSB64/
ssb64.elf --data=hdd0:+OPL:pfs:/SSB64/
ssb64.elf --data=udpfs:/SSB64/
```

This is particularly useful when the launcher lives on a memory card, since
`SSB64.DAT` is much larger than a standard 8 MiB card. For cross-device
`--data=` use, prefer a typed transport such as `usb0:`, `ata0:`,
`mx4sio0:` or `ilink0:`; a generic `massN:` data path is located by probing
(USB first) as described above.

Network modes inherit
the PS2's address from `mc0:/SYS-CONF/IPCONFIG.DAT` or
`mc1:/SYS-CONF/IPCONFIG.DAT`. UDPBD/UDPFS use their legacy unauthenticated
LAN discovery protocols, so treat them as trusted-LAN transports rather than
Internet-facing services; after discovery this port binds data replies to the
selected peer. A bare `bdm:` path is rejected because it does not identify a
transport or an existing filesystem mount.

Testing in PCSX2 (Windows helpers, default Pad 1 keyboard bindings):

```bash
powershell -File ps2/tools/run_pcsx2.ps1 -Seconds 20       # boot + print the port's log lines
powershell -File ps2/tools/goto_1p_match.ps1              # drive the menus into a 1P match
powershell -File ps2/tools/screenshot_pcsx2.ps1 -Out shot.png
```

### Build layout (`ps2/ps2.yaml`)

| target | kind | contents |
|---|---|---|
| `ssb_platform` | ee_lib | `ps2/src/*`: platform, memory, storage, input, audio, renderer, libultra replacement |
| `ssb_game` | ee_lib | the decomp's game sources (`src/`) plus the libultra parts that are pure C (gu, sp sprites, audio synthesis, libc) |
| `ssb64` | ee | `ps2/src/boot/main.c`, linked with a generated linkfile and the IOP modules embedded |

Game code is compiled with `-DPLATFORM_PS2 -DNON_MATCHING -DAVOID_UB`,
`-fno-strict-aliasing -fwrapv -fsingle-precision-constant`. The two libraries
are linked twice to resolve their mutual references.

## Architecture

### Threads and messages (`ps2/src/ultra/`)
`OSThread` maps onto EE kernel threads (priority `127 - pri`, stacks from
pools). Message queues keep libultra semantics (waiter lists, blocking and
non-blocking send/receive) on top of `SleepThread`/`WakeupThread`. The VBlank
interrupt drives VI retrace events and display-buffer swaps. SP tasks
(`osSpTaskStartGo`) are dispatched to the renderer (gfx) or completed
immediately (audio), and post the same SP/DP events the game waits for.

### Renderer (`ps2/src/renderer/`)
- GS: NTSC 240p, 320×240, three 16-bit (CT16S) framebuffers + Z16S; the
  texture pool uses the rest of the 4 MiB VRAM (~3.4 MiB).
- `gbi.c` walks F3DEX2 display lists (matrices, lighting, clipping against
  near/guard planes, RDP state) and emits GS primitives into a GIF DMA chain,
  batching triangles by GS state. The colour combiner is evaluated per vertex
  as `k·T + c` and mapped to GS MODULATE; lerps by texel alpha use a two-pass
  decal. The RDP blender maps to GS ALPHA/TEST.
- `texcache.c` converts N64 textures to GS formats (CI4/CI8 → T4/T8 + CLUT,
  I/IA → T4/T8 with fixed CLUTs, RGBA16 → CT16, RGBA32/IA16 → CT32), handles
  TMEM odd-row interleave, 32-bit TMEM half-word addressing and mask/clamp
  wrap semantics, and keeps textures VRAM-resident with LRU eviction keyed by
  source address + content.
- `overlay.c`: debug overlay (Select + R3): FPS, GBI time, memory by
  category, VRAM, texture and packet stats.

### Memory (`ps2/src/memory/`)
Every platform allocation is tracked per category (`ps2_mem_*`). The N64
game allocates scene heaps from "end of overlay BSS"; on PS2 all overlay
linker symbols resolve to one 6 MiB scene arena (`arena_glue.S`, generated
by `gen_arena_glue.py`) followed by the framebuffer block the game expects.

All N64 code overlays are linked statically. To keep N64 semantics, the
generated linkfile groups each overlay's `.data`/`.bss`; `syDmaLoadOverlay`
restores the overlay's initial data image and clears its bss, exactly what
the N64 DMA reload did (`overlay_state.c`).

### Assets (`ps2/tools/`, `ps2/src/storage/assets.c`)
The game reads everything through PI DMA from ROM addresses. `SSB64.DAT`
contains *virtual ROM regions*; `osPiStartDma` becomes a region lookup + file
read (a small metadata part is resident).

- relocData (2132 files): the decomp's typed sources are compiled natively
  with the EE gcc (o32 ABI, so layout matches IDO), sizes are checked against
  the ROM, and the ELF relocations are re-encoded into the game's reloc-chain
  format, so `lbreloc.c` runs unmodified.
- Endianness policy: everything the CPU reads is native (little-endian);
  **texel data stays in N64 byte order** (the converter reads it as such);
  **palettes are native u16**. `build_assets.py` fixes the places where the
  decomp's typing does not match how the data is used (mistyped script blocks,
  AnimJoint animations typed as u16 figatree data, u16/u32-typed texels,
  palettes inside u8 blobs, sequence/bank headers).
- Bitfields read from integer-encoded scripts are reversed for LE by
  `le_bitfields.py`; C-initialised structs keep declaration order.

Stage 1 (`n64prep.mk`) runs the decomp's own extractor without IDO; stage 2
(`build_assets.py`) compiles and packs. `prepare_assets.sh` runs both.

### Audio (`ps2/src/audio/`, `ps2/src/game/ps2_synth.c`, `ps2/tools/spu_samples.py`)
No RSP audio microcode runs and nothing is mixed on the EE. The game's own
sequence player and sound-effect engine run unchanged and drive libultra's
synthesizer voice API (`n_alSyn*`); only the bottom layer, `n_alAudioFrame`,
is replaced. It calls the players back at their sample times like the N64
driver, then turns the queued voice updates (start, pitch, volume ramp, pan,
FX mix, stop) into SPU2 voice registers: each of the game's 16 physical
voices is one voice of SPU2 core 1 (dry output).

Samples: all 439 wavetables of the two instrument banks are decoded from
VADPCM and re-encoded to PS-ADPCM offline. Loops are block-aligned (silence
prepended) and the loop body is unrolled and resampled by a factor ~1 so it
spans whole 28-sample blocks; the runtime multiplies the pitch by that
factor. The 4.1 MiB set lives in EE RAM and samples are copied to SPU RAM
(1.9 MiB cache, LRU) on first use. All register changes of a frame go to the
IOP as one `sceSdProcBatch` via sdrdrv.

### Input (`ps2/src/input/pad.c`)
libpad + libmtap, up to four players (multitap on port 1, or port 1 + a
multitap on port 2). One mapping table (Cross = A, Square = B,
Triangle/Circle = C-up/C-right, L1/R2 = Z, R1 = R, L2 = L, right stick = C
buttons). The left stick is rescaled to the N64 range with a dead zone.
Rumble maps to the DualShock small motor.

### Saves (`ps2/src/storage/save.c`)
The 32 KiB SRAM image is kept in RAM and flushed to the memory card by a
low-priority thread, debounced. Two slots (A/B) with sequence number + CRC;
the newest valid slot wins at boot, so a torn write never loses the previous
save. Without a formatted card, saves stay in RAM.

### Boot (`ps2/src/platform/boot.c`, `iop.c`, `storage/bootpath.c`)
Launch identity, filesystem identity and asset location are separate. The
default data directory is beside the ELF, while `--data=<directory>` can
select another device.

The IOP policy is deliberately transport-aware:

- `host:` and bare inherited `pfsN:` mounts are kept alive because
  resetting the IOP would destroy information that `argv[0]` does not
  contain.
- Every other path resets the IOP exactly once. Before that reset, a small
  module (`ssb_usb_quiesce`) halts any OHCI USB controller the launcher left
  running: PS2SDK's usbd does not stop it on IOP reboot, so it would keep
  DMAing into IOP RAM while the new kernel loads, which hung the reset from
  some launchers. No path resets the IOP again once its USB stack is up;
  on hardware the stick never came back after a second reset.
- Generic `massN:` is a connection-order alias, not a transport. It is rebuilt
  as USB first and resolved by finding the slot that holds `SSB64.DAT`;
  MX4SIO/iLink/ATA are added to the live stack after 5 s if needed.
- Explicit USB, ATA/exFAT, MX4SIO, iLink, UDPBD, UDPFS, MMCE, APA/PFS and
  optical paths can be reconstructed from embedded drivers after a clean IOP
  reset.
- Typed BDM identities are resolved back to the matching `massN:` filesystem
  by verifying both `SSB64.DAT` and the BDM driver's transport token. This
  avoids assuming that BDM slot numbers and transport unit numbers are the
  same.
- Unknown paths are rejected; there is no "unknown means USB" fallback.

The embedded IOP stacks are:

| data path | IOP stack / handling |
|---|---|
| `host:` | inherited ps2link/PCSX2 filesystem |
| `massN:` | bdm + bdmfs_fatfs + usbd_mini + usbmass_bd_mini; after 5 s without the pack, + iLink, ATA, MX4SIO |
| `usbN:` | bdm + bdmfs_fatfs + usbd_mini + usbmass_bd_mini |
| `ataN:` | ps2dev9 + bdm + bdmfs_fatfs + BDM-enabled ps2atad |
| `mx4sioN:` | bdm + bdmfs_fatfs + mx4sio_bd |
| `ilinkN:` | bdm + bdmfs_fatfs + iLinkman + IEEE1394_bd |
| `udpbd:` | ps2dev9 + bdm + bdmfs_fatfs + SUDPBDv2 SMAP/UDPBD |
| `udpfs:` | ps2dev9 + UDPFS SMAP + ministack + udpfs_ioman |
| `hdd0:<partition>:pfs:/...` | ps2dev9 + ps2atad + ps2hdd + ps2fs, then mount on `pfs0:` |
| `pfsN:` | inherited PFS mount (partition identity is not recoverable from a bare PFS path) |
| `mmceN:` | mmceman |
| `mcN:` | base mcman/mcserv stack |
| `cdrom0:` | cdfs, with ISO9660 `;1` fallback for `SSB64.DAT` |

Real-device files are read in bounded 64 KiB requests and the boot path allows
up to roughly 20 seconds for asynchronous BDM/network media to become ready.

### Game-source changes
- `include/PR/rcp.h`: register reads/writes go through the platform layer.
- `include/PR/guint.h` + `src/libultra/gu/sinf.c`, `cosf.c`: `DU(hi, lo)`
  initialisers so the double constants are right on little-endian (identical
  initialisers on N64).
- `src/libultra/n_audio/n_env.c`: `n_alAudioFrame` excluded on PS2.
- `apply_ps2_guards.py`, `fix_missing_returns.py`, `le_bitfields.py`:
  scripted, reviewable edits (colour packing, top-of-RAM assumptions,
  unsigned-loop UB, missing returns, anti-tamper checks, LE bitfields).
- `src/ft/ftchardata.h`, `src/sys/dma.c` (overlay reset hook) and a few
  one-line fixes.

## Debugging

`ps2_log` output goes to the PCSX2 console log (`logs/emulog.txt`). These
globals can be poked from the PCSX2 debugger (addresses via `nm` on
`ps2/build/obj/ssb64/ssb64.unstripped.elf`):

| variable | effect |
|---|---|
| `gPS2GbiTrace = N` | log the next N GBI commands |
| `gPS2TexTrace = N` + `gPS2TexFlush = 1` | re-convert all textures and log N conversions with tile state |
| `gPS2CombTrace = 1` | log each distinct textured combiner setup once |
| `gPS2TlutTrace = N` | log N CI texture binds with the commands that loaded texel/TLUT data |

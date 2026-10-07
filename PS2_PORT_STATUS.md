# PS2 port — status

Last updated 2026-10-04. PCSX2 remains the reference environment unless a
hardware result is called out explicitly. A first real-PS2 USB run reached GS,
IOP/storage setup, scene/overlay allocation and controller initialization, then
stalled before the asset manager completed. That exposed that the original
device layer had only been exercised through `host:`; the current branch
replaces it with a transport-aware launch/data layer. The new device work still
needs the next hardware run for confirmation. See [PS2_PORT.md](PS2_PORT.md)
for the design.

## Works (verified in PCSX2)

| area | notes |
|---|---|
| Build | `ps2build build` produces `ssb64.elf`; `prepare_assets.sh` pipeline builds `SSB64.DAT` (2309 regions, 25 MiB incl. 4.1 MiB of PS-ADPCM samples) from the US ROM |
| Boot (PCSX2) | IOP modules, VBlank, GS, threads, asset pack from `argv[0]` directory (`host:`) |
| Front end | "No Controller" check, Nintendo/HAL logo, full opening movie, title, Mode Select, 1P Game character select (portraits, fighter model, options) |
| Scene changes | leaving and re-entering scenes (overlay `.data`/`.bss` reset on load) |
| Gameplay | 1P Game stage 1 (Mario vs CPU Link, Hyrule Castle): entry animations, HUD, timer, damage accumulation and knockback, CPU opponent, player movement/attacks from the keyboard-mapped pad |
| Audio (driver side) | SPU2 backend active: music and sound-effect voices start with the right samples and plausible pitches; the SPU2 reports sounding voices with advancing play addresses. **Not yet checked by ear** (see below) |
| Rendering | textured/lit 3D, sprites, CI4/CI8/I/IA/RGBA16/RGBA32 textures, clipping, fog, alpha blend/test, depth |
| Memory | 17.3 MB committed at boot and in the match (budget 24 MB), of which 4.1 MB is the PS-ADPCM sample set; scene arena 1.5 of 6 MB used in the 1P match; peak equals steady state so far. SPU RAM: ~1 of 1.9 MB sample cache used in a match, no evictions yet |
| Debug overlay | Select + R3 (debug builds: `PS2_DEBUG=1`) |
| Frame-clear diagnostic | debug builds: Select + L3 cycles off → magenta → black: clears each framebuffer before the game draws into it, so rows the game leaves undrawn show in that colour (for the flickering line seen on hardware) |
| Row-coverage capture | debug builds: Select + R1: for 3 frames, records every primitive drawn and logs the framebuffer rows none of them covers (GS rule), with the neighbouring primitives, then saves SSB64.LOG |

## Measured performance (PCSX2, 1P match on Hyrule)

- ~31 FPS on the debug overlay.
- GBI translation ~17–18 ms per frame on the EE (3200 commands, ~470
  triangles). This is the main cost; menus take 1–2 ms.
- Texture VRAM pool (3.4 MiB) is full in the match; evictions stop once the
  match is running (steady working set).

The frame rate the N64 original achieves in this scene has not been measured
for comparison.

## Not working / not implemented

- **Audio effects**: reverb (AL_FX) is not mapped to the SPU2 effect unit;
  the game's default settings use AL_FX_NONE, but any scene that enables
  reverb plays dry.
- **Performance** is below target: no static display-list caching, no VU1
  transform path, combiner evaluated per vertex on the EE.
- **CPU framebuffer writes** (staff roll, congratulations screens) are not
  mirrored to the GS; those screens will be wrong.
- **Real-device validation of the new storage layer** is pending. The code now
  contains explicit stacks/path handling for USB, generic BDM `massN:`,
  ATA/exFAT, MX4SIO, iLink, MMCE, APA/PFS HDD, UDPBD, UDPFS, memory card and
  optical media, but these paths must not be called hardware-verified until
  they have been exercised on-console.

## Device-layer implementation (2026-10-04)

The storage/launch rewrite is implemented on
`fix/full-ps2-device-support`:

- launch device and data device are separate; `--data=<directory>` supports
  cross-device asset placement;
- generic `massN:` paths halt the launcher's USB controller, reset the IOP
  once, rebuild BDM + USB (other BDM transports after 5 s) and locate the slot
  holding `SSB64.DAT` (hardware-verified on USB, 2026-10-06);
- explicit `usb/ata/mx4sio/ilink/udpbd` identities load the correct transport
  and resolve to the matching `massN:` filesystem containing `SSB64.DAT`;
- unknown/bare `bdm:` identities are rejected rather than silently routed to
  USB;
- APA/PFS paths retain the partition identity long enough to rebuild and mount
  `pfs0:`; bare inherited `pfsN:` paths are preserved instead of reset;
- UDPBD and UDPFS use their separate pinned network stacks and read the PS2 IP
  from `mc?:/SYS-CONF/IPCONFIG.DAT`;
- optical reads have a `;1` ISO9660 fallback;
- real-device reads are chunked to 64 KiB and asynchronous media readiness gets
  a ~20-second window;
- the real-hardware one-slot multitap false positive seen in the first run is
  rejected before controller-slot remapping.

This section records **implementation status**, not hardware verification.

## Known visual issues

- Character select: the scrolling "Ready to fight" banner is garbled.
- 2 unknown GBI commands per frame in the match (skipped).
- Title/other scenes: the Yoshi heart / blocky cloud issue seen earlier was
  not re-checked after the texture fixes.

## Not yet verified

- **Audio by ear.** The SPU2 path was verified through the driver's own
  statistics and SPU2 register read-back only (no listening test was
  possible in this environment). Things to listen for: overall loudness
  against the N64, loop seams on sustained instruments, pitch of looped
  samples (loops are resampled by up to 0.5 % to fit 28-sample blocks), and
  per-frame (1/60 s) granularity of volume/pan changes.
- Physics after the sinf/cosf fix: verified in a 1P match (damage accumulates,
  hits no longer launch fighters off-screen); VS mode and Kirby's specials,
  where the problem was reported, not re-tested yet.

- The rebuilt real-device layer on hardware: USB/generic BDM first, then
  ATA/exFAT, MX4SIO, iLink, MMCE, APA/PFS, UDPBD/UDPFS, memory card and optical
  paths. The prior USB build reached controller initialization but did not
  complete the asset-manager transition.
- Memory card saves: the save path is implemented (A/B slots, CRC, debounced
  flush) but the test setup had no formatted card, so saves stayed in RAM.
- Multitap / 4 players; DualShock rumble.
- VS mode, other stages and characters, a full 1P run, bonus stages, ending.
- The texel-alpha two-pass decal combiner path (no scene seen so far uses it).
- relocData file 200 (`SYSignValidate`, anti-tamper data) is 40 bytes larger
  natively than in the ROM; the anti-tamper checks are disabled on PS2, so it
  is not read, but this is unconfirmed for all scenes.

## Next steps (in priority order)

1. Listening test of the SPU2 audio; SPU2 reverb for AL_FX settings.
2. Performance: cache translated static DLs, move transforms to VU1.
3. Fix the character-select banner and the unknown GBI commands.
4. Verify VS mode, all stages/characters, memory card saves, multitap.
5. GS mirroring for CPU framebuffer writes.
6. Hardware validation of each launch/data transport, starting with the USB
   setup that exposed the original stall.

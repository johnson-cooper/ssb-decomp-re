/*
 * PS2 entry point.
 *
 * Brings up the platform services in dependency order, then hands over to
 * the game's own boot path (syMainLoop, the N64 "thread 0" code), which
 * creates the idle/main/scheduler/audio/controller threads exactly as on the
 * N64 - through the libultra layer in ps2/src/ultra/.
 */
#include <ps2/platform.h>

#include <fcntl.h>
#include <kernel.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

/* Game side (src/sys/main.c): the N64 boot thread's code. */
extern void syMainLoop(void);

/* Platform services implemented elsewhere. */
extern void ps2_gs_init(void);
extern void ps2_gs_boot_screen(const char *title);
extern void ps2_ultra_threads_init(void);
extern void ps2_vi_init(void);
extern void ps2_input_init(void);
extern int ps2_assets_init(void);
extern void ps2_save_init(void);
extern void ps2_audio_init(void);
extern void ps2_render_thread_init(void);
extern void ps2_arena_init(void);
extern void ps2_overlay_state_init(void);

#define PS2_BOOT_TITLE "Super Smash Bros. 64 - PS2 native port [UNIFIED-RECOVERY]"

int ps2_main(int argc, char *argv[])
{
    const PS2MemStats *mem;
    int bad_data_arg = 0;
    int i;

    ps2_log_init();
    ps2_crash_init();
    ps2_storage_set_boot_path((argc > 0) ? argv[0] : NULL);

    /* Optional explicit asset/log directory. This is deliberately a directory
     * rather than a device switch: e.g. an ELF may live on mc0: while the
     * large SSB64.DAT lives on mass0:, udpfs:, MMCE or PFS. */
    for (i = 1; i < argc; i++)
    {
        if (argv[i] != NULL && strncmp(argv[i], "--data=", 7) == 0)
        {
            if (!ps2_storage_set_data_path(argv[i] + 7))
                bad_data_arg = 1;
        }
    }
    /* Stage colours (troubleshooting on hardware, see PS2_PORT.md): navy =
     * started, half-intensity colours subdivide IOP reset/module bring-up,
     * blue enters GS init, and after GS init the boot log is shown as text. */
    ps2_boot_stage("started", 0x000080);
    ps2_mem_init();

    /* Boot with a high priority so init is not preempted by game threads
     * that get created along the way. */
    ChangeThreadPriority(GetThreadId(), 2);

    ps2_log("boot: argv0=%s", (argc > 0 && argv[0] != NULL) ? argv[0] : "(none)");
    ps2_log("boot: launch=%s data=%s dir=%s",
            ps2_storage_device_name(ps2_storage_launch_device()),
            ps2_storage_device_name(ps2_storage_data_device()),
            ps2_storage_boot_dir());

    {
        /* Display lists address memory through RSP segments; a real EE
         * pointer at or above 16 MiB would alias segment 1. Everything the
         * game can reference (the ELF image incl. the scene arena) must stay
         * below that line; platform-only buffers may live above it. */
        extern char _end[];

        if ((uintptr_t)_end >= 0x01000000u)
        {
            ps2_panic("ELF image ends at %p, above the 16 MiB segment limit", _end);
        }
    }

    ps2_boot_stage("IOP reset + modules", 0x800080);
    ps2_iop_init();
    /* Initialize the GS before installing the game-side VBlank ISR. gsKit
     * reprograms GS state during bring-up, so no VBlank callback should run
     * across that hardware transition. */
    ps2_boot_stage("GS video init", 0x0000FF);
    ps2_gs_init();
    ps2_vblank_init();
    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    if (bad_data_arg)
        ps2_panic("invalid --data path; use a supported device directory (for example mass0:/SSB64/)");

    if (ps2_storage_data_device() == PS2_BOOT_UNKNOWN)
        ps2_panic("unsupported or ambiguous launch/data path: %s", ps2_storage_launch_path());

    if (ps2_iop_load_boot_device_drivers(ps2_storage_data_device()) < 0)
        ps2_panic("failed to initialize %s storage", ps2_storage_device_name(ps2_storage_data_device()));

    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    ps2_ultra_threads_init();
    ps2_vi_init();
    ps2_arena_init();
    ps2_overlay_state_init();

    /*
     * Open and validate the asset stream before starting controller clients.
     * Every device keeps the same filesystem/backend for the lifetime of the
     * pack. Transient descriptor failures are recovered by reopening the same
     * path; storage is never switched underneath the game.
     */
    if (!ps2_assets_init())
    {
        ps2_panic("asset pack not found next to the ELF (%sSSB64.DAT). Run ps2/tools/prepare_assets.sh first.",
                  ps2_storage_boot_dir());
    }

    if (ps2_storage_data_device() != PS2_BOOT_CDROM)
    {
        ps2_log_enable_save(1);
        ps2_log_save();
    }

    ps2_input_init();
    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    /* Keep the on-screen boot log truthful. Previously the screen was only
     * redrawn before save/audio init and after both had completed, so a hang
     * inside either subsystem misleadingly left "input: ... ready" as the
     * final visible line. */
    ps2_log("boot: initializing save backend");
    ps2_gs_boot_screen(PS2_BOOT_TITLE);
    ps2_save_init();
    ps2_log("boot: save backend initialized");
    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    ps2_log("boot: scheduling audio backend");
    ps2_audio_init();
    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    ps2_render_thread_init();

    mem = ps2_mem_stats();
    ps2_log("mem: %u KiB committed (code/static %u KiB), budget %u KiB", (unsigned)(mem->total_used >> 10),
            (unsigned)(mem->used[PS2_MEM_CODE_STATIC] >> 10), (unsigned)(mem->budget >> 10));
    ps2_gs_boot_screen(PS2_BOOT_TITLE);

    ps2_log("boot: starting game");
    ps2_log_save();
    /* syMainLoop creates the idle thread (libultra priority 127), which in
     * turn starts the game's main thread; this boot thread then just parks
     * at the lowest priority. */
    syMainLoop();
    ChangeThreadPriority(GetThreadId(), 127);
    for (;;)
    {
        SleepThread();
    }
    return 0;
}

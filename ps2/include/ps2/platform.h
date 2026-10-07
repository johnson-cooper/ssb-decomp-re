/*
 * ps2/include/ps2/platform.h
 *
 * Internal API shared between the PS2 platform modules (everything under ps2/src).
 * Game code never includes this header directly; it only sees the
 * libultra-shaped API (PR/os.h etc.), which ps2/src/ultra/ implements on
 * top of these services.
 */
#ifndef PS2_PLATFORM_H
#define PS2_PLATFORM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/* Build configuration                                                 */
/* ------------------------------------------------------------------ */

/* PS2_DEBUG = 1 builds the hardware-debugging aids in: the debug overlay
 * (Select + R3), the frame-clear and row-coverage diagnostics (Select + L3,
 * Select + R1), framebuffer scan-out counters, and SSB64.LOG written at
 * boot.  Production builds leave it 0 (the default); a crash or fatal
 * error still writes SSB64.LOG either way.  Enable it by adding
 * "- PS2_DEBUG=1" to the ssb_platform defines in ps2/ps2.yaml. */
#ifndef PS2_DEBUG
#define PS2_DEBUG 0
#endif

#ifndef PS2_DEBUG_OVERLAY
#define PS2_DEBUG_OVERLAY PS2_DEBUG /* compile the debug overlay in (toggle at runtime) */
#endif

/* ------------------------------------------------------------------ */
/* Logging (ps2/src/platform/log.c)                                    */
/* ------------------------------------------------------------------ */

void ps2_log_init(void);
void ps2_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
/* Fatal error: logs, shows the error on screen and halts. */
void ps2_panic(const char *fmt, ...) __attribute__((format(printf, 1, 2), noreturn));
/* Access the most recent log lines (for the boot/debug screen). */
int ps2_log_line_count(void);
const char *ps2_log_line(int i); /* 0 = oldest retained line */

/* ------------------------------------------------------------------ */
/* Memory accounting (ps2/src/memory/memstat.c)                        */
/* ------------------------------------------------------------------ */

typedef enum PS2MemCategory
{
    PS2_MEM_CODE_STATIC,  /* ELF text/data/bss (measured at boot)          */
    PS2_MEM_GAME_HEAP,    /* libultra-era general heaps (non-scene)        */
    PS2_MEM_SCENE_ARENA,  /* per-scene arena (N64 "overlay BSS end..")     */
    PS2_MEM_FIGHTER,      /* fighter / common asset pools                   */
    PS2_MEM_GFX_STAGING,  /* GIF packet / DMA buffers                       */
    PS2_MEM_TEXTURE_CACHE,/* EE-side converted texture cache               */
    PS2_MEM_AUDIO,        /* audio buffers / sample staging                 */
    PS2_MEM_SCRATCH,      /* temporary (file I/O, conversion)               */
    PS2_MEM_THREADS,      /* EE thread stacks                               */
    PS2_MEM_CATEGORY_COUNT
} PS2MemCategory;

typedef struct PS2MemStats
{
    uint32_t used[PS2_MEM_CATEGORY_COUNT];
    uint32_t peak[PS2_MEM_CATEGORY_COUNT];
    uint32_t reserved[PS2_MEM_CATEGORY_COUNT]; /* capacity of fixed pools */
    uint32_t total_used;
    uint32_t total_peak;
    uint32_t ram_size;   /* 32 MiB retail EE RAM */
    uint32_t budget;     /* engineering target (24 MiB) */
} PS2MemStats;

void ps2_mem_init(void);
/* Fixed-pool bookkeeping: `reserve` records a pool's capacity (counts as
 * used memory, since it is statically committed); `use` records how much of
 * that pool is currently occupied. */
void ps2_mem_reserve(PS2MemCategory cat, uint32_t bytes);
void ps2_mem_reclassify_static(PS2MemCategory cat, uint32_t bytes);
void ps2_mem_set_used(PS2MemCategory cat, uint32_t bytes);
void ps2_mem_add(PS2MemCategory cat, int32_t delta);
const PS2MemStats *ps2_mem_stats(void);
const char *ps2_mem_category_name(PS2MemCategory cat);
/* Tracked, aligned allocations for platform subsystems (never per-frame). */
void *ps2_mem_alloc(PS2MemCategory cat, uint32_t size, uint32_t align);
void ps2_mem_free(PS2MemCategory cat, void *p, uint32_t size);

/* ------------------------------------------------------------------ */
/* Timing (ps2/src/platform/timing.c)                                  */
/* ------------------------------------------------------------------ */

uint64_t ps2_time_ticks(void);           /* EE bus clock ticks (147.456 MHz) */
uint32_t ps2_time_us(void);              /* microseconds, wraps */
uint32_t ps2_vblank_count(void);         /* VBlank-start interrupts since boot */
void ps2_vblank_init(void);
/* Register the libultra VI event (osViSetEvent) target. */
struct OSMesgQueue_s;
void ps2_vblank_set_event(struct OSMesgQueue_s *mq, void *msg, uint32_t retrace_count);

/* ------------------------------------------------------------------ */
/* Boot device / filesystem (ps2/src/storage/)                         */
/* ------------------------------------------------------------------ */

typedef enum PS2BootDevice
{
    PS2_BOOT_UNKNOWN,
    PS2_BOOT_HOST,       /* host: (ps2link / PCSX2 host fs) */
    PS2_BOOT_BDM,        /* generic massN: (transport found by probing) */
    PS2_BOOT_USB,        /* explicit usbN: transport identity */
    PS2_BOOT_MC,         /* mc0:/mc1: */
    PS2_BOOT_ATA,        /* ata: BDM (internal/exFAT) */
    PS2_BOOT_MX4SIO,     /* mx4sio: BDM */
    PS2_BOOT_ILINK,      /* ilink: BDM */
    PS2_BOOT_UDPBD,      /* udpbd: network block device */
    PS2_BOOT_UDPFS,      /* udpfs: network filesystem */
    PS2_BOOT_HDD,        /* APA/PFS hdd0:<partition>:pfs:/ */
    PS2_BOOT_MMCE,       /* mmce0:/mmce1: */
    PS2_BOOT_CDROM       /* cdrom0: */
} PS2BootDevice;

/* argv[0] selects the launch device. By default assets/logs live beside the
 * ELF, but --data=<directory> can select another supported device (required
 * for practical mc: launches because SSB64.DAT is larger than an 8 MiB card). */
void ps2_storage_set_boot_path(const char *argv0);
int ps2_storage_set_data_path(const char *path);
PS2BootDevice ps2_storage_launch_device(void);
PS2BootDevice ps2_storage_data_device(void);
/* Backwards-compatible alias used by older platform code: returns data device. */
PS2BootDevice ps2_storage_boot_device(void);
const char *ps2_storage_launch_path(void);
const char *ps2_storage_hdd_mount_source(void);
/* True when the selected data path depends on an inherited IOP filesystem
 * (host:, or a bare pfsN: mount whose transport/source cannot be
 * reconstructed from argv[0]). */
int ps2_storage_requires_iop_preserve(void);
/* Resolve BDM launch paths (generic massN: and typed usb/ata/mx4sio/ilink/
 * udpbd identities) to the actual massN: filesystem that contains
 * probe_name. */
int ps2_storage_resolve_data_root(const char *probe_name);
int ps2_video_progressive(void);         /* 1 = 240p (ELF name contains "240p"), 0 = 480i */

/* Boot-stage marker: shows a solid background colour (GS BGCOLOR with both
 * display circuits off) so a hang before any graphics identifies the stage.
 * Also mirrored into the log. See PS2_PORT.md "Troubleshooting". */
void ps2_boot_stage(const char *name, uint32_t rgb);

/* Installs the EE exception handlers that turn a crash into a register dump
 * on screen (and into the log file when possible). */
void ps2_crash_init(void);

/* Writes the in-memory log to <boot dir>SSB64.LOG (boot device drivers
 * must be loaded). Safe to call repeatedly; each call rewrites the file. */
void ps2_log_save(void);
void ps2_log_enable_save(int enable);    /* once the boot device is usable */
void ps2_log_console(int enable);        /* printf mirroring (off in the crash handler) */
const char *ps2_storage_boot_dir(void);   /* e.g. "mass0:/SSB64/" */
const char *ps2_storage_device_name(PS2BootDevice dev);
/* Build "<boot dir><name>" into out. */
void ps2_storage_path(char *out, size_t out_size, const char *name);

/* Simple blocking file API over fileXio / fio (platform-internal). */
int ps2_file_open_read(const char *path);
int ps2_file_read(int fd, void *dst, uint32_t size);
int ps2_file_seek(int fd, uint32_t offset);
int ps2_file_size(int fd);
int ps2_file_mmce_enter_runtime_stream(int fd);
void ps2_file_close(int fd);

/* ------------------------------------------------------------------ */
/* IOP modules (ps2/src/platform/iop.c)                                */
/* ------------------------------------------------------------------ */

void ps2_iop_init(void);                 /* reset IOP + load base modules */
int ps2_iop_load_boot_device_drivers(PS2BootDevice dev);
/* massN: only: add non-USB BDM transports to the live stack (once, no reset). */
int ps2_iop_load_bdm_fallback_transports(void);
int ps2_iop_load_audio_driver(void); /* deferred SDR server; safe to call after storage/input */
int ps2_iop_mmce_prepare_runtime_stream(void);
int  ps2_iop_load_audio_drivers(void);    /* libsd + sdr; 0 on success */
int  ps2_iop_module_loaded(const char *name);
int  ps2_iop_module_count(void);
const char *ps2_iop_module_name(int i);

#ifdef __cplusplus
}
#endif

#endif /* PS2_PLATFORM_H */

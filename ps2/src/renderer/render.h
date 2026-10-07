/*
 * ps2/src/renderer/render.h - renderer-internal interfaces.
 *
 * Game-facing entry points are the libultra calls (osSpTaskStart, VI);
 * this header is shared by gs.c (GS setup, VRAM, DMA packets), gbi.c (the
 * F3DEX2/RDP translator), texcache.c (texture conversion + VRAM residency)
 * and overlay.c (debug overlay). It uses only <stdint.h> types so it can be
 * included from translation units on either side of the ultratypes/tamtypes
 * split (see ps2/include/ps2/ee_shim.h).
 */
#ifndef PS2_RENDER_H
#define PS2_RENDER_H

#include <stdint.h>
#include "gsregs.h"

/* ------------------------------------------------------------------ */
/* Screen / VRAM layout                                                 */
/* ------------------------------------------------------------------ */

#define PS2_SCREEN_W 320
#define PS2_SCREEN_H 240
#define PS2_FB_COUNT 3          /* matches the game's gSYFramebufferSets[3] */
#define PS2_FB_PSM GSPSM_CT16S
#define PS2_Z_PSM GSPSM_Z16S
#define PS2_FBW (PS2_SCREEN_W / 64) /* 5 */

/* VRAM is addressed in 256-byte blocks (TBP/CBP units); a page is 32 blocks
 * (8 KiB) and FBP/ZBP are in page units. */
#define PS2_VRAM_BLOCKS 16384   /* 4 MiB */
#define PS2_PAGE_BLOCKS 32
#define PS2_FB_PAGES 20         /* 320x240 at 16 bpp = 5 x 4 pages */
#define PS2_FB_PAGE(i) ((i) * PS2_FB_PAGES)
#define PS2_Z_PAGE (PS2_FB_COUNT * PS2_FB_PAGES)             /* 60 */
#define PS2_FONT_BLOCK ((PS2_Z_PAGE + PS2_FB_PAGES) * PS2_PAGE_BLOCKS) /* 2560: T4 128x64 */
#define PS2_FONT_CLUT_BLOCK (PS2_FONT_BLOCK + 16)            /* 2576 */
#define PS2_TEX_POOL_FIRST_BLOCK (PS2_FONT_BLOCK + 32)       /* 2592 (page aligned) */
#define PS2_TEX_POOL_BLOCKS (PS2_VRAM_BLOCKS - PS2_TEX_POOL_FIRST_BLOCK)

/* ------------------------------------------------------------------ */
/* Packet building (gs.c)                                               */
/* ------------------------------------------------------------------ */

/* The frame is built as a GIF DMA source chain in a pair of persistent,
 * 64-byte aligned buffers. CNT segments carry inline GIF data; REF segments
 * point straight at converted texture memory so texel data is never copied.
 * When a buffer fills up it is kicked while the other one is filled. */
typedef struct PS2Packet
{
    uint64_t *base;
    uint64_t *ptr;   /* next free u64 */
    uint64_t *end;   /* keep space for closing tags */
    uint64_t *cnt;   /* open CNT tag */
} PS2Packet;

extern PS2Packet gPS2Pkt;

void ps2_gs_init(void);
void ps2_pkt_reserve(uint32_t qwords); /* ensure space; may kick + switch buffers */
void ps2_pkt_flush(void);              /* kick what is queued (non-blocking) */
void ps2_pkt_finish(void);             /* kick + wait for GS completion */
void ps2_pkt_ref(const void *data, uint32_t qwc); /* append a REF segment */

/* A+D register writes: open a batch of n writes then add them. */
static inline void ps2_pkt_ad_begin(uint32_t n)
{
    uint64_t *p;

    ps2_pkt_reserve(n + 1);
    p = gPS2Pkt.ptr;
    p[0] = GIFTAG_LO(n, 0, 0, 0, GIF_FLG_PACKED, 1);
    p[1] = GSR_AD;
    gPS2Pkt.ptr = p + 2;
}

static inline void ps2_pkt_ad(uint64_t reg, uint64_t data)
{
    uint64_t *p = gPS2Pkt.ptr;

    p[0] = data;
    p[1] = reg;
    gPS2Pkt.ptr = p + 2;
}

/* Framebuffer / display (gs.c) */
int ps2_gs_fb_index_for(const void *n64_fb);   /* -1 if not a known framebuffer */
int ps2_gs_is_zbuffer(const void *n64_addr);
void ps2_gs_frame_setup(int fb_index);          /* FRAME/ZBUF/offset/scissor defaults */
void ps2_gs_clear(int fb_index, uint32_t rgba, int clear_z);
void ps2_gs_present_now(int fb_index);          /* direct display switch (boot screen) */

/* Debug text (gs.c): 8x8 font, coordinates in 320x240 screen pixels. */
void ps2_gs_text(int x, int y, uint32_t rgba, const char *s);
void ps2_gs_rect(int x0, int y0, int x1, int y1, uint32_t rgba); /* alpha-blended */

/* Boot / status screen (drawn outside of any game frame). */
void ps2_gs_boot_screen(const char *title);

/* Per-frame statistics for the overlay. */
typedef struct PS2RenderStats
{
    uint32_t triangles;
    uint32_t rects;
    uint32_t batches;       /* GIF tags emitted for primitives */
    uint32_t state_writes;  /* A+D register writes */
    uint32_t tex_uploads;
    uint32_t tex_upload_bytes;
    uint32_t tex_converts;
    uint32_t dl_commands;
    uint32_t kicks;
    uint32_t packet_bytes;
    uint32_t gfx_us;        /* last gfx task translate+draw time */
    uint32_t unknown_cmds;
} PS2RenderStats;

extern PS2RenderStats gPS2RenderStats;      /* current frame (being built) */
extern PS2RenderStats gPS2RenderStatsLast;  /* last completed frame */

/* ------------------------------------------------------------------ */
/* Texture cache (texcache.c)                                           */
/* ------------------------------------------------------------------ */

typedef struct PS2TexKey
{
    const void *addr;      /* N64 texel source (typed relocData / heap) */
    const void *tlut;      /* palette source for CI, else NULL */
    uint16_t width, height;/* source wrap/mirror period in texels */
    uint16_t clamp_width, clamp_height; /* 0 = one period; else materialize this legal tile extent */
    uint16_t line_bytes;   /* source row pitch in bytes */
    uint8_t fmt, siz;      /* G_IM_FMT_*, G_IM_SIZ_* */
    uint8_t tlut_type;     /* 0 none, 2 RGBA16, 3 IA16 (G_TT_* >> 14) */
    uint8_t pal_index;     /* CI4 palette bank */
    uint8_t mirror_s, mirror_t;
    uint8_t odd_swap;      /* odd rows stored with 32-bit halves swapped (TMEM order) */
} PS2TexKey;

typedef struct PS2TexBinding
{
    uint64_t tex0;         /* TEX0 without TFX/TCC */
    uint16_t gs_w, gs_h;   /* uploaded texture size (texels) */
} PS2TexBinding;

void ps2_texcache_init(void);
/* Ensure the texture is converted and VRAM-resident; appends the upload to
 * the current packet if needed. Returns 0 on failure (unsupported format). */
int ps2_texcache_bind(const PS2TexKey *key, PS2TexBinding *out);
void ps2_texcache_frame_begin(void);    /* safe point: previous frame's DMA done */
void ps2_texcache_invalidate_all(void); /* scene transition */

typedef struct PS2TexStats
{
    uint32_t entries;
    uint32_t ee_bytes, ee_capacity, ee_peak;
    uint32_t vram_blocks_used, vram_blocks_peak;
    uint32_t evictions;
} PS2TexStats;
const PS2TexStats *ps2_texcache_stats(void);

/* VRAM block allocator for the texture pool (texcache.c). */
uint32_t ps2_vram_footprint_blocks(int psm, int w, int h, int tbw);

/* ------------------------------------------------------------------ */
/* GBI translator (gbi.c)                                               */
/* ------------------------------------------------------------------ */

void ps2_gbi_init(void);
/* Execute an F3DEX2 display list for a gfx task. */
void ps2_gbi_run(const void *dl);
/* True if the last run targeted a framebuffer (sets *fb_index). */
int ps2_gbi_last_color_target(void);

/* ------------------------------------------------------------------ */
/* Debug overlay (overlay.c)                                            */
/* ------------------------------------------------------------------ */

void ps2_overlay_draw(void);
int ps2_overlay_enabled(void);
void ps2_overlay_toggle(void);
void ps2_overlay_frame_presented(void); /* FPS accounting */
void ps2_overlay_set_scene_name(const char *name);

#endif /* PS2_RENDER_H */

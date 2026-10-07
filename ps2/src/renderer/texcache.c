/*
 * Texture cache: N64 texel formats -> GS formats, and GS VRAM residency.
 *
 * Conversion targets keep textures small (the originals are tiny and mostly
 * palettized; GS VRAM is only 4 MiB):
 *
 *   N64            GS                        notes
 *   CI4 / CI8      PSMT4 / PSMT8 + CLUT      palette -> CT32 CLUT (TLUT RGBA16 or IA16)
 *   I4 / I8        PSMT4 / PSMT8 + fixed CLUT  intensity as color and alpha
 *   IA4 / IA8      PSMT4 / PSMT8 + fixed CLUT  I/A decoded through the CLUT
 *   RGBA16         PSMCT16                   5551 -> GS 1555 (TEXA maps alpha)
 *   RGBA32, IA16   PSMCT32
 *
 * Nothing is converted per frame: a texture is converted into a staging ring
 * only when it becomes VRAM-resident, uploaded by DMA straight from the ring
 * (REF tag, no copy), and stays resident until evicted (LRU) or the scene
 * changes. Keys are the N64 source pointers + format; a short content hash
 * guards against a scene reusing memory for different data.
 */
#include "render.h"

#include <ps2/platform.h>

#include <string.h>

#define MAX_ENTRIES 1536
#define HASH_SLOTS 4096
#define STAGING_BYTES (1024 * 1024)

/* N64 format codes (G_IM_FMT_* / G_IM_SIZ_*) */
#define FMT_RGBA 0
#define FMT_YUV 1
#define FMT_CI 2
#define FMT_IA 3
#define FMT_I 4
#define SIZ_4 0
#define SIZ_8 1
#define SIZ_16 2
#define SIZ_32 3

typedef struct TexEntry
{
    PS2TexKey key;
    uint32_t hash;
    uint32_t last_used;   /* frame number */
    uint32_t vram_block;  /* texture */
    uint32_t vram_blocks; /* texture + clut */
    uint32_t clut_block;  /* 0 = none */
    uint64_t tex0;
    uint16_t gs_w, gs_h;
    int16_t next_free;
    uint8_t in_use;
    uint8_t resident;
} TexEntry;

static TexEntry sEntries[MAX_ENTRIES];
static int16_t sHash[HASH_SLOTS]; /* entry index or -1 */
static int sFreeHead;
static uint32_t sFrame;
static PS2TexStats sStats;
int gPS2TexTrace; /* debug: set > 0 to log the next N texture conversions */
int gPS2TexJustConverted; /* debug: log the first N texture conversions */
int gPS2TexFlush; /* debug: set to drop all cached textures at the next frame */

/* Staging ring for converted texel + CLUT data awaiting DMA. */
static uint8_t *sStaging;
static uint32_t sStagingPos;

/* Fixed CLUTs for I/IA formats, resident at the start of the pool. */
static uint32_t sClutI4[16] __attribute__((aligned(64)));
static uint32_t sClutIA4[16] __attribute__((aligned(64)));
static uint32_t sClutI8[256] __attribute__((aligned(64)));
static uint32_t sClutIA8[256] __attribute__((aligned(64)));
static uint32_t sFixedClutBlock[4];
static uint32_t sPoolFirst;

/* ------------------------------------------------------------------ */
/* VRAM allocator: sorted list of used [start, end) block ranges.       */
/* ------------------------------------------------------------------ */

typedef struct VramRange
{
    uint32_t start, end;
    int16_t owner;
} VramRange;

static VramRange sRanges[MAX_ENTRIES];
static int sRangeCount;

static int vram_alloc(uint32_t blocks, int16_t owner, uint32_t *out)
{
    uint32_t prev_end = PS2_TEX_POOL_FIRST_BLOCK;
    int i;

    for (i = 0; i <= sRangeCount; i++)
    {
        uint32_t next_start = (i < sRangeCount) ? sRanges[i].start : PS2_VRAM_BLOCKS;

        if (next_start >= prev_end && next_start - prev_end >= blocks)
        {
            memmove(&sRanges[i + 1], &sRanges[i], sizeof(VramRange) * (size_t)(sRangeCount - i));
            sRanges[i].start = prev_end;
            sRanges[i].end = prev_end + blocks;
            sRanges[i].owner = owner;
            sRangeCount++;
            *out = prev_end;
            sStats.vram_blocks_used += blocks;
            if (sStats.vram_blocks_used > sStats.vram_blocks_peak)
                sStats.vram_blocks_peak = sStats.vram_blocks_used;
            return 1;
        }
        if (i < sRangeCount)
            prev_end = sRanges[i].end;
    }
    return 0;
}

static void vram_free(int16_t owner)
{
    int i;

    for (i = 0; i < sRangeCount; i++)
    {
        if (sRanges[i].owner == owner)
        {
            sStats.vram_blocks_used -= sRanges[i].end - sRanges[i].start;
            memmove(&sRanges[i], &sRanges[i + 1], sizeof(VramRange) * (size_t)(sRangeCount - i - 1));
            sRangeCount--;
            return;
        }
    }
}

/* ------------------------------------------------------------------ */
/* GS memory footprint                                                  */
/* ------------------------------------------------------------------ */

/* Block numbers within a page (GS manual), [row][col]. */
static const uint8_t sBlk32[4][8] = {
    { 0, 1, 4, 5, 16, 17, 20, 21 }, { 2, 3, 6, 7, 18, 19, 22, 23 },
    { 8, 9, 12, 13, 24, 25, 28, 29 }, { 10, 11, 14, 15, 26, 27, 30, 31 } };
static const uint8_t sBlk16[8][4] = {
    { 0, 2, 8, 10 }, { 1, 3, 9, 11 }, { 4, 6, 12, 14 }, { 5, 7, 13, 15 },
    { 16, 18, 24, 26 }, { 17, 19, 25, 27 }, { 20, 22, 28, 30 }, { 21, 23, 29, 31 } };

uint32_t ps2_vram_footprint_blocks(int psm, int w, int h, int tbw)
{
    int pw, ph, bw, bh, cols, rows;
    const uint8_t *tab;
    int tab_cols;
    int pages_x, pages_y, bx, by;
    uint32_t max_blk = 0;

    switch (psm)
    {
    case GSPSM_CT32: pw = 64; ph = 32; bw = 8; bh = 8; tab = &sBlk32[0][0]; tab_cols = 8; break;
    case GSPSM_CT16: pw = 64; ph = 64; bw = 16; bh = 8; tab = &sBlk16[0][0]; tab_cols = 4; break;
    case GSPSM_T8: pw = 128; ph = 64; bw = 16; bh = 16; tab = &sBlk32[0][0]; tab_cols = 8; break;
    default: /* T4 */ pw = 128; ph = 128; bw = 32; bh = 16; tab = &sBlk16[0][0]; tab_cols = 4; break;
    }
    pages_x = (tbw * 64) / pw;
    if (pages_x < 1)
        pages_x = 1;
    if (w >= pw || h >= ph)
    {
        pages_y = (h + ph - 1) / ph;
        return (uint32_t)(pages_x * pages_y * PS2_PAGE_BLOCKS);
    }
    cols = (w + bw - 1) / bw;
    rows = (h + bh - 1) / bh;
    for (by = 0; by < rows; by++)
        for (bx = 0; bx < cols; bx++)
            if (tab[by * tab_cols + bx] > max_blk)
                max_blk = tab[by * tab_cols + bx];
    return max_blk + 1;
}

/* ------------------------------------------------------------------ */
/* Upload                                                               */
/* ------------------------------------------------------------------ */

static void upload(uint32_t dbp, uint32_t dbw, uint32_t psm, int w, int h, const void *data, uint32_t bytes)
{
    uint32_t qwc = (bytes + 15) / 16;

    ps2_pkt_ad_begin(4);
    ps2_pkt_ad(GSR_BITBLTBUF, GSV_BITBLTBUF(0, 0, 0, dbp, dbw, psm));
    ps2_pkt_ad(GSR_TRXPOS, GSV_TRXPOS(0, 0, 0, 0, 0));
    ps2_pkt_ad(GSR_TRXREG, GSV_TRXREG(w, h));
    ps2_pkt_ad(GSR_TRXDIR, 0);
    ps2_pkt_reserve(1);
    gPS2Pkt.ptr[0] = GIFTAG_LO(qwc, 0, 0, 0, GIF_FLG_IMAGE, 0);
    gPS2Pkt.ptr[1] = 0;
    gPS2Pkt.ptr += 2;
    ps2_pkt_ref(data, qwc);
    gPS2RenderStats.tex_uploads++;
    gPS2RenderStats.tex_upload_bytes += bytes;
}

static void *staging_alloc(uint32_t bytes)
{
    void *p;

    bytes = (bytes + 63) & ~63u;
    if (sStagingPos + bytes > STAGING_BYTES)
    {
        /* Everything staged so far is referenced by queued DMA: drain the
         * GS, then the ring can be reused from the start. */
        ps2_pkt_finish();
        sStagingPos = 0;
    }
    p = sStaging + sStagingPos;
    sStagingPos += bytes;
    return p;
}

/* ------------------------------------------------------------------ */
/* Conversion                                                           */
/* ------------------------------------------------------------------ */

static inline uint32_t ct32(uint32_t r, uint32_t g, uint32_t b, uint32_t a255)
{
    return r | (g << 8) | (b << 16) | (((a255 + 1) >> 1) << 24);
}

static inline uint32_t rgba5551_to_ct32(uint16_t c)
{
    uint32_t r = (c >> 11) & 31, g = (c >> 6) & 31, b = (c >> 1) & 31;

    return ct32((r << 3) | (r >> 2), (g << 3) | (g >> 2), (b << 3) | (b >> 2), (c & 1) ? 255 : 0);
}

static inline uint32_t ia16_to_ct32(uint16_t c)
{
    uint32_t i = c >> 8, a = c & 0xFF;

    return ct32(i, i, i, a);
}

static void build_fixed_cluts(void)
{
    int i;

    for (i = 0; i < 16; i++)
    {
        uint32_t v = (uint32_t)i * 17;
        uint32_t iv = (uint32_t)((i >> 1) * 255 / 7);

        sClutI4[i] = ct32(v, v, v, v);
        sClutIA4[i] = ct32(iv, iv, iv, (i & 1) ? 255 : 0);
    }
    for (i = 0; i < 256; i++)
    {
        uint32_t iv = (uint32_t)(i >> 4) * 17, av = (uint32_t)(i & 15) * 17;
        int pos = (i & ~0x18) | ((i & 0x08) << 1) | ((i & 0x10) >> 1); /* CSM1 swizzle */

        sClutI8[pos] = ct32((uint32_t)i, (uint32_t)i, (uint32_t)i, (uint32_t)i);
        sClutIA8[pos] = ct32(iv, iv, iv, av);
    }
}

/* Source texel fetchers (N64 data is big-endian byte streams). */
static inline uint32_t fetch4(const uint8_t *row, int x)
{
    uint8_t b = row[x >> 1];

    return (x & 1) ? (b & 0xF) : (b >> 4);
}

static inline uint32_t fetch4_swz(const uint8_t *row, int x, uint32_t swz)
{
    uint8_t b = row[((uint32_t)x >> 1) ^ swz];

    return (x & 1) ? (b & 0xF) : (b >> 4);
}

static inline uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((p[0] << 8) | p[1]);
}

/* Wrap/mirror a coordinate into the source extent. */
static inline int src_coord(int x, int size, int mirror)
{
    if (mirror)
    {
        int period = size * 2;
        int m = x % period;

        return (m < size) ? m : (period - 1 - m);
    }
    return x % size;
}

static uint32_t hash_source(const PS2TexKey *k)
{
    const uint8_t *p = (const uint8_t *)k->addr;
    uint32_t h = 2166136261u;
    int i;

    for (i = 0; i < 32; i++)
        h = (h ^ p[i]) * 16777619u;
    if (k->tlut != NULL)
    {
        const uint8_t *t = (const uint8_t *)k->tlut;

        for (i = 0; i < 16; i++)
            h = (h ^ t[i]) * 16777619u;
    }
    return h;
}

static int log2_ceil(int v)
{
    int l = 0;

    while ((1 << l) < v)
        l++;
    return l;
}

/* Convert + stage + queue the upload for entry e. Returns 0 on failure. */
static int make_resident(TexEntry *e, int16_t idx)
{
    const PS2TexKey *k = &e->key;
    int sw = k->width, sh = k->height;          /* source mask period */
    int w = k->clamp_width ? k->clamp_width : (sw << (k->mirror_s ? 1 : 0));
    int h = k->clamp_height ? k->clamp_height : (sh << (k->mirror_t ? 1 : 0));
    int tw = log2_ceil(w), th = log2_ceil(h);
    int gw, gh, psm, tbw, x, y;
    uint32_t bytes, clut_bytes = 0, blocks, clut_blocks = 0, base, clut_base = 0;
    uint8_t *dst;
    const uint32_t *clut_src = NULL;
    uint32_t *clut_dst = NULL;
    int clut_entries = 0;
    int fixed_clut = -1;
    const uint8_t *src = (const uint8_t *)k->addr;
    int pitch = k->line_bytes;

    if (tw < 3) tw = 3; /* >= 8x8 keeps uploads qword-sized */
    if (th < 3) th = 3;
    gw = 1 << tw;
    gh = 1 << th;

    switch ((k->fmt << 4) | k->siz)
    {
    case (FMT_CI << 4) | SIZ_4: psm = GSPSM_T4; clut_entries = 16; break;
    case (FMT_CI << 4) | SIZ_8: psm = GSPSM_T8; clut_entries = 256; break;
    case (FMT_I << 4) | SIZ_4: psm = GSPSM_T4; fixed_clut = 0; break;
    case (FMT_IA << 4) | SIZ_4: psm = GSPSM_T4; fixed_clut = 1; break;
    case (FMT_I << 4) | SIZ_8: psm = GSPSM_T8; fixed_clut = 2; break;
    case (FMT_IA << 4) | SIZ_8: psm = GSPSM_T8; fixed_clut = 3; break;
    case (FMT_RGBA << 4) | SIZ_16: psm = GSPSM_CT16; break;
    case (FMT_RGBA << 4) | SIZ_32: psm = GSPSM_CT32; break;
    case (FMT_IA << 4) | SIZ_16: psm = GSPSM_CT32; break;
    case (FMT_I << 4) | SIZ_16: psm = GSPSM_CT32; break; /* rare: treat as IA16 */
    default:
        return 0;
    }

    tbw = (gw + 63) / 64;
    if ((psm == GSPSM_T4 || psm == GSPSM_T8) && (tbw & 1))
        tbw++;
    if (tbw < 1)
        tbw = 1;

    switch (psm)
    {
    case GSPSM_T4: bytes = (uint32_t)(gw * gh / 2); break;
    case GSPSM_T8: bytes = (uint32_t)(gw * gh); break;
    case GSPSM_CT16: bytes = (uint32_t)(gw * gh * 2); break;
    default: bytes = (uint32_t)(gw * gh * 4); break;
    }
    blocks = ps2_vram_footprint_blocks(psm, gw, gh, tbw);
    if (clut_entries)
    {
        clut_bytes = (uint32_t)clut_entries * 4;
        clut_blocks = (clut_entries == 16) ? 1 : 4;
    }

    /* VRAM: evict least-recently-used entries until it fits. */
    while (!vram_alloc(blocks + clut_blocks, idx, &base))
    {
        int16_t victim = -1;
        uint32_t oldest = 0xFFFFFFFFu;
        int i;

        for (i = 0; i < MAX_ENTRIES; i++)
        {
            TexEntry *o = &sEntries[i];

            if (o->in_use && o->resident && o != e && o->last_used < oldest)
            {
                oldest = o->last_used;
                victim = (int16_t)i;
            }
        }
        if (victim < 0)
        {
            ps2_log("tex: VRAM exhausted (%u blocks needed)", (unsigned)(blocks + clut_blocks));
            return 0;
        }
        vram_free(victim);
        sEntries[victim].resident = 0;
        sStats.evictions++;
    }
    clut_base = base + blocks;

    /* Convert. */
    dst = staging_alloc(bytes);
    memset(dst, 0, bytes);
    for (y = 0; y < h; y++)
    {
        int sy = src_coord(y, sh, k->mirror_t);
        const uint8_t *row = src + sy * pitch;
        /* Rows loaded by LoadBlock without dxt are stored the way TMEM
         * holds them: odd rows have the two 32-bit halves of every 64-bit
         * word swapped (64-bit halves for 32-bit texels). */
        uint32_t swz = (k->odd_swap && (sy & 1)) ? ((k->siz == SIZ_32) ? 8u : 4u) : 0u;

        for (x = 0; x < w; x++)
        {
            int sx = src_coord(x, sw, k->mirror_s);
            int di = y * gw + x;

            switch (psm)
            {
            case GSPSM_T4:
                dst[di >> 1] |= (uint8_t)(fetch4_swz(row, sx, swz) << ((di & 1) ? 4 : 0));
                break;
            case GSPSM_T8:
                dst[di] = row[(uint32_t)sx ^ swz];
                break;
            case GSPSM_CT16:
            {
                uint16_t c = be16(row + (((uint32_t)sx * 2) ^ swz));
                uint16_t g = (uint16_t)(((c >> 11) & 31) | (((c >> 6) & 31) << 5) | (((c >> 1) & 31) << 10) |
                                        ((c & 1) << 15));

                ((uint16_t *)dst)[di] = g;
                break;
            }
            default:
                if (k->siz == SIZ_32)
                {
                    const uint8_t *p = row + (((uint32_t)sx * 4) ^ swz);

                    ((uint32_t *)dst)[di] = ct32(p[0], p[1], p[2], p[3]);
                }
                else
                {
                    ((uint32_t *)dst)[di] = ia16_to_ct32(be16(row + (((uint32_t)sx * 2) ^ swz)));
                }
                break;
            }
        }
    }
    upload(base, (uint32_t)tbw, (uint32_t)psm, gw, gh, dst, bytes);

    if (clut_entries)
    {
        /* Palettes are typed u16 arrays in the decompiled data, i.e. native
         * endianness on the PS2. */
        const uint16_t *pal = (const uint16_t *)k->tlut;
        int i;

        clut_dst = staging_alloc(clut_bytes);
        for (i = 0; i < clut_entries; i++)
        {
            uint32_t c = (k->tlut_type == 3) ? ia16_to_ct32(pal[i]) : rgba5551_to_ct32(pal[i]);
            int pos = (clut_entries == 256) ? ((i & ~0x18) | ((i & 0x08) << 1) | ((i & 0x10) >> 1)) : i;

            clut_dst[pos] = c;
        }
        if (clut_entries == 16)
            upload(clut_base, 1, GSPSM_CT32, 8, 2, clut_dst, clut_bytes);
        else
            upload(clut_base, 1, GSPSM_CT32, 16, 16, clut_dst, clut_bytes);
    }
    (void)clut_src;

    ps2_pkt_ad_begin(1);
    ps2_pkt_ad(GSR_TEXFLUSH, 0);

    e->vram_block = base;
    e->vram_blocks = blocks + clut_blocks;
    e->clut_block = clut_entries ? clut_base : (fixed_clut >= 0 ? sFixedClutBlock[fixed_clut] : 0);
    e->gs_w = (uint16_t)gw;
    e->gs_h = (uint16_t)gh;
    e->tex0 = GSV_TEX0(base, (uint64_t)tbw, (uint64_t)psm, (uint64_t)tw, (uint64_t)th, 0, 0,
                      (psm == GSPSM_T4 || psm == GSPSM_T8) ? e->clut_block : 0, GSPSM_CT32, 0, 0,
                      (psm == GSPSM_T4 || psm == GSPSM_T8) ? 1 : 0);
    e->resident = 1;
    gPS2RenderStats.tex_converts++;
    if (gPS2TexTrace > 0)
    {
        gPS2TexTrace--;
        gPS2TexJustConverted = 1;
        ps2_log("tex %p fmt%d siz%d %dx%d line%d mir%d%d swz%d tlut %p/%d", k->addr, k->fmt, k->siz, k->width,
                k->height, k->line_bytes, k->mirror_s, k->mirror_t, k->odd_swap, k->tlut, k->tlut_type);
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* Lookup                                                               */
/* ------------------------------------------------------------------ */

static uint32_t key_hash(const PS2TexKey *k)
{
    uint32_t h = (uint32_t)(uintptr_t)k->addr * 2654435761u;

    h ^= (uint32_t)(uintptr_t)k->tlut * 40503u;
    h ^= ((uint32_t)k->width << 16) ^ k->height ^ ((uint32_t)k->fmt << 5) ^ ((uint32_t)k->siz << 9) ^
         ((uint32_t)k->mirror_s << 12) ^ ((uint32_t)k->mirror_t << 13) ^ ((uint32_t)k->line_bytes << 3);
    h ^= ((uint32_t)k->clamp_width << 16) ^ k->clamp_height;
    return h;
}

static int key_eq(const PS2TexKey *a, const PS2TexKey *b)
{
    return a->addr == b->addr && a->tlut == b->tlut && a->width == b->width && a->height == b->height &&
           a->clamp_width == b->clamp_width && a->clamp_height == b->clamp_height &&
           a->fmt == b->fmt && a->siz == b->siz && a->line_bytes == b->line_bytes &&
           a->tlut_type == b->tlut_type && a->mirror_s == b->mirror_s && a->mirror_t == b->mirror_t &&
           a->odd_swap == b->odd_swap;
}

static void entry_release(int16_t i)
{
    TexEntry *e = &sEntries[i];

    if (e->resident)
    {
        vram_free(i);
    }
    e->in_use = 0;
    e->resident = 0;
    e->next_free = (int16_t)sFreeHead;
    sFreeHead = i;
    sStats.entries--;
}

void ps2_texcache_invalidate_all(void)
{
    int i;

    for (i = 0; i < HASH_SLOTS; i++)
        sHash[i] = -1;
    sFreeHead = -1;
    for (i = MAX_ENTRIES - 1; i >= 0; i--)
    {
        sEntries[i].in_use = 0;
        sEntries[i].resident = 0;
        sEntries[i].next_free = (int16_t)sFreeHead;
        sFreeHead = i;
    }
    sRangeCount = 0;
    sStats.entries = 0;
    sStats.vram_blocks_used = sPoolFirst - PS2_TEX_POOL_FIRST_BLOCK; /* fixed CLUTs */
    /* fixed CLUTs occupy the first pool blocks (see init) */
    sRanges[0].start = PS2_TEX_POOL_FIRST_BLOCK;
    sRanges[0].end = sPoolFirst;
    sRanges[0].owner = -2;
    sRangeCount = 1;
}

int ps2_texcache_bind(const PS2TexKey *key, PS2TexBinding *out)
{
    uint32_t h = key_hash(key) & (HASH_SLOTS - 1);
    uint32_t probe;
    int16_t idx = -1;
    TexEntry *e;
    uint32_t content;

    if (key->addr == NULL || key->width == 0 || key->height == 0)
        return 0;

    content = hash_source(key);

    for (probe = 0; probe < HASH_SLOTS; probe++)
    {
        int16_t i = sHash[(h + probe) & (HASH_SLOTS - 1)];

        if (i < 0)
            break;
        if (sEntries[i].in_use && key_eq(&sEntries[i].key, key))
        {
            idx = i;
            break;
        }
    }

    if (idx >= 0 && sEntries[idx].hash != content)
    {
        /* Same address, different data (memory reused): reconvert. */
        if (sEntries[idx].resident)
        {
            vram_free(idx);
            sEntries[idx].resident = 0;
        }
        sEntries[idx].hash = content;
    }

    if (idx < 0)
    {
        if (sFreeHead < 0 || sStats.entries >= MAX_ENTRIES - 1)
        {
            /* Table full: start over (rare - scene with >1500 textures). */
            ps2_texcache_invalidate_all();
            h = key_hash(key) & (HASH_SLOTS - 1);
        }
        idx = (int16_t)sFreeHead;
        sFreeHead = sEntries[idx].next_free;
        e = &sEntries[idx];
        memset(e, 0, sizeof(*e));
        e->key = *key;
        e->hash = content;
        e->in_use = 1;
        sStats.entries++;
        for (probe = 0; probe < HASH_SLOTS; probe++)
        {
            uint32_t slot = (h + probe) & (HASH_SLOTS - 1);

            if (sHash[slot] < 0 || !sEntries[sHash[slot]].in_use)
            {
                sHash[slot] = idx;
                break;
            }
        }
    }

    e = &sEntries[idx];
    e->last_used = sFrame;
    if (!e->resident && !make_resident(e, idx))
    {
        (void)entry_release;
        return 0;
    }
    out->tex0 = e->tex0;
    out->gs_w = e->gs_w;
    out->gs_h = e->gs_h;
    return 1;
}

void ps2_texcache_frame_begin(void)
{
    sFrame++;
    sStagingPos = 0; /* previous frame's DMA has completed */
    if (gPS2TexFlush)
    {
        /* debug: reconvert everything (with gPS2TexTrace to log it) */
        gPS2TexFlush = 0;
        ps2_texcache_invalidate_all();
    }
}

const PS2TexStats *ps2_texcache_stats(void)
{
    return &sStats;
}

void ps2_texcache_init(void)
{
    uint32_t b = PS2_TEX_POOL_FIRST_BLOCK;

    sStaging = ps2_mem_alloc(PS2_MEM_TEXTURE_CACHE, STAGING_BYTES, 64);
    sStats.ee_capacity = STAGING_BYTES;
    build_fixed_cluts();

    /* Fixed CLUTs: I4, IA4 (1 block each), I8, IA8 (4 blocks each). */
    sFixedClutBlock[0] = b; b += 1;
    sFixedClutBlock[1] = b; b += 1;
    sFixedClutBlock[2] = b; b += 4;
    sFixedClutBlock[3] = b; b += 4;
    sPoolFirst = (b + PS2_PAGE_BLOCKS - 1) & ~(uint32_t)(PS2_PAGE_BLOCKS - 1);

    upload(sFixedClutBlock[0], 1, GSPSM_CT32, 8, 2, sClutI4, sizeof(sClutI4));
    upload(sFixedClutBlock[1], 1, GSPSM_CT32, 8, 2, sClutIA4, sizeof(sClutIA4));
    upload(sFixedClutBlock[2], 1, GSPSM_CT32, 16, 16, sClutI8, sizeof(sClutI8));
    upload(sFixedClutBlock[3], 1, GSPSM_CT32, 16, 16, sClutIA8, sizeof(sClutIA8));
    ps2_pkt_finish();

    ps2_texcache_invalidate_all();
    ps2_log("tex: pool %u KiB, staging %u KiB", (unsigned)((PS2_VRAM_BLOCKS - sPoolFirst) / 4),
            (unsigned)(STAGING_BYTES >> 10));
}

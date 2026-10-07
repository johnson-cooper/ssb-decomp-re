/*
 * F3DEX2 + RDP display-list translator (GBI -> GS).
 *
 * This is the only place that understands N64 display lists. It keeps the
 * RSP/RDP *state* (matrices, vertices, lights, tiles, TMEM loads, other
 * modes, combiner, colors) and turns each primitive into GS primitives with
 * the closest equivalent GS state:
 *
 *   vertex transform / lighting / fog / texgen / clipping : EE (VU1 later)
 *   color combiner   -> per-vertex RGBA + GS MODULATE (see eval_combiner)
 *   blender          -> GS ALPHA / TEST / FOG / ZBUF
 *   TMEM loads       -> texture cache keys (texcache.c converts + uploads)
 *
 * Triangles sharing GS state are batched under one GIFtag, so a typical
 * frame is a few hundred batches rather than thousands of primitives.
 *
 * Unsupported / approximated features are listed in PS2_PORT.md.
 */
#include "render.h"

#include <ps2/platform.h>

#define F3DEX_GBI_2 1
#include <PR/ultratypes.h>
#include <PR/gbi.h>

#include <math.h>
#include <string.h>

/* ------------------------------------------------------------------ */
/* RSP / RDP state                                                      */
/* ------------------------------------------------------------------ */

#define MAX_VTX 64
#define MTX_STACK 16
#define MAX_LIGHTS 8
#define MAX_TMEM_LOADS 16

typedef struct GbiVtx
{
    float x, y, z, w;   /* clip space */
    float s, t;         /* texel coords (N64 s10.5 already scaled by gSPTexture) */
    uint8_t r, g, b, a; /* shade (color or lit), alpha may be fog */
    uint8_t clip;       /* outcodes */
    uint8_t fog;        /* 0..255 fog factor (N64 shade alpha when G_FOG) */
} GbiVtx;

typedef struct GbiTile
{
    uint8_t fmt, siz, palette;
    uint8_t cms, cmt, masks, maskt, shifts, shiftt;
    uint16_t line, tmem;
    uint16_t uls, ult, lrs, lrt; /* 10.2 */
} GbiTile;

typedef struct TmemLoad
{
    uint16_t tmem;      /* start, 64-bit words */
    uint16_t words;
    const uint8_t *src; /* DRAM of the loaded texel (0,0) */
    uint32_t pitch;     /* DRAM bytes per row as loaded */
    uint8_t siz;
    uint8_t is_tlut;
    uint8_t odd_swap; /* LoadBlock with dxt 0: RAM holds TMEM's odd-row word swap */
    const void *cmd;  /* debug: the load command */
} TmemLoad;

static struct
{
    uint32_t segments[16];
    uint32_t seg_set_mask;

    float mv[MTX_STACK][4][4];
    int mv_top;
    float proj[4][4];
    float mvp[4][4];
    int mvp_dirty;

    GbiVtx vtx[MAX_VTX];

    uint32_t geom;
    uint32_t om_h, om_l;
    uint32_t cc_w0, cc_w1;
    uint8_t prim[4], env[4], fog[4], blend[4];
    uint8_t prim_lod_frac;
    uint32_t fill_color;
    uint16_t prim_z;

    float vp_scale[3], vp_trans[3];

    int num_lights;
    float light_col[MAX_LIGHTS + 1][3]; /* last = ambient */
    float light_dir[MAX_LIGHTS][3];     /* world-space normalized */
    float lookat[2][3];
    float light_dir_model[MAX_LIGHTS][3];
    float lookat_model[2][3];
    int lights_dirty;

    int fog_mul, fog_off;

    /* gSPTexture */
    int tex_on, tex_tile;
    float tex_scale_s, tex_scale_t;

    /* SetTextureImage */
    uint8_t timg_fmt, timg_siz;
    uint16_t timg_width;
    const uint8_t *timg_addr;

    GbiTile tiles[8];
    TmemLoad loads[MAX_TMEM_LOADS];
    int load_count;

    int color_target;  /* FB index, or -1 */
    int zimg_is_z;
    int cimg_is_z;
    int cimg_offscreen; /* unsupported non-display color image: suppress draws */

    uint32_t rdphalf1, rdphalf2;
    int rect_tile;
    int scissor[4];
} R;

static int sLastColorTarget = -1;
int gPS2GbiTrace; /* debug: log this many upcoming commands */
int gPS2CombTrace; /* debug: log distinct textured combiner setups */
int gPS2TlutTrace; /* debug: log this many CI texture binds with their TLUT load */
static const void *sCurCmd; /* debug: command being executed */

/* ------------------------------------------------------------------ */
/* GS state tracking and batching                                       */
/* ------------------------------------------------------------------ */

typedef struct GsState
{
    uint64_t tex0, clamp, tex1, alpha, test, zbuf, fogcol, scissor;
    uint64_t prim;     /* PRIM register bits used in the GIFtag */
    int nreg;          /* 2 = RGBAQ,XYZ ; 3 = ST,RGBAQ,XYZ */
    int use_fog;
} GsState;

static GsState sCur;       /* what the GS has */
static int sCurValid;
static uint64_t *sBatchTag; /* open GIFtag, or NULL */
static uint32_t sBatchVerts;
static GsState sBatchState;

static void batch_close(void)
{
    if (sBatchTag != NULL)
    {
        sBatchTag[0] |= (uint64_t)sBatchVerts; /* NLOOP */
        sBatchTag = NULL;
        sBatchVerts = 0;
    }
}

static void gs_apply_state(const GsState *s)
{
    int n = 0;
    uint64_t *save;

    batch_close();
    /* Count changes first so they fit one A+D tag. */
    if (!sCurValid || s->tex0 != sCur.tex0) n++;
    if (!sCurValid || s->clamp != sCur.clamp) n++;
    if (!sCurValid || s->tex1 != sCur.tex1) n++;
    if (!sCurValid || s->alpha != sCur.alpha) n++;
    if (!sCurValid || s->test != sCur.test) n++;
    if (!sCurValid || s->zbuf != sCur.zbuf) n++;
    if (!sCurValid || s->fogcol != sCur.fogcol) n++;
    if (!sCurValid || s->scissor != sCur.scissor) n++;
    if (n == 0)
    {
        return;
    }
    ps2_pkt_ad_begin((uint32_t)n);
    save = gPS2Pkt.ptr;
    (void)save;
    if (!sCurValid || s->tex0 != sCur.tex0) ps2_pkt_ad(GSR_TEX0_1, s->tex0);
    if (!sCurValid || s->clamp != sCur.clamp) ps2_pkt_ad(GSR_CLAMP_1, s->clamp);
    if (!sCurValid || s->tex1 != sCur.tex1) ps2_pkt_ad(GSR_TEX1_1, s->tex1);
    if (!sCurValid || s->alpha != sCur.alpha) ps2_pkt_ad(GSR_ALPHA_1, s->alpha);
    if (!sCurValid || s->test != sCur.test) ps2_pkt_ad(GSR_TEST_1, s->test);
    if (!sCurValid || s->zbuf != sCur.zbuf) ps2_pkt_ad(GSR_ZBUF_1, s->zbuf);
    if (!sCurValid || s->fogcol != sCur.fogcol) ps2_pkt_ad(GSR_FOGCOL, s->fogcol);
    if (!sCurValid || s->scissor != sCur.scissor) ps2_pkt_ad(GSR_SCISSOR_1, s->scissor);
    gPS2RenderStats.state_writes += (uint32_t)n;
    sCur = *s;
    sCurValid = 1;
}

/* Reserve room for `verts` vertices in a batch compatible with `s`. */
static uint64_t *batch_reserve(const GsState *s, uint32_t verts)
{
    uint32_t qw = verts * (uint32_t)s->nreg;
    uint64_t *p;

    if (sBatchTag != NULL && (sBatchState.prim != s->prim || sBatchState.nreg != s->nreg ||
                              gPS2Pkt.ptr + qw * 2 >= gPS2Pkt.end || sBatchVerts + verts > 30000 ||
                              memcmp(&sBatchState, s, sizeof(*s)) != 0))
    {
        batch_close();
    }
    if (sBatchTag == NULL)
    {
        if (!sCurValid || memcmp(&sCur, s, sizeof(*s)) != 0)
        {
            gs_apply_state(s);
        }
        ps2_pkt_reserve(1 + qw + 64);
        p = gPS2Pkt.ptr;
        p[0] = GIFTAG_LO(0, 0, 1, s->prim, GIF_FLG_PACKED, s->nreg);
        p[1] = (s->nreg == 3) ? ((uint64_t)GSR_ST | ((uint64_t)GSR_RGBAQ << 4) |
                                 ((uint64_t)(s->use_fog ? GSR_XYZF2 : GSR_XYZ2) << 8))
                              : ((uint64_t)GSR_RGBAQ | ((uint64_t)(s->use_fog ? GSR_XYZF2 : GSR_XYZ2) << 4));
        sBatchTag = p;
        sBatchState = *s;
        gPS2Pkt.ptr = p + 2;
        gPS2RenderStats.batches++;
    }
    else
    {
        ps2_pkt_reserve(qw);
    }
    sBatchVerts += verts;
    return gPS2Pkt.ptr;
}

/* ------------------------------------------------------------------ */
/* Helpers                                                              */
/* ------------------------------------------------------------------ */

static const void *seg_addr(uint32_t a)
{
    if (a & 0x80000000u)
    {
        /* KSEG0/KSEG1 N64 pointer: physical part (never produced by PS2
         * code, but literal hardware addresses may appear in data). */
        return (const void *)(uintptr_t)(a & 0x1FFFFFFFu);
    }
    return (const void *)(uintptr_t)(R.segments[(a >> 24) & 0xF] + (a & 0x00FFFFFFu));
}

static void mtx_from_fixed(float out[4][4], const void *addr)
{
    const int32_t *m = (const int32_t *)addr;
    int i, j;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j += 2)
        {
            int32_t ip = m[i * 2 + j / 2];
            uint32_t fp = (uint32_t)m[8 + i * 2 + j / 2];

            out[i][j] = (float)(int32_t)((ip & 0xFFFF0000) | (fp >> 16)) * (1.0f / 65536.0f);
            out[i][j + 1] = (float)(int32_t)(((uint32_t)ip << 16) | (fp & 0xFFFF)) * (1.0f / 65536.0f);
        }
    }
}

static void mtx_mul(float r[4][4], const float a[4][4], const float b[4][4])
{
    float t[4][4];
    int i, j;

    for (i = 0; i < 4; i++)
    {
        for (j = 0; j < 4; j++)
        {
            t[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j] + a[i][3] * b[3][j];
        }
    }
    memcpy(r, t, sizeof(t));
}

static inline uint8_t clamp_u8(float v)
{
    return (v <= 0.0f) ? 0 : (v >= 255.0f) ? 255 : (uint8_t)v;
}

static void normalize3(float v[3])
{
    float l = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    if (l > 0.0f)
    {
        l = 1.0f / l;
        v[0] *= l;
        v[1] *= l;
        v[2] *= l;
    }
}

/* Light directions -> model space (transpose of the modelview's 3x3). */
static void update_model_lights(void)
{
    float (*m)[4] = R.mv[R.mv_top];
    int i;

    for (i = 0; i < R.num_lights; i++)
    {
        float *d = R.light_dir[i];
        float *o = R.light_dir_model[i];

        o[0] = d[0] * m[0][0] + d[1] * m[0][1] + d[2] * m[0][2];
        o[1] = d[0] * m[1][0] + d[1] * m[1][1] + d[2] * m[1][2];
        o[2] = d[0] * m[2][0] + d[1] * m[2][1] + d[2] * m[2][2];
        normalize3(o);
    }
    for (i = 0; i < 2; i++)
    {
        float *d = R.lookat[i];
        float *o = R.lookat_model[i];

        o[0] = d[0] * m[0][0] + d[1] * m[0][1] + d[2] * m[0][2];
        o[1] = d[0] * m[1][0] + d[1] * m[1][1] + d[2] * m[1][2];
        o[2] = d[0] * m[2][0] + d[1] * m[2][1] + d[2] * m[2][2];
        normalize3(o);
    }
    R.lights_dirty = 0;
}

/* ------------------------------------------------------------------ */
/* Vertices                                                             */
/* ------------------------------------------------------------------ */

#define CLIP_NEAR 0x01
#define CLIP_GUARD 0x02
#define GUARD 6.0f /* keep screen coords inside the GS 4096 range */

static void load_vertices(const Vtx *src, int n, int dst)
{
    int i;

    if (R.mvp_dirty)
    {
        mtx_mul(R.mvp, R.mv[R.mv_top], R.proj);
        R.mvp_dirty = 0;
    }
    if ((R.geom & G_LIGHTING) && R.lights_dirty)
    {
        update_model_lights();
    }

    for (i = 0; i < n && dst + i < MAX_VTX; i++)
    {
        const Vtx_t *v = &src[i].v;
        GbiVtx *o = &R.vtx[dst + i];
        float x = v->ob[0], y = v->ob[1], z = v->ob[2];
        float (*m)[4] = R.mvp;
        uint8_t clip = 0;

        o->x = x * m[0][0] + y * m[1][0] + z * m[2][0] + m[3][0];
        o->y = x * m[0][1] + y * m[1][1] + z * m[2][1] + m[3][1];
        o->z = x * m[0][2] + y * m[1][2] + z * m[2][2] + m[3][2];
        o->w = x * m[0][3] + y * m[1][3] + z * m[2][3] + m[3][3];

        if (o->z < -o->w || o->w < 0.001f)
            clip |= CLIP_NEAR;
        if (o->x < -GUARD * o->w || o->x > GUARD * o->w || o->y < -GUARD * o->w || o->y > GUARD * o->w)
            clip |= CLIP_GUARD;
        o->clip = clip;

        if (R.geom & G_LIGHTING)
        {
            const Vtx_tn *vn = &src[i].n;
            float nx = vn->n[0], ny = vn->n[1], nz = vn->n[2];
            float cr = R.light_col[R.num_lights][0];
            float cg = R.light_col[R.num_lights][1];
            float cb = R.light_col[R.num_lights][2];
            int l;

            for (l = 0; l < R.num_lights; l++)
            {
                float d = (nx * R.light_dir_model[l][0] + ny * R.light_dir_model[l][1] +
                           nz * R.light_dir_model[l][2]) * (1.0f / 127.0f);

                if (d > 0.0f)
                {
                    cr += d * R.light_col[l][0];
                    cg += d * R.light_col[l][1];
                    cb += d * R.light_col[l][2];
                }
            }
            o->r = clamp_u8(cr);
            o->g = clamp_u8(cg);
            o->b = clamp_u8(cb);
            o->a = vn->a;

            if (R.geom & G_TEXTURE_GEN)
            {
                float dx = (nx * R.lookat_model[0][0] + ny * R.lookat_model[0][1] + nz * R.lookat_model[0][2]);
                float dy = (nx * R.lookat_model[1][0] + ny * R.lookat_model[1][1] + nz * R.lookat_model[1][2]);

                /* Maps [-127,127] onto [0, texture size] like the ucode. */
                o->s = (dx / 127.0f + 1.0f) * 0.25f * R.tex_scale_s * 65536.0f / 32.0f;
                o->t = (dy / 127.0f + 1.0f) * 0.25f * R.tex_scale_t * 65536.0f / 32.0f;
            }
            else
            {
                o->s = (float)v->tc[0] * R.tex_scale_s / 32.0f;
                o->t = (float)v->tc[1] * R.tex_scale_t / 32.0f;
            }
        }
        else
        {
            o->r = v->cn[0];
            o->g = v->cn[1];
            o->b = v->cn[2];
            o->a = v->cn[3];
            o->s = (float)v->tc[0] * R.tex_scale_s / 32.0f;
            o->t = (float)v->tc[1] * R.tex_scale_t / 32.0f;
        }

        if (R.geom & G_FOG)
        {
            float wi = (o->w > 0.0001f) ? 1.0f / o->w : 10000.0f;
            float f = o->z * wi * (float)R.fog_mul + (float)R.fog_off;

            o->fog = clamp_u8(f);
        }
        else
        {
            o->fog = 0;
        }
    }
}

/* ------------------------------------------------------------------ */
/* Color combiner -> per-vertex color                                   */
/* ------------------------------------------------------------------ */

/* A combiner input evaluated at one vertex as k*T + c (T = texel). */
typedef struct Lin
{
    float k, c;
} Lin;

typedef struct CombineOut
{
    Lin rgb[3];
    Lin a;
} CombineOut;

enum
{
    IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ONE, IN_ZERO,
    IN_COMBINED_A, IN_TEXEL0_A, IN_TEXEL1_A, IN_PRIM_A, IN_SHADE_A, IN_ENV_A,
    IN_LODFRAC, IN_PRIM_LODFRAC, IN_NOISE
};

static int map_rgb_a(int v)
{
    static const int t[16] = { IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ONE, IN_NOISE,
                               IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO };
    return t[v & 15];
}

static int map_rgb_b(int v)
{
    static const int t[16] = { IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ZERO, IN_ZERO,
                               IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO, IN_ZERO };
    return t[v & 15];
}

static int map_rgb_c(int v)
{
    static const int t[32] = { IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ONE,
                               IN_COMBINED_A, IN_TEXEL0_A, IN_TEXEL1_A, IN_PRIM_A, IN_SHADE_A, IN_ENV_A,
                               IN_LODFRAC, IN_PRIM_LODFRAC, IN_ZERO };
    return ((v & 31) < 16) ? t[v & 31] : IN_ZERO;
}

static int map_rgb_d(int v)
{
    static const int t[8] = { IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ONE, IN_ZERO };
    return t[v & 7];
}

static int map_a_abd(int v)
{
    static const int t[8] = { IN_COMBINED, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_ONE, IN_ZERO };
    return t[v & 7];
}

static int map_a_c(int v)
{
    static const int t[8] = { IN_LODFRAC, IN_TEXEL0, IN_TEXEL1, IN_PRIM, IN_SHADE, IN_ENV, IN_PRIM_LODFRAC,
                              IN_ZERO };
    return t[v & 7];
}

/* Value that texel-alpha color inputs (TEXEL0_ALPHA as the C of an RGB
 * cycle) evaluate to. The GS cannot scale texel color by texel alpha inside
 * one primitive, so lerps of the form (TEXEL0 - X) * TEXEL0_ALPHA + X (decal
 * textures over shade, e.g. fighter faces) are drawn in two passes: X with
 * this set to 0, then the texel part with 1, blended by texel alpha. */
static float sTexAlphaIn = 1.0f;

/* Input value for channel ch (0..2 = rgb, 3 = alpha). */
static Lin input(int in, int ch, const GbiVtx *v, const CombineOut *prev)
{
    Lin r = { 0.0f, 0.0f };
    const float s = 1.0f / 255.0f;

    switch (in)
    {
    case IN_COMBINED: r = (ch < 3) ? prev->rgb[ch] : prev->a; break;
    case IN_TEXEL0:
    case IN_TEXEL1: r.k = 1.0f; break;
    case IN_PRIM: r.c = ((ch < 3) ? R.prim[ch] : R.prim[3]) * s; break;
    case IN_SHADE:
        r.c = ((ch == 0) ? v->r : (ch == 1) ? v->g : (ch == 2) ? v->b : ((R.geom & G_FOG) ? 255 : v->a)) * s;
        break;
    case IN_ENV: r.c = ((ch < 3) ? R.env[ch] : R.env[3]) * s; break;
    case IN_ONE: r.c = 1.0f; break;
    case IN_COMBINED_A: r = prev->a; break;
    case IN_TEXEL0_A:
    case IN_TEXEL1_A:
        /* texel alpha inside a color channel: see sTexAlphaIn */
        if (ch == 3) r.k = 1.0f; else r.c = sTexAlphaIn;
        break;
    case IN_PRIM_A: r.c = R.prim[3] * s; break;
    case IN_SHADE_A: r.c = ((R.geom & G_FOG) ? 255 : v->a) * s; break;
    case IN_ENV_A: r.c = R.env[3] * s; break;
    case IN_PRIM_LODFRAC: r.c = R.prim_lod_frac * s; break;
    case IN_LODFRAC: r.c = 0.0f; break;
    case IN_NOISE: r.c = 0.5f; break;
    default: break;
    }
    return r;
}

/* (a - b) * c + d with T^2 ~= T. */
static Lin combine(Lin a, Lin b, Lin c, Lin d)
{
    Lin ab = { a.k - b.k, a.c - b.c };
    Lin r;

    r.k = ab.k * c.c + ab.c * c.k + ab.k * c.k + d.k;
    r.c = ab.c * c.c + d.c;
    return r;
}

static void eval_combiner(const GbiVtx *v, CombineOut *out)
{
    uint32_t w0 = R.cc_w0, w1 = R.cc_w1;
    int cycles = ((R.om_h & (3u << 20)) == G_CYC_2CYCLE) ? 2 : 1;
    int cyc, ch;
    CombineOut prev, cur;

    memset(&prev, 0, sizeof(prev));
    for (cyc = 0; cyc < cycles; cyc++)
    {
        int a, b, c, d, aa, ab, ac, ad;

        if (cyc == 0)
        {
            a = (w0 >> 20) & 0xF; c = (w0 >> 15) & 0x1F; aa = (w0 >> 12) & 0x7; ac = (w0 >> 9) & 0x7;
            b = (w1 >> 28) & 0xF; d = (w1 >> 15) & 0x7; ab = (w1 >> 12) & 0x7; ad = (w1 >> 9) & 0x7;
        }
        else
        {
            a = (w0 >> 5) & 0xF; c = (w0 >> 0) & 0x1F; aa = (w1 >> 21) & 0x7; ac = (w1 >> 18) & 0x7;
            b = (w1 >> 24) & 0xF; d = (w1 >> 6) & 0x7; ab = (w1 >> 3) & 0x7; ad = (w1 >> 0) & 0x7;
        }
        for (ch = 0; ch < 3; ch++)
        {
            cur.rgb[ch] = combine(input(map_rgb_a(a), ch, v, &prev), input(map_rgb_b(b), ch, v, &prev),
                                  input(map_rgb_c(c), ch, v, &prev), input(map_rgb_d(d), ch, v, &prev));
        }
        cur.a = combine(input(map_a_abd(aa), 3, v, &prev), input(map_a_abd(ab), 3, v, &prev),
                        input(map_a_c(ac), 3, v, &prev), input(map_a_abd(ad), 3, v, &prev));
        prev = cur;
    }
    *out = prev;
}

/* Does an RGB cycle use texel alpha as its interpolation factor? */
static int combiner_color_lerps_by_texel_alpha(void)
{
    int cycles = ((R.om_h & (3u << 20)) == G_CYC_2CYCLE) ? 2 : 1;
    int c0 = (int)((R.cc_w0 >> 15) & 0x1F), c1 = (int)(R.cc_w0 & 0x1F);

    return (c0 == G_CCMUX_TEXEL0_ALPHA || c0 == G_CCMUX_TEXEL1_ALPHA) ||
           (cycles == 2 && (c1 == G_CCMUX_TEXEL0_ALPHA || c1 == G_CCMUX_TEXEL1_ALPHA));
}

/* Does the current combiner reference a texel input at all? */
static int combiner_uses_texture(void)
{
    uint32_t w0 = R.cc_w0, w1 = R.cc_w1;
    int cycles = ((R.om_h & (3u << 20)) == G_CYC_2CYCLE) ? 2 : 1;
    int f[2][8];
    int i, c;

    f[0][0] = (w0 >> 20) & 0xF; f[0][1] = (w1 >> 28) & 0xF; f[0][2] = (w0 >> 15) & 0x1F; f[0][3] = (w1 >> 15) & 0x7;
    f[0][4] = (w0 >> 12) & 0x7; f[0][5] = (w1 >> 12) & 0x7; f[0][6] = (w0 >> 9) & 0x7; f[0][7] = (w1 >> 9) & 0x7;
    f[1][0] = (w0 >> 5) & 0xF; f[1][1] = (w1 >> 24) & 0xF; f[1][2] = (w0 >> 0) & 0x1F; f[1][3] = (w1 >> 6) & 0x7;
    f[1][4] = (w1 >> 21) & 0x7; f[1][5] = (w1 >> 3) & 0x7; f[1][6] = (w1 >> 18) & 0x7; f[1][7] = (w1 >> 0) & 0x7;

    for (c = 0; c < cycles; c++)
    {
        for (i = 0; i < 8; i++)
        {
            int v = f[c][i];

            if (v == 1 || v == 2)
                return 1;
            if (i == 2 && (v == 8 || v == 9))
                return 1;
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* Texture binding                                                      */
/* ------------------------------------------------------------------ */

static int find_load(uint16_t tmem, int tlut)
{
    int i;

    for (i = R.load_count - 1; i >= 0; i--)
    {
        const TmemLoad *l = &R.loads[i];

        if (l->is_tlut == tlut && tmem >= l->tmem && tmem < l->tmem + (l->words ? l->words : 1))
        {
            return i;
        }
    }
    return -1;
}

static void record_load(uint16_t tmem, uint16_t words, const uint8_t *src, uint32_t pitch, uint8_t siz,
                        uint8_t is_tlut, uint8_t odd_swap)
{
    int i;
    TmemLoad *l;

    /* Drop loads fully overwritten by this one. */
    for (i = 0; i < R.load_count;)
    {
        const TmemLoad *o = &R.loads[i];

        if (o->is_tlut == is_tlut && o->tmem >= tmem && o->tmem + o->words <= tmem + words)
        {
            R.loads[i] = R.loads[--R.load_count];
        }
        else
        {
            i++;
        }
    }
    if (R.load_count == MAX_TMEM_LOADS)
    {
        memmove(&R.loads[0], &R.loads[1], sizeof(TmemLoad) * (MAX_TMEM_LOADS - 1));
        R.load_count--;
    }
    l = &R.loads[R.load_count++];
    l->tmem = tmem;
    l->words = words;
    l->src = src;
    l->pitch = pitch;
    l->siz = siz;
    l->is_tlut = is_tlut;
    l->odd_swap = odd_swap;
    l->cmd = sCurCmd;
}

typedef struct TexInfo
{
    int valid;
    PS2TexBinding bind;
    float off_s, off_t; /* tile origin in texels */
    float shift_s, shift_t;
    int wrap_s_repeat, wrap_t_repeat;
    int clamp_s_materialized, clamp_t_materialized;
} TexInfo;

static int tile_shift_mul(int shift, float *mul)
{
    if (shift == 0)
        *mul = 1.0f;
    else if (shift <= 10)
        *mul = 1.0f / (float)(1 << shift);
    else
        *mul = (float)(1 << (16 - shift));
    return 0;
}

static void bind_texture(int tile_index, TexInfo *ti)
{
    const GbiTile *t = &R.tiles[tile_index & 7];
    PS2TexKey key;
    int li = find_load(t->tmem, 0);
    int w, h;
    uint32_t tmem_bytes;
    uint32_t tlut_type = (R.om_h >> 14) & 3;

    ti->valid = 0;
    ti->clamp_s_materialized = 0;
    ti->clamp_t_materialized = 0;
    if (li < 0)
    {
        return;
    }
    memset(&key, 0, sizeof(key));

    /*
     * With a non-zero mask, the RDP can wrap/mirror *inside* the legal
     * SL/TL..SH/TH tile and still clamp outside that tile.  One repeating GS
     * period cannot represent both operations.  Keep the mask period as the
     * source extent and, on a clamped axis, ask texcache to materialize the
     * complete legal tile before REGION_CLAMP is applied.
     *
     * Nintendo's documented example (SH=11, mask=2, mirror+clamp) produces
     * 0,1,2,3,3,2,1,0,0,1,2,3,3,3...; the first 12 texels are the
     * materialized masked/mirrored tile and the last value is then clamped.
     */
    w = ((t->lrs - t->uls) >> 2) + 1;
    h = ((t->lrt - t->ult) >> 2) + 1;
    if (w <= 0 || h <= 0 || w > 1024 || h > 1024)
    {
        return;
    }

    ti->wrap_s_repeat = t->masks && (!(t->cms & G_TX_CLAMP) || w > (1 << t->masks));
    ti->wrap_t_repeat = t->maskt && (!(t->cmt & G_TX_CLAMP) || h > (1 << t->maskt));

    if (ti->wrap_s_repeat)
    {
        if (t->cms & G_TX_CLAMP)
        {
            key.clamp_width = (uint16_t)w;
            ti->clamp_s_materialized = 1;
        }
        w = 1 << t->masks;
    }
    if (ti->wrap_t_repeat)
    {
        if (t->cmt & G_TX_CLAMP)
        {
            key.clamp_height = (uint16_t)h;
            ti->clamp_t_materialized = 1;
        }
        h = 1 << t->maskt;
    }

    /* 32-bit texels are split across TMEM's two halves (RG low, BA high), so
     * tile line and TMEM addresses count half-words: one TMEM word there
     * holds 16 source bytes. */
    tmem_bytes = (t->siz == G_IM_SIZ_32b) ? 16u : 8u;
    key.addr = R.loads[li].src + (uint32_t)(t->tmem - R.loads[li].tmem) * tmem_bytes;
    key.width = (uint16_t)w;
    key.height = (uint16_t)h;
    key.fmt = t->fmt;
    key.siz = t->siz;
    key.line_bytes = (uint16_t)((t->line ? t->line : 1) * tmem_bytes);
    if (R.loads[li].pitch != 0 && R.loads[li].siz == t->siz && t->line * tmem_bytes != R.loads[li].pitch)
    {
        /* LoadTile rows keep the DRAM image pitch. */
        key.line_bytes = (uint16_t)R.loads[li].pitch;
    }
    key.mirror_s = (t->cms & G_TX_MIRROR) && ti->wrap_s_repeat;
    key.mirror_t = (t->cmt & G_TX_MIRROR) && ti->wrap_t_repeat;
    key.odd_swap = R.loads[li].odd_swap;

    if (t->fmt == G_IM_FMT_CI)
    {
        uint16_t pal_tmem = (uint16_t)(256 + ((t->siz == G_IM_SIZ_4b) ? (t->palette * 16) : 0));
        int pl = find_load(pal_tmem, 1);

        if (pl < 0)
        {
            return;
        }
        key.tlut = R.loads[pl].src + (uint32_t)(pal_tmem - R.loads[pl].tmem) * 2u;
        if (gPS2TlutTrace > 0)
        {
            gPS2TlutTrace--;
            ps2_log("ci bind tex %p tlut %p (tlut load cmd %p tmem %u+%u) tex load cmd %p at cmd %p", key.addr,
                    key.tlut, R.loads[pl].cmd, R.loads[pl].tmem, R.loads[pl].words, R.loads[li].cmd, sCurCmd);
        }
        key.tlut_type = (uint8_t)((tlut_type == 3) ? 3 : 2);
        key.pal_index = t->palette;
    }

    {
        extern int gPS2TexJustConverted;

        gPS2TexJustConverted = 0;
        if (!ps2_texcache_bind(&key, &ti->bind))
        {
            return;
        }
        if (gPS2TexJustConverted)
        {
            ps2_log(" tile%d fmt%d siz%d line%d tmem%d ms%d mt%d sh%d/%d cm%d/%d ul%d,%d lr%d,%d load:tmem%d pitch%u siz%d",
                    tile_index & 7, t->fmt, t->siz, t->line, t->tmem, t->masks, t->maskt, t->shifts, t->shiftt, t->cms,
                    t->cmt, t->uls, t->ult, t->lrs, t->lrt, R.loads[li].tmem, (unsigned)R.loads[li].pitch,
                    R.loads[li].siz);
        }
    }
    tile_shift_mul(t->shifts, &ti->shift_s);
    tile_shift_mul(t->shiftt, &ti->shift_t);
    ti->off_s = (float)t->uls * 0.25f;
    ti->off_t = (float)t->ult * 0.25f;
    ti->valid = 1;
}

/* ------------------------------------------------------------------ */
/* Other modes -> GS state                                              */
/* ------------------------------------------------------------------ */

static uint64_t gs_scissor(void)
{
    int x0 = R.scissor[0] >> 2, y0 = R.scissor[1] >> 2;
    int x1 = ((R.scissor[2] + 3) >> 2) - 1, y1 = ((R.scissor[3] + 3) >> 2) - 1;

    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > PS2_SCREEN_W - 1) x1 = PS2_SCREEN_W - 1;
    if (y1 > PS2_SCREEN_H - 1) y1 = PS2_SCREEN_H - 1;
    if (x1 < x0) x1 = x0;
    if (y1 < y0) y1 = y0;
    return GSV_SCISSOR(x0, x1, y0, y1);
}

typedef struct DrawMode
{
    GsState gs;
    int textured;
    int fog_blend;  /* use GS fog with vertex fog factor */
    int prim_depth; /* G_ZS_PRIM */
    int decal;      /* two passes: untextured base, then texel-alpha blended texels */
    GsState decal_gs;
    TexInfo tex;
} DrawMode;

static void build_mode(DrawMode *dm, int for_rect)
{
    uint32_t cyc = R.om_h & (3u << 20);
    uint32_t l = R.om_l;
    uint32_t bl = (cyc == G_CYC_2CYCLE) ? (l >> 16) & 0x3333 : (l >> 16) & 0xCCCC;
    int p, a, m, b;
    int z_cmp, z_upd, zmode;
    int ate = 0, atst = GSATST_ALWAYS, aref = 0;
    int abe = 0;
    uint64_t alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_AS, GSBL_CD, 0);

    memset(dm, 0, sizeof(*dm));
    /* Texture uploads below append to the packet: never inside a batch. */
    batch_close();

    /* ---- texture ---- */
    dm->textured = 0;
    if ((cyc == G_CYC_COPY) || (cyc != G_CYC_FILL && (R.tex_on || for_rect) && combiner_uses_texture()))
    {
        bind_texture(for_rect ? R.rect_tile : R.tex_tile, &dm->tex);
        dm->textured = dm->tex.valid;
    }

    /* ---- blender ---- */
    if (cyc == G_CYC_2CYCLE)
    {
        p = (l >> 28) & 3; a = (l >> 24) & 3; m = (l >> 20) & 3; b = (l >> 16) & 3;
        /* the first cycle commonly applies fog */
        if (((l >> 30) & 3) == G_BL_CLR_FOG && ((l >> 26) & 3) == G_BL_A_SHADE)
        {
            dm->fog_blend = 1;
        }
    }
    else
    {
        p = (l >> 30) & 3; a = (l >> 26) & 3; m = (l >> 22) & 3; b = (l >> 18) & 3;
    }
    (void)bl;

    if (cyc == G_CYC_1CYCLE || cyc == G_CYC_2CYCLE)
    {
        if ((l & FORCE_BL) && p == G_BL_CLR_IN && m == G_BL_CLR_MEM)
        {
            if (a == G_BL_A_IN && b == G_BL_1MA)
            {
                abe = 1;
                alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_AS, GSBL_CD, 0);
            }
            else if (a == G_BL_A_IN && b == G_BL_1)
            {
                abe = 1; /* additive */
                alpha = GSV_ALPHA(GSBL_CS, GSBL_ZERO, GSBL_AS, GSBL_CD, 0);
            }
            else if (a == G_BL_A_FOG && b == G_BL_1MA)
            {
                abe = 1;
                alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_FIX, GSBL_CD, (R.fog[3] + 1) >> 1);
            }
            else if (a == G_BL_0 && b == G_BL_1)
            {
                abe = 1; /* keep destination */
                alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_FIX, GSBL_CD, 0);
            }
        }
        else if (p == G_BL_CLR_IN && m == G_BL_CLR_MEM && a == G_BL_A_IN && b == G_BL_1MA && (l & IM_RD) &&
                 !(l & CVG_X_ALPHA))
        {
            abe = 1;
        }
        if (p == G_BL_CLR_FOG && cyc == G_CYC_1CYCLE && a == G_BL_A_SHADE)
        {
            dm->fog_blend = 1;
        }
    }
    if (!(R.geom & G_FOG))
    {
        dm->fog_blend = 0;
    }

    /* ---- alpha test ---- */
    if (cyc != G_CYC_FILL)
    {
        uint32_t ac = l & 3;

        if (ac == G_AC_THRESHOLD)
        {
            ate = 1;
            atst = GSATST_GREATER;
            aref = R.blend[3] >> 1;
        }
        else if ((l & CVG_X_ALPHA) && (l & ALPHA_CVG_SEL))
        {
            /* coverage-from-alpha edges (G_RM_*TEX_EDGE) */
            ate = 1;
            atst = GSATST_GEQUAL;
            aref = 0x40;
        }
    }

    /* ---- depth ---- */
    z_cmp = (l & Z_CMP) && (R.geom & G_ZBUFFER || (l & G_ZS_PRIM)) && cyc != G_CYC_FILL && cyc != G_CYC_COPY;
    z_upd = (l & Z_UPD) && (R.geom & G_ZBUFFER || (l & G_ZS_PRIM)) && cyc != G_CYC_FILL && cyc != G_CYC_COPY;
    zmode = (int)(l & ZMODE_DEC);
    if (zmode == ZMODE_XLU)
    {
        z_upd = z_upd && !abe ? z_upd : 0;
    }
    dm->prim_depth = (l & G_ZS_PRIM) != 0;

    dm->gs.test = GSV_TEST(ate, atst, aref, GSAFAIL_KEEP, 0, 0, 1, z_cmp ? GSZTST_GEQUAL : GSZTST_ALWAYS);
    dm->gs.zbuf = GSV_ZBUF(PS2_Z_PAGE, PS2_Z_PSM, z_upd ? 0 : 1);
    dm->gs.alpha = alpha;
    dm->gs.fogcol = GSV_FOGCOL(R.fog[0], R.fog[1], R.fog[2]);
    dm->gs.scissor = gs_scissor();
    dm->gs.use_fog = dm->fog_blend;

    if (dm->textured)
    {
        uint32_t filt = (R.om_h >> 12) & 3; /* 0 point, 2 bilerp, 3 average */
        int lin = (filt != 0) && cyc != G_CYC_COPY;
        const TexInfo *ti = &dm->tex;
        int wms = (ti->wrap_s_repeat && !ti->clamp_s_materialized) ? GSWRAP_REPEAT : GSWRAP_REGION_CLAMP;
        int wmt = (ti->wrap_t_repeat && !ti->clamp_t_materialized) ? GSWRAP_REPEAT : GSWRAP_REGION_CLAMP;
        const GbiTile *t = &R.tiles[(for_rect ? R.rect_tile : R.tex_tile) & 7];
        int maxu = ((t->lrs - t->uls) >> 2);
        int maxv = ((t->lrt - t->ult) >> 2);

        if (maxu < 0 || maxu >= ti->bind.gs_w) maxu = ti->bind.gs_w - 1;
        if (maxv < 0 || maxv >= ti->bind.gs_h) maxv = ti->bind.gs_h - 1;

        dm->gs.tex0 = ti->bind.tex0 | ((uint64_t)1 << 34) /* TCC rgba */ | ((uint64_t)GSTFX_MODULATE << 35);
        dm->gs.clamp = GSV_CLAMP(wms, wmt, 0, maxu, 0, maxv);
        dm->gs.tex1 = GSV_TEX1(1, 0, lin, lin, 0, 0, 0);
        dm->gs.nreg = 3;
    }
    else
    {
        dm->gs.tex0 = sCurValid ? sCur.tex0 : 0;
        dm->gs.clamp = sCurValid ? sCur.clamp : 0;
        dm->gs.tex1 = sCurValid ? sCur.tex1 : 0;
        dm->gs.nreg = 2;
    }
    dm->gs.prim = GSV_PRIM(for_rect ? GSPRIM_SPRITE : GSPRIM_TRI, (R.geom & G_SHADING_SMOOTH) ? 1 : 0,
                          dm->textured, dm->fog_blend, abe, 0, for_rect ? 1 : 0, 0, 0);

    if (gPS2CombTrace && dm->textured)
    {
        /* debug: log each distinct textured combiner setup once */
        static uint32_t seen[64][3];
        static int nseen;
        const GbiTile *t = &R.tiles[R.tex_tile & 7];
        uint32_t key2 = R.om_h ^ (R.om_l << 1) ^ ((uint32_t)t->fmt << 28) ^ ((uint32_t)t->siz << 26);
        int i;

        for (i = 0; i < nseen; i++)
            if (seen[i][0] == R.cc_w0 && seen[i][1] == R.cc_w1 && seen[i][2] == key2)
                break;
        if (i == nseen && nseen < 64)
        {
            seen[nseen][0] = R.cc_w0;
            seen[nseen][1] = R.cc_w1;
            seen[nseen][2] = key2;
            nseen++;
            ps2_log("comb %08x %08x omh %08x oml %08x fmt%d siz%d abe%d prim %02x%02x%02x%02x env %02x%02x%02x%02x",
                    (unsigned)R.cc_w0, (unsigned)R.cc_w1, (unsigned)R.om_h, (unsigned)R.om_l, t->fmt, t->siz, abe,
                    R.prim[0], R.prim[1], R.prim[2], R.prim[3], R.env[0], R.env[1], R.env[2], R.env[3]);
        }
    }

    /* Opaque surfaces whose color is lerped by texel alpha: see sTexAlphaIn.
     * The second pass draws on the depth the first one wrote. */
    dm->decal = dm->textured && !abe && !for_rect && combiner_color_lerps_by_texel_alpha();
    if (dm->decal)
    {
        dm->decal_gs = dm->gs;
        dm->decal_gs.test = GSV_TEST(1, GSATST_GREATER, 0, GSAFAIL_KEEP, 0, 0, 1,
                                     z_cmp ? GSZTST_GEQUAL : GSZTST_ALWAYS);
        dm->decal_gs.zbuf = GSV_ZBUF(PS2_Z_PAGE, PS2_Z_PSM, 1);
        dm->decal_gs.alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_AS, GSBL_CD, 0);
        dm->decal_gs.prim = GSV_PRIM(GSPRIM_TRI, (R.geom & G_SHADING_SMOOTH) ? 1 : 0, 1, dm->fog_blend, 1, 0, 0, 0, 0);
        /* base pass: same state without the texture */
        dm->gs.prim = GSV_PRIM(GSPRIM_TRI, (R.geom & G_SHADING_SMOOTH) ? 1 : 0, 0, dm->fog_blend, 0, 0, 0, 0, 0);
        dm->gs.nreg = 2;
    }
}

/* ------------------------------------------------------------------ */
/* Triangle emission                                                    */
/* ------------------------------------------------------------------ */

typedef struct OutVtx
{
    float x, y, z, w, s, t;
    float r, g, b, a; /* GS vertex color 0..255 (0x80 = 1.0 for modulate) */
    float fog;
} OutVtx;

static void make_outvtx(const GbiVtx *v, const DrawMode *dm, OutVtx *o)
{
    CombineOut co;

    eval_combiner(v, &co);
    o->x = v->x;
    o->y = v->y;
    o->z = v->z;
    o->w = v->w;
    o->fog = (float)v->fog;
    if (dm->textured)
    {
        const TexInfo *ti = &dm->tex;

        o->r = (co.rgb[0].k + co.rgb[0].c) * 128.0f;
        o->g = (co.rgb[1].k + co.rgb[1].c) * 128.0f;
        o->b = (co.rgb[2].k + co.rgb[2].c) * 128.0f;
        o->a = (co.a.k + co.a.c) * 128.0f;
        o->s = (v->s * ti->shift_s - ti->off_s) / (float)ti->bind.gs_w;
        o->t = (v->t * ti->shift_t - ti->off_t) / (float)ti->bind.gs_h;
    }
    else
    {
        o->r = co.rgb[0].c * 255.0f;
        o->g = co.rgb[1].c * 255.0f;
        o->b = co.rgb[2].c * 255.0f;
        o->a = co.a.c * 128.0f;
        o->s = o->t = 0.0f;
    }
}

static void lerp_out(OutVtx *r, const OutVtx *a, const OutVtx *b, float t)
{
    r->x = a->x + (b->x - a->x) * t;
    r->y = a->y + (b->y - a->y) * t;
    r->z = a->z + (b->z - a->z) * t;
    r->w = a->w + (b->w - a->w) * t;
    r->s = a->s + (b->s - a->s) * t;
    r->t = a->t + (b->t - a->t) * t;
    r->r = a->r + (b->r - a->r) * t;
    r->g = a->g + (b->g - a->g) * t;
    r->b = a->b + (b->b - a->b) * t;
    r->a = a->a + (b->a - a->a) * t;
    r->fog = a->fog + (b->fog - a->fog) * t;
}

/* Signed distance to clip plane i (inside >= 0). */
static float plane_dist(const OutVtx *v, int i)
{
    switch (i)
    {
    case 0: return v->z + v->w;               /* near */
    case 1: return GUARD * v->w - v->x;
    case 2: return GUARD * v->w + v->x;
    case 3: return GUARD * v->w - v->y;
    default: return GUARD * v->w + v->y;
    }
}

static int clip_poly(OutVtx *in, int n, OutVtx *tmp, int planes)
{
    int p;
    OutVtx *src = in, *dst = tmp;

    for (p = 0; p < 5; p++)
    {
        int i, m = 0;

        if (!(planes & (1 << p)))
            continue;
        for (i = 0; i < n; i++)
        {
            const OutVtx *a = &src[i];
            const OutVtx *b = &src[(i + 1) % n];
            float da = plane_dist(a, p), db = plane_dist(b, p);

            if (da >= 0.0f)
                dst[m++] = *a;
            if ((da >= 0.0f) != (db >= 0.0f))
                lerp_out(&dst[m++], a, b, da / (da - db));
        }
        n = m;
        if (n < 3)
            return 0;
        {
            OutVtx *t = src;
            src = dst;
            dst = t;
        }
    }
    if (src != in)
        memcpy(in, src, sizeof(OutVtx) * (size_t)n);
    return n;
}

static void emit_vertex(uint64_t *q, const OutVtx *v, const DrawMode *dm)
{
    float wi = 1.0f / v->w;
    float sx = v->x * wi * R.vp_scale[0] + R.vp_trans[0];
    float sy = v->y * wi * R.vp_scale[1] + R.vp_trans[1];
    float zn = v->z * wi;
    uint32_t x = (uint32_t)(int32_t)((2048.0f + sx) * 16.0f);
    uint32_t y = (uint32_t)(int32_t)((2048.0f + sy) * 16.0f);
    uint32_t z;
    uint32_t r = (uint32_t)clamp_u8(v->r), g = (uint32_t)clamp_u8(v->g), b = (uint32_t)clamp_u8(v->b);
    uint32_t a = (uint32_t)clamp_u8(v->a);

    if (dm->prim_depth)
    {
        z = (uint32_t)(0xFFFF - ((R.prim_z > 0x7FFF) ? 0xFFFF : (uint32_t)R.prim_z * 2));
    }
    else
    {
        float zz = (1.0f - zn) * 32767.5f;

        z = (zz <= 0.0f) ? 0 : (zz >= 65535.0f) ? 65535 : (uint32_t)zz;
    }

    if (dm->textured)
    {
        gs_packed_stq(q, v->s * wi, v->t * wi, wi);
        q += 2;
    }
    gs_packed_rgba(q, r, g, b, a);
    q += 2;
    if (dm->fog_blend)
    {
        gs_packed_xyzf2(q, x, y, z, 255u - (uint32_t)clamp_u8(v->fog));
    }
    else
    {
        gs_packed_xyz2(q, x, y, z);
    }
}

static int cull(const OutVtx *v0, const OutVtx *v1, const OutVtx *v2)
{
    uint32_t mode = R.geom & G_CULL_BOTH;
    float x0, y0, x1, y1, x2, y2, cross;

    if (!mode)
        return 0;
    if (mode == G_CULL_BOTH)
        return 1;
    x0 = v0->x / v0->w; y0 = v0->y / v0->w;
    x1 = v1->x / v1->w; y1 = v1->y / v1->w;
    x2 = v2->x / v2->w; y2 = v2->y / v2->w;
    cross = (x1 - x0) * (y2 - y0) - (y1 - y0) * (x2 - x0);
    if (mode == G_CULL_FRONT)
        return cross >= 0.0f;
    return cross <= 0.0f; /* G_CULL_BACK */
}

static DrawMode sMode;
static int sModeDirty = 1;

static void tri_pass(const GbiVtx *a, const GbiVtx *b, const GbiVtx *c, const DrawMode *dm);

static void tri(int i0, int i1, int i2)
{
    /*
     * The N64 sometimes renders intermediate effects into color images that
     * are not one of the three VI framebuffers. We do not emulate those
     * off-screen render targets yet. Drawing those commands into the last
     * on-screen framebuffer is worse than omitting the unsupported effect:
     * it produces stray full-width strips/panels and apparent "jitter".
     */
    if (R.cimg_offscreen)
        return;
    if (i0 >= MAX_VTX || i1 >= MAX_VTX || i2 >= MAX_VTX)
        return;
    if (sModeDirty)
    {
        build_mode(&sMode, 0);
        sModeDirty = 0;
    }
    if (sMode.decal)
    {
        DrawMode tex_pass = sMode;

        sMode.textured = 0;
        sTexAlphaIn = 0.0f;
        tri_pass(&R.vtx[i0], &R.vtx[i1], &R.vtx[i2], &sMode);
        sMode.textured = 1;
        tex_pass.gs = tex_pass.decal_gs;
        sTexAlphaIn = 1.0f;
        tri_pass(&R.vtx[i0], &R.vtx[i1], &R.vtx[i2], &tex_pass);
        return;
    }
    tri_pass(&R.vtx[i0], &R.vtx[i1], &R.vtx[i2], &sMode);
}

#if PS2_DEBUG
/* ------------------------------------------------------------------ */
/* Row-coverage capture (hardware diagnostic, Select + R1)              */
/* ------------------------------------------------------------------ */

/* When armed, every primitive drawn into a colour framebuffer is recorded
 * in GS coordinates; at the end of the frame the rows no primitive covers
 * (by the GS rule: a row r is drawn when y0 <= r < y1, clipped to the
 * scissor) are written to the log together with the primitives bordering
 * them, and the log is saved to SSB64.LOG. */
enum { DIAG_FILL, DIAG_FILL_CYC, DIAG_TEXRECT, DIAG_TRI };
static const char *const sDiagKindName[] = { "fill", "fill1c", "texrect", "tri" };

typedef struct DiagPrim
{
    int32_t x0, y0, x1, y1; /* 1/16 px, primitive bounds */
    int16_t sy0, sy1;       /* scissor rows, inclusive */
    int16_t sx0, sx1;       /* scissor columns, inclusive */
    uint8_t kind, fb;
    uint32_t cmd;           /* display-list command index */
} DiagPrim;

#define DIAG_MAX 6144
static DiagPrim sDiag[DIAG_MAX];
static int sDiagCount;
static int sDiagOverflow;
int gPS2DiagCaptureFrames; /* frames still to capture */

static void diag_add(int kind, float x0, float y0, float x1, float y1, int zonly)
{
    DiagPrim *d;
    uint64_t sc;

    if (gPS2DiagCaptureFrames <= 0 || zonly || R.color_target < 0)
        return;
    if (sDiagCount >= DIAG_MAX)
    {
        sDiagOverflow = 1;
        return;
    }
    sc = gs_scissor();
    d = &sDiag[sDiagCount++];
    d->x0 = (int32_t)(x0 * 16.0f);
    d->y0 = (int32_t)(y0 * 16.0f);
    d->x1 = (int32_t)(x1 * 16.0f);
    d->y1 = (int32_t)(y1 * 16.0f);
    d->sy0 = (int16_t)((sc >> 32) & 0x7FF);
    d->sy1 = (int16_t)((sc >> 48) & 0x7FF);
    d->sx0 = (int16_t)(sc & 0x7FF);
    d->sx1 = (int16_t)((sc >> 16) & 0x7FF);
    d->kind = (uint8_t)kind;
    d->fb = (uint8_t)R.color_target;
    d->cmd = gPS2RenderStats.dl_commands;
}

/* first/last row a primitive covers by the GS rule, clipped to scissor */
static void diag_rows(const DiagPrim *d, int *first, int *last)
{
    int f = (d->y0 + 15) >> 4; /* ceil */
    int l = ((d->y1 + 15) >> 4) - 1;

    if (f < d->sy0) f = d->sy0;
    if (l > d->sy1) l = d->sy1;
    *first = f;
    *last = l;
}

static void diag_log_prim(const char *tag, const DiagPrim *d)
{
    int f, l;

    diag_rows(d, &f, &l);
    /* coordinates in 1/16 pixel (no %f: keep to integer printf) */
    ps2_log("diag:  %s %s cmd %u fb%d y16 %d..%d (rows %d..%d) x16 %d..%d scis %d..%d", tag,
            sDiagKindName[d->kind], (unsigned)d->cmd, d->fb, (int)d->y0, (int)d->y1, f, l, (int)d->x0, (int)d->x1,
            d->sy0, d->sy1);
}

static void diag_cols(const DiagPrim *d, int *first, int *last)
{
    int f = (d->x0 + 15) >> 4;
    int l = ((d->x1 + 15) >> 4) - 1;

    if (f < d->sx0) f = d->sx0;
    if (l > d->sx1) l = d->sx1;
    if (f < 0) f = 0;
    if (l > PS2_SCREEN_W - 1) l = PS2_SCREEN_W - 1;
    *first = f;
    *last = l;
}

static void diag_finish_frame(void)
{
    /* per-pixel coverage map of the target framebuffer, by the GS rule */
    static uint32_t cov[PS2_SCREEN_H][PS2_SCREEN_W / 32];
    static uint16_t cnt[PS2_SCREEN_H];
    int i, r, c, fb = sLastColorTarget, reported = 0, lo = PS2_SCREEN_H, hi = -1, best = 0;

    if (gPS2DiagCaptureFrames <= 0)
        return;
    memset(cov, 0, sizeof(cov));
    for (i = 0; i < sDiagCount; i++)
    {
        const DiagPrim *d = &sDiag[i];
        int f, l, cf, cl;

        if (d->fb != fb)
            continue;
        diag_rows(d, &f, &l);
        diag_cols(d, &cf, &cl);
        if (f < 0) f = 0;
        if (l > PS2_SCREEN_H - 1) l = PS2_SCREEN_H - 1;
        if (f > l || cf > cl)
            continue;
        if (f < lo) lo = f;
        if (l > hi) hi = l;
        for (r = f; r <= l; r++)
            for (c = cf; c <= cl; c++)
                cov[r][c >> 5] |= 1u << (c & 31);
    }
    for (r = 0; r < PS2_SCREEN_H; r++)
    {
        int n = 0;

        for (c = 0; c < PS2_SCREEN_W / 32; c++)
            n += __builtin_popcount(cov[r][c]);
        cnt[r] = (uint16_t)n;
        if (r >= lo && r <= hi && n > best)
            best = n;
    }
    ps2_log("diag: frame fb%d, %d prims%s, drawn rows %d..%d, widest row %d px", fb, sDiagCount,
            sDiagOverflow ? " (overflow)" : "", lo, hi, best);

    /* rows inside the drawn area with fewer pixels than the widest row */
    for (r = (lo < 0 ? 0 : lo); r <= hi; r++)
    {
        int e, n = 0, gx0 = -1, gx1 = -1;

        if (cnt[r] >= best)
            continue;
        for (e = r; e + 1 <= hi && cnt[e + 1] == cnt[r] && memcmp(cov[e + 1], cov[r], sizeof(cov[r])) == 0; e++)
            ;
        /* first hole in the row, relative to the widest row's span */
        for (c = 0; c < PS2_SCREEN_W; c++)
        {
            int set = (cov[r][c >> 5] >> (c & 31)) & 1;

            if (!set && gx0 < 0 && c >= 10 && c < PS2_SCREEN_W - 10)
                gx0 = c;
            if (gx0 >= 0 && set)
            {
                gx1 = c - 1;
                break;
            }
        }
        ps2_log("diag: HOLE rows %d..%d: %d of %d px covered, first gap x %d..%d", r, e, cnt[r], best, gx0,
                gx1 < 0 ? PS2_SCREEN_W - 1 : gx1);
        for (i = 0; i < sDiagCount && n < 10; i++)
        {
            const DiagPrim *d = &sDiag[i];
            int f, l, cf, cl;

            if (d->fb != fb)
                continue;
            diag_rows(d, &f, &l);
            diag_cols(d, &cf, &cl);
            if (gx0 >= 0 && (cl < gx0 || cf > gx0))
                continue; /* not in the gap's column */
            if (l == r - 1 || f == e + 1)
            {
                diag_log_prim(l == r - 1 ? "above" : "below", d);
                n++;
            }
        }
        if (++reported >= 6)
            break;
        r = e;
    }
    if (reported == 0)
        ps2_log("diag: every pixel of rows %d..%d is covered", lo, hi);
    sDiagCount = 0;
    sDiagOverflow = 0;
    if (--gPS2DiagCaptureFrames == 0)
    {
        extern uint32_t gPS2FbDrawDisplayed, gPS2FbDrawPending;

        ps2_log("diag: capture done; vsync: drew into on-screen fb %u, into queued fb %u",
                (unsigned)gPS2FbDrawDisplayed, (unsigned)gPS2FbDrawPending);
        ps2_log_save();
    }
}

#else /* !PS2_DEBUG: the capture is compiled out */

#define diag_add(kind, x0, y0, x1, y1, zonly) ((void)0)
#define diag_finish_frame() ((void)0)

#endif /* PS2_DEBUG */

static void tri_pass(const GbiVtx *a, const GbiVtx *b, const GbiVtx *c, const DrawMode *dm)
{
    OutVtx poly[12], tmp[12];
    uint8_t clip_or = a->clip | b->clip | c->clip;
    int n = 3, i, planes = 0;
    uint64_t *q;

    make_outvtx(a, dm, &poly[0]);
    make_outvtx(b, dm, &poly[1]);
    make_outvtx(c, dm, &poly[2]);

    if (clip_or)
    {
        if (clip_or & CLIP_NEAR)
            planes |= 1;
        if (clip_or & CLIP_GUARD)
            planes |= 0x1E;
        n = clip_poly(poly, 3, tmp, planes);
        if (n < 3)
            return;
    }
    if (cull(&poly[0], &poly[1], &poly[2]))
        return;

    /* Fan-triangulate the (possibly clipped) polygon. */
    q = batch_reserve(&dm->gs, (uint32_t)(n - 2) * 3);
    for (i = 1; i + 1 < n; i++)
    {
        int stride = dm->gs.nreg * 2;

        emit_vertex(q, &poly[0], dm);
        emit_vertex(q + stride, &poly[i], dm);
        emit_vertex(q + stride * 2, &poly[i + 1], dm);
        q += stride * 3;
#if PS2_DEBUG
        if (gPS2DiagCaptureFrames > 0)
        {
            const OutVtx *t3[3] = { &poly[0], &poly[i], &poly[i + 1] };
            float ymin = 1e9f, ymax = -1e9f, xmin = 1e9f, xmax = -1e9f;
            int k;

            for (k = 0; k < 3; k++)
            {
                float wi = 1.0f / t3[k]->w;
                /* same fixed-point truncation as emit_vertex() */
                float sx = (float)(int32_t)((2048.0f + t3[k]->x * wi * R.vp_scale[0] + R.vp_trans[0]) * 16.0f) / 16.0f - 2048.0f;
                float sy = (float)(int32_t)((2048.0f + t3[k]->y * wi * R.vp_scale[1] + R.vp_trans[1]) * 16.0f) / 16.0f - 2048.0f;

                if (sy < ymin) ymin = sy;
                if (sy > ymax) ymax = sy;
                if (sx < xmin) xmin = sx;
                if (sx > xmax) xmax = sx;
            }
            diag_add(DIAG_TRI, xmin, ymin, xmax, ymax, R.cimg_is_z);
        }
#endif
    }
    gPS2Pkt.ptr = q;
    gPS2RenderStats.triangles += (uint32_t)(n - 2);
}

/* ------------------------------------------------------------------ */
/* Rectangles                                                           */
/* ------------------------------------------------------------------ */

static void unpack_fill_color(uint8_t *rgba)
{
    uint32_t c = R.fill_color >> 16; /* 16-bit fb: color replicated in both halves */

    rgba[0] = (uint8_t)(((c >> 11) & 31) * 255 / 31);
    rgba[1] = (uint8_t)(((c >> 6) & 31) * 255 / 31);
    rgba[2] = (uint8_t)(((c >> 1) & 31) * 255 / 31);
    rgba[3] = (c & 1) ? 255 : 0;
}

static void fill_rect(int ulx, int uly, int lrx, int lry)
{
    uint32_t cyc = R.om_h & (3u << 20);

    if (R.cimg_offscreen)
        return;
    float x0 = ulx * 0.25f, y0 = uly * 0.25f, x1 = lrx * 0.25f, y1 = lry * 0.25f;
    uint64_t *p;

    batch_close();
    if (cyc == G_CYC_FILL || cyc == G_CYC_COPY)
    {
        x1 += 1.0f;
        y1 += 1.0f;
    }
    /* partly covered first row/column: see snap_leading_edge() */
    x0 = floorf(x0);
    y0 = floorf(y0);

    diag_add(cyc == G_CYC_FILL ? DIAG_FILL : DIAG_FILL_CYC, x0, y0, x1, y1, R.cimg_is_z);
    if (cyc == G_CYC_FILL)
    {
        uint8_t c[4];

        if (R.cimg_is_z)
        {
            /* Z buffer clear: write only Z (farthest = 0 on the GS). */
            ps2_pkt_ad_begin(4);
            ps2_pkt_ad(GSR_FRAME_1, GSV_FRAME(PS2_FB_PAGE(R.color_target < 0 ? 0 : R.color_target), PS2_FBW,
                                             PS2_FB_PSM, 0xFFFFFFFFu));
            ps2_pkt_ad(GSR_ZBUF_1, GSV_ZBUF(PS2_Z_PAGE, PS2_Z_PSM, 0));
            ps2_pkt_ad(GSR_TEST_1, GSV_TEST(0, 0, 0, 0, 0, 0, 1, GSZTST_ALWAYS));
            ps2_pkt_ad(GSR_SCISSOR_1, GSV_SCISSOR(0, PS2_SCREEN_W - 1, 0, PS2_SCREEN_H - 1));
            sCurValid = 0;
        }
        else
        {
            ps2_pkt_ad_begin(3);
            ps2_pkt_ad(GSR_TEST_1, GSV_TEST(0, 0, 0, 0, 0, 0, 1, GSZTST_ALWAYS));
            ps2_pkt_ad(GSR_ZBUF_1, GSV_ZBUF(PS2_Z_PAGE, PS2_Z_PSM, 1));
            ps2_pkt_ad(GSR_SCISSOR_1, gs_scissor());
            sCurValid = 0;
        }
        unpack_fill_color(c);
        ps2_pkt_reserve(4);
        p = gPS2Pkt.ptr;
        p[0] = GIFTAG_LO(1, 0, 1, GSV_PRIM(GSPRIM_SPRITE, 0, 0, 0, 0, 0, 0, 0, 0), GIF_FLG_PACKED, 3);
        p[1] = (uint64_t)GSR_RGBAQ | ((uint64_t)GSR_XYZ2 << 4) | ((uint64_t)GSR_XYZ2 << 8);
        gs_packed_rgba(p + 2, c[0], c[1], c[2], 0x80);
        gs_packed_xyz2(p + 4, (uint32_t)((2048.0f + x0) * 16.0f), (uint32_t)((2048.0f + y0) * 16.0f), 0);
        gs_packed_xyz2(p + 6, (uint32_t)((2048.0f + x1) * 16.0f), (uint32_t)((2048.0f + y1) * 16.0f), 0);
        gPS2Pkt.ptr = p + 8;
        if (R.cimg_is_z)
        {
            ps2_pkt_ad_begin(1);
            ps2_pkt_ad(GSR_FRAME_1, GSV_FRAME(PS2_FB_PAGE(R.color_target < 0 ? 0 : R.color_target), PS2_FBW,
                                             PS2_FB_PSM, 0));
        }
    }
    else
    {
        /* 1/2-cycle fill: combiner output (usually prim color). */
        DrawMode dm;
        GbiVtx v;
        OutVtx o;

        build_mode(&dm, 0);
        dm.textured = 0;
        dm.gs.nreg = 2;
        dm.gs.prim = GSV_PRIM(GSPRIM_SPRITE, 0, 0, 0, (dm.gs.prim >> 6) & 1, 0, 0, 0, 0);
        dm.gs.use_fog = 0;
        memset(&v, 0, sizeof(v));
        v.r = v.g = v.b = v.a = 255;
        make_outvtx(&v, &dm, &o);
        p = batch_reserve(&dm.gs, 2);
        gs_packed_rgba(p, clamp_u8(o.r), clamp_u8(o.g), clamp_u8(o.b), clamp_u8(o.a));
        gs_packed_xyz2(p + 2, (uint32_t)((2048.0f + x0) * 16.0f), (uint32_t)((2048.0f + y0) * 16.0f), 0);
        gs_packed_rgba(p + 4, clamp_u8(o.r), clamp_u8(o.g), clamp_u8(o.b), clamp_u8(o.a));
        gs_packed_xyz2(p + 6, (uint32_t)((2048.0f + x1) * 16.0f), (uint32_t)((2048.0f + y1) * 16.0f), 0);
        gPS2Pkt.ptr = p + 8;
        batch_close();
    }
    sModeDirty = 1;
    gPS2RenderStats.rects++;
}

/* The GS UV register holds unsigned 14-bit (10.4) coordinates.  A negative
 * texel coordinate - which the game produces for sprites at sub-pixel
 * positions, e.g. s = -0.25 - wraps to ~1023.75 on a real GS, so under
 * REGION_CLAMP almost the whole sprite samples the texture's last row/column
 * and only a thin line of it shows (PCSX2's hardware renderer hides this).
 * The N64 clamps such coordinates to texel 0; do the same by clipping the
 * rectangle edge [p0, p1] until its texel coordinate t reaches 0. */
static void clip_negative_texcoord(float *p0, float *p1, float *t0, float *t1)
{
    if (*t0 >= 0.0f && *t1 >= 0.0f)
    {
        return;
    }
    if (*t0 < 0.0f && *t1 < 0.0f)
    {
        *t0 = *t1 = 0.0f; /* everything samples the clamped first texel */
    }
    else if (*t0 < 0.0f)
    {
        *p0 += (*p1 - *p0) * (-*t0 / (*t1 - *t0));
        *t0 = 0.0f;
        /* keep the partly covered first row/column, like the RDP does (see
         * snap_leading_edge); it samples the clamped first texel */
        *p0 = floorf(*p0);
    }
    else
    {
        *p1 -= (*p1 - *p0) * (-*t1 / (*t0 - *t1));
        *t1 = 0.0f;
    }
}

/* Coverage of a rectangle's leading (top/left) edge.  The RDP walks
 * quarter-scanlines, so a rectangle whose top edge is at y = 10.25 still
 * draws row 10; the GS draws only pixels whose integer coordinate lies
 * inside the primitive, so it starts at row 11.  Where the game places a
 * background element at a fractional position that row is then left
 * undrawn and shows whatever an earlier frame put in that one of the three
 * rotating framebuffers: a flickering line on hardware (PCSX2's hardware
 * renderer rounds differently and hides it).  Move the edge down to the
 * pixel boundary, extending the texel coordinate t along with it.  Trailing
 * edges already agree: both cover up to ceil(p1) - 1. */
static void snap_leading_edge(float *p0, float p1, float *t0, float t1)
{
    float frac = *p0 - floorf(*p0);

    if (frac > 0.0f && p1 > *p0)
    {
        *t0 -= frac * (t1 - *t0) / (p1 - *p0);
        *p0 -= frac;
    }
}

static void tex_rect(uint32_t w0, uint32_t w1, uint32_t h1, uint32_t h2, int flip)
{
    uint32_t cyc = R.om_h & (3u << 20);

    if (R.cimg_offscreen)
        return;
    int lrx = (w0 >> 12) & 0xFFF, lry = w0 & 0xFFF;
    int tile = (w1 >> 24) & 7;
    int ulx = (w1 >> 12) & 0xFFF, uly = w1 & 0xFFF;
    float s = (float)(int16_t)(h1 >> 16) / 32.0f;
    float t = (float)(int16_t)(h1 & 0xFFFF) / 32.0f;
    float dsdx = (float)(int16_t)(h2 >> 16) / 1024.0f;
    float dtdy = (float)(int16_t)(h2 & 0xFFFF) / 1024.0f;
    float x0 = ulx * 0.25f, y0 = uly * 0.25f, x1 = lrx * 0.25f, y1 = lry * 0.25f;
    DrawMode dm;
    GbiVtx v;
    OutVtx o;
    uint64_t *p;
    float u0, v0, u1, v1;
    uint32_t z = 0;

    if (cyc == G_CYC_COPY)
    {
        dsdx *= 0.25f; /* copy mode draws 4 pixels per step */
        x1 += 1.0f;
        y1 += 1.0f;
    }
    R.rect_tile = tile;
    build_mode(&dm, 1);
    if (!dm.textured)
    {
        return;
    }
    memset(&v, 0, sizeof(v));
    v.r = v.g = v.b = v.a = 255;
    make_outvtx(&v, &dm, &o);
    if (cyc == G_CYC_COPY)
    {
        o.r = o.g = o.b = o.a = 128.0f;
        dm.gs.alpha = GSV_ALPHA(GSBL_CS, GSBL_CD, GSBL_AS, GSBL_CD, 0);
        dm.gs.prim &= ~((uint64_t)1 << 6); /* no blending in copy mode */
    }
    dm.gs.use_fog = 0;
    dm.gs.nreg = 3; /* STQ, RGBAQ, XYZ2 */
    /*
     * Texture rectangles can carry small negative S/T offsets when a sprite
     * moves by a fractional pixel. GS UV is unsigned fixed point, so encoding
     * those values wrapped them to the far edge of the texture and made
     * moving sprites (notably the CSS hand cursor) disappear. Use floating
     * STQ here instead; it preserves signed sub-texel coordinates and lets the
     * existing CLAMP state do the right thing.
     */
    dm.gs.prim &= ~((uint64_t)1 << 8); /* FST=0: STQ */

    /* Texel coordinates relative to the bound tile origin. */
    s = s * dm.tex.shift_s - dm.tex.off_s;
    t = t * dm.tex.shift_t - dm.tex.off_t;
    if (!flip)
    {
        u0 = s;
        v0 = t;
        u1 = s + (x1 - x0) * dsdx;
        v1 = t + (y1 - y0) * dtdy;
    }
    else
    {
        u0 = s;
        v0 = t;
        u1 = s + (y1 - y0) * dsdx;
        v1 = t + (x1 - x0) * dtdy;
    }
    if (!flip)
    {
        snap_leading_edge(&x0, x1, &u0, u1);
        snap_leading_edge(&y0, y1, &v0, v1);
    }
    else
    {
        snap_leading_edge(&x0, x1, &v0, v1);
        snap_leading_edge(&y0, y1, &u0, u1);
    }
    /* Repeat-wrapped axes are fine: 1024 texels is a multiple of every
     * (power-of-two) GS texture size, so the 14-bit wrap lands on the same
     * texel.  Clamped axes need the negative part clipped away. */
    if (!dm.tex.wrap_s_repeat)
    {
        if (!flip)
            clip_negative_texcoord(&x0, &x1, &u0, &u1);
        else
            clip_negative_texcoord(&y0, &y1, &u0, &u1);
    }
    if (!dm.tex.wrap_t_repeat)
    {
        if (!flip)
            clip_negative_texcoord(&y0, &y1, &v0, &v1);
        else
            clip_negative_texcoord(&x0, &x1, &v0, &v1);
    }
    if (dm.prim_depth)
    {
        z = (uint32_t)(0xFFFF - ((R.prim_z > 0x7FFF) ? 0xFFFF : (uint32_t)R.prim_z * 2));
    }
    diag_add(DIAG_TEXRECT, x0, y0, x1, y1, R.cimg_is_z);

    {
        GsState st = dm.gs;

        batch_close();
        if (!sCurValid || memcmp(&sCur, &st, sizeof(st)) != 0)
        {
            gs_apply_state(&st);
        }
        ps2_pkt_reserve(8);
        p = gPS2Pkt.ptr;
        p[0] = GIFTAG_LO(2, 0, 1, st.prim, GIF_FLG_PACKED, 3);
        p[1] = (uint64_t)GSR_ST | ((uint64_t)GSR_RGBAQ << 4) | ((uint64_t)GSR_XYZ2 << 8);
        p += 2;
        gs_packed_stq(p, u0 / (float)dm.tex.bind.gs_w, v0 / (float)dm.tex.bind.gs_h, 1.0f);
        gs_packed_rgba(p + 2, clamp_u8(o.r), clamp_u8(o.g), clamp_u8(o.b), clamp_u8(o.a));
        gs_packed_xyz2(p + 4, (uint32_t)((2048.0f + x0) * 16.0f), (uint32_t)((2048.0f + y0) * 16.0f), z);
        gs_packed_stq(p + 6, u1 / (float)dm.tex.bind.gs_w, v1 / (float)dm.tex.bind.gs_h, 1.0f);
        gs_packed_rgba(p + 8, clamp_u8(o.r), clamp_u8(o.g), clamp_u8(o.b), clamp_u8(o.a));
        gs_packed_xyz2(p + 10, (uint32_t)((2048.0f + x1) * 16.0f), (uint32_t)((2048.0f + y1) * 16.0f), z);
        gPS2Pkt.ptr = p + 12;
        gPS2RenderStats.batches++;
    }
    gPS2RenderStats.rects++;
}

/* ------------------------------------------------------------------ */
/* Display list interpreter                                             */
/* ------------------------------------------------------------------ */

/* Hardware diagnostic (toggled with Select + L3, see render_thread.c):
 * 0 = off, otherwise the RGB colour each framebuffer is cleared to before
 * the game's first draw into it in a frame.  Rows the game leaves undrawn
 * then show in that colour instead of whatever an earlier frame left in
 * that one of the three buffers. */
uint32_t gPS2FrameClearColor;
static int sFrameCleared;

static void set_color_image(const void *addr)
{
    int fb = ps2_gs_fb_index_for(addr);

    batch_close();
    R.cimg_is_z = ps2_gs_is_zbuffer(addr);
    R.cimg_offscreen = 0;
    if (fb >= 0)
    {
        R.color_target = fb;
        sLastColorTarget = fb;
        if (!sFrameCleared)
        {
            extern void ps2_gs_before_draw_into(int fb);

            sFrameCleared = 1;
            ps2_gs_before_draw_into(fb);
            if (gPS2FrameClearColor != 0)
            {
                ps2_gs_clear(fb, gPS2FrameClearColor & 0xFFFFFF, 0);
            }
        }
        ps2_gs_frame_setup(fb);
        sCurValid = 0;
    }
    else if (!R.cimg_is_z)
    {
        /*
         * Off-screen render targets are not implemented yet. Previously we
         * silently left FRAME pointing at the last display framebuffer, so
         * every draw intended for the intermediate image corrupted the
         * visible frame. Suppress those primitives until the display list
         * selects a real framebuffer again. State/TMEM commands still run,
         * so returning to the screen target keeps normal RDP state flow.
         */
        R.cimg_offscreen = 1;
    }
    sModeDirty = 1;
}

static void reset_state(void)
{
    int i;

    memset(&R, 0, sizeof(R));
    for (i = 0; i < 4; i++)
    {
        R.proj[i][i] = 1.0f;
        R.mv[0][i][i] = 1.0f;
    }
    R.mvp_dirty = 1;
    R.vp_scale[0] = PS2_SCREEN_W / 2.0f;
    R.vp_scale[1] = -PS2_SCREEN_H / 2.0f;
    R.vp_scale[2] = 1.0f;
    R.vp_trans[0] = PS2_SCREEN_W / 2.0f;
    R.vp_trans[1] = PS2_SCREEN_H / 2.0f;
    R.scissor[2] = PS2_SCREEN_W << 2;
    R.scissor[3] = PS2_SCREEN_H << 2;
    R.color_target = -1;
    R.tex_scale_s = R.tex_scale_t = 1.0f;
    R.num_lights = 1;
    sBatchTag = NULL;
    sBatchVerts = 0;
    sCurValid = 0;
    sModeDirty = 1;
}

void ps2_gbi_init(void)
{
    reset_state();
}

int ps2_gbi_last_color_target(void)
{
    return sLastColorTarget;
}

#define DL_STACK 18

void ps2_gbi_run(const void *dl_start)
{
    const Gfx *stack[DL_STACK];
    int sp = 0;
    const Gfx *dl = (const Gfx *)dl_start;
    uint32_t guard = 0;

    reset_state();
    sLastColorTarget = -1;
    sFrameCleared = 0;

    while (dl != NULL)
    {
        uint32_t w0 = dl->words.w0;
        uint32_t w1 = dl->words.w1;
        uint8_t op = (uint8_t)(w0 >> 24);

        sCurCmd = dl;
        dl++;
        gPS2RenderStats.dl_commands++;
        if (gPS2GbiTrace > 0)
        {
            gPS2GbiTrace--;
            ps2_log("dl %p: %08x %08x", (const void *)(dl - 1), (unsigned)w0, (unsigned)w1);
        }
        if (++guard > 400000)
        {
            ps2_log("gbi: runaway display list");
            break;
        }

        switch (op)
        {
        case G_NOOP:
        case G_SPNOOP:
        case G_RDPLOADSYNC:
        case G_RDPPIPESYNC:
        case G_RDPTILESYNC:
        case G_RDPFULLSYNC:
        case G_LOAD_UCODE:
        case G_SETKEYGB:
        case G_SETKEYR:
        case G_SETCONVERT:
            break;

        case G_VTX:
        {
            int n = (w0 >> 12) & 0xFF;
            int end = (w0 >> 1) & 0x7F;

            load_vertices((const Vtx *)seg_addr(w1), n, end - n);
            break;
        }

        case G_MODIFYVTX:
        {
            int where = (w0 >> 16) & 0xFF;
            int vi = (w0 & 0xFFFF) >> 1;

            if (vi < MAX_VTX)
            {
                GbiVtx *v = &R.vtx[vi];

                if (where == G_MWO_POINT_RGBA)
                {
                    v->r = w1 >> 24; v->g = w1 >> 16; v->b = w1 >> 8; v->a = w1;
                }
                else if (where == G_MWO_POINT_ST)
                {
                    v->s = (float)(int16_t)(w1 >> 16) / 32.0f;
                    v->t = (float)(int16_t)(w1 & 0xFFFF) / 32.0f;
                }
            }
            break;
        }

        case G_TRI1:
            tri(((w0 >> 16) & 0xFF) / 2, ((w0 >> 8) & 0xFF) / 2, (w0 & 0xFF) / 2);
            break;

        case G_TRI2:
        case G_QUAD:
            tri(((w0 >> 16) & 0xFF) / 2, ((w0 >> 8) & 0xFF) / 2, (w0 & 0xFF) / 2);
            tri(((w1 >> 16) & 0xFF) / 2, ((w1 >> 8) & 0xFF) / 2, (w1 & 0xFF) / 2);
            break;

        case G_CULLDL:
        {
            /* Cull the rest of this list if all vertices [v0,vn] are
             * outside the same frustum side. */
            int v0 = (w0 & 0xFFFF) / 2, vn = (w1 & 0xFFFF) / 2, i;
            int outx0 = 1, outx1 = 1, outy0 = 1, outy1 = 1;

            for (i = v0; i <= vn && i < MAX_VTX; i++)
            {
                const GbiVtx *v = &R.vtx[i];

                if (v->x >= -v->w) outx0 = 0;
                if (v->x <= v->w) outx1 = 0;
                if (v->y >= -v->w) outy0 = 0;
                if (v->y <= v->w) outy1 = 0;
            }
            if (outx0 || outx1 || outy0 || outy1)
            {
                dl = (sp > 0) ? stack[--sp] : NULL;
            }
            break;
        }

        case G_BRANCH_Z:
        {
            int vi = ((w0 >> 12) & 0xFFF) / 5;
            const GbiVtx *v = (vi < MAX_VTX) ? &R.vtx[vi] : NULL;

            if (v != NULL && v->w > 0.0f)
            {
                float zs = (v->z / v->w) * R.vp_scale[2] + R.vp_trans[2];

                if (zs <= (float)(int32_t)w1)
                {
                    dl = (const Gfx *)seg_addr(R.rdphalf1);
                }
            }
            break;
        }

        case G_DL:
            if (((w0 >> 16) & 0xFF) == G_DL_PUSH)
            {
                if (sp < DL_STACK)
                {
                    stack[sp++] = dl;
                }
            }
            dl = (const Gfx *)seg_addr(w1);
            break;

        case G_ENDDL:
            dl = (sp > 0) ? stack[--sp] : NULL;
            break;

        case G_MTX:
        {
            uint32_t p = (w0 & 0xFF) ^ G_MTX_PUSH;
            float m[4][4];

            mtx_from_fixed(m, seg_addr(w1));
            if (p & G_MTX_PROJECTION)
            {
                if (p & G_MTX_LOAD)
                    memcpy(R.proj, m, sizeof(m));
                else
                    mtx_mul(R.proj, m, R.proj);
            }
            else
            {
                if ((p & G_MTX_PUSH) && R.mv_top < MTX_STACK - 1)
                {
                    memcpy(R.mv[R.mv_top + 1], R.mv[R.mv_top], sizeof(m));
                    R.mv_top++;
                }
                if (p & G_MTX_LOAD)
                    memcpy(R.mv[R.mv_top], m, sizeof(m));
                else
                    mtx_mul(R.mv[R.mv_top], m, R.mv[R.mv_top]);
                R.lights_dirty = 1;
            }
            R.mvp_dirty = 1;
            break;
        }

        case G_POPMTX:
        {
            int n = (int)(w1 / 64);

            while (n-- > 0 && R.mv_top > 0)
            {
                R.mv_top--;
            }
            R.mvp_dirty = 1;
            R.lights_dirty = 1;
            break;
        }

        case G_GEOMETRYMODE:
            R.geom = (R.geom & (w0 | 0xFF000000u)) | w1;
            sModeDirty = 1;
            break;

        case G_TEXTURE:
            R.tex_on = (w0 >> 1) & 0x7F ? 1 : 0;
            R.tex_tile = (w0 >> 8) & 7;
            R.tex_scale_s = (float)(w1 >> 16) / 65536.0f;
            R.tex_scale_t = (float)(w1 & 0xFFFF) / 65536.0f;
            if ((w1 >> 16) == 0xFFFF) R.tex_scale_s = 1.0f;
            if ((w1 & 0xFFFF) == 0xFFFF) R.tex_scale_t = 1.0f;
            sModeDirty = 1;
            break;

        case G_MOVEWORD:
        {
            int index = (w0 >> 16) & 0xFF;
            int offset = w0 & 0xFFFF;

            switch (index)
            {
            case G_MW_SEGMENT:
                R.segments[(offset >> 2) & 0xF] = w1 & 0x1FFFFFFFu;
                break;
            case G_MW_NUMLIGHT:
                R.num_lights = (int)(w1 / 24);
                if (R.num_lights > MAX_LIGHTS)
                    R.num_lights = MAX_LIGHTS;
                R.lights_dirty = 1;
                break;
            case G_MW_FOG:
                R.fog_mul = (int16_t)(w1 >> 16);
                R.fog_off = (int16_t)(w1 & 0xFFFF);
                break;
            case G_MW_LIGHTCOL:
            {
                int li = offset / 24;

                if (li <= MAX_LIGHTS && (offset % 24) == 0)
                {
                    R.light_col[li][0] = (float)((w1 >> 24) & 0xFF);
                    R.light_col[li][1] = (float)((w1 >> 16) & 0xFF);
                    R.light_col[li][2] = (float)((w1 >> 8) & 0xFF);
                }
                break;
            }
            default:
                break;
            }
            break;
        }

        case G_MOVEMEM:
        {
            int index = w0 & 0xFF;
            int offset = ((w0 >> 8) & 0xFF) * 8;
            const uint8_t *src = (const uint8_t *)seg_addr(w1);

            if (index == G_MV_VIEWPORT)
            {
                const Vp_t *vp = (const Vp_t *)src;

                R.vp_scale[0] = vp->vscale[0] / 4.0f;
                R.vp_scale[1] = -vp->vscale[1] / 4.0f;
                R.vp_scale[2] = vp->vscale[2] / 4.0f;
                R.vp_trans[0] = vp->vtrans[0] / 4.0f;
                R.vp_trans[1] = vp->vtrans[1] / 4.0f;
                R.vp_trans[2] = vp->vtrans[2] / 4.0f;
            }
            else if (index == G_MV_LIGHT)
            {
                const Light_t *l = (const Light_t *)src;

                if (offset < 48)
                {
                    int la = offset / 24;
                    float *d = R.lookat[la];

                    d[0] = (float)(int8_t)l->dir[0];
                    d[1] = (float)(int8_t)l->dir[1];
                    d[2] = (float)(int8_t)l->dir[2];
                    normalize3(d);
                }
                else
                {
                    int li = (offset - 48) / 24;

                    if (li <= MAX_LIGHTS)
                    {
                        R.light_col[li][0] = l->col[0];
                        R.light_col[li][1] = l->col[1];
                        R.light_col[li][2] = l->col[2];
                        if (li < MAX_LIGHTS)
                        {
                            R.light_dir[li][0] = (float)(int8_t)l->dir[0];
                            R.light_dir[li][1] = (float)(int8_t)l->dir[1];
                            R.light_dir[li][2] = (float)(int8_t)l->dir[2];
                            normalize3(R.light_dir[li]);
                        }
                    }
                }
                R.lights_dirty = 1;
            }
            else if (index == G_MV_MATRIX)
            {
                /* gSPForceMatrix: replaces the combined matrix. */
                mtx_from_fixed(R.mvp, src);
                R.mvp_dirty = 0;
            }
            break;
        }

        case G_RDPHALF_1:
            R.rdphalf1 = w1;
            break;

        case G_RDPHALF_2:
            R.rdphalf2 = w1;
            break;

        case G_SETOTHERMODE_H:
        case G_SETOTHERMODE_L:
        {
            int len = (int)(w0 & 0xFF) + 1;
            int sft = 32 - (int)((w0 >> 8) & 0xFF) - len;
            uint32_t mask = (len >= 32) ? 0xFFFFFFFFu : (((1u << len) - 1u) << sft);

            if (op == G_SETOTHERMODE_H)
                R.om_h = (R.om_h & ~mask) | (w1 & mask);
            else
                R.om_l = (R.om_l & ~mask) | (w1 & mask);
            sModeDirty = 1;
            break;
        }

        case G_RDPSETOTHERMODE:
            R.om_h = w0 & 0x00FFFFFF;
            R.om_l = w1;
            sModeDirty = 1;
            break;

        case G_SETCOMBINE:
            R.cc_w0 = w0 & 0x00FFFFFF;
            R.cc_w1 = w1;
            sModeDirty = 1;
            break;

        case G_SETPRIMCOLOR:
            R.prim[0] = w1 >> 24; R.prim[1] = w1 >> 16; R.prim[2] = w1 >> 8; R.prim[3] = w1;
            R.prim_lod_frac = w0 & 0xFF;
            break;

        case G_SETENVCOLOR:
            R.env[0] = w1 >> 24; R.env[1] = w1 >> 16; R.env[2] = w1 >> 8; R.env[3] = w1;
            break;

        case G_SETFOGCOLOR:
            R.fog[0] = w1 >> 24; R.fog[1] = w1 >> 16; R.fog[2] = w1 >> 8; R.fog[3] = w1;
            sModeDirty = 1;
            break;

        case G_SETBLENDCOLOR:
            R.blend[0] = w1 >> 24; R.blend[1] = w1 >> 16; R.blend[2] = w1 >> 8; R.blend[3] = w1;
            sModeDirty = 1;
            break;

        case G_SETFILLCOLOR:
            R.fill_color = w1;
            break;

        case G_SETPRIMDEPTH:
            R.prim_z = (uint16_t)(w1 >> 16);
            break;

        case G_SETSCISSOR:
            R.scissor[0] = (w0 >> 12) & 0xFFF;
            R.scissor[1] = w0 & 0xFFF;
            R.scissor[2] = (w1 >> 12) & 0xFFF;
            R.scissor[3] = w1 & 0xFFF;
            sModeDirty = 1;
            break;

        case G_SETCIMG:
            set_color_image(seg_addr(w1));
            break;

        case G_SETZIMG:
            break;

        case G_SETTIMG:
            R.timg_fmt = (w0 >> 21) & 7;
            R.timg_siz = (w0 >> 19) & 3;
            R.timg_width = (uint16_t)((w0 & 0xFFF) + 1);
            R.timg_addr = (const uint8_t *)seg_addr(w1);
            break;

        case G_SETTILE:
        {
            GbiTile *t = &R.tiles[(w1 >> 24) & 7];

            t->fmt = (w0 >> 21) & 7;
            t->siz = (w0 >> 19) & 3;
            t->line = (w0 >> 9) & 0x1FF;
            t->tmem = w0 & 0x1FF;
            t->palette = (w1 >> 20) & 0xF;
            t->cmt = (w1 >> 18) & 3;
            t->maskt = (w1 >> 14) & 0xF;
            t->shiftt = (w1 >> 10) & 0xF;
            t->cms = (w1 >> 8) & 3;
            t->masks = (w1 >> 4) & 0xF;
            t->shifts = w1 & 0xF;
            sModeDirty = 1;
            break;
        }

        case G_SETTILESIZE:
        {
            GbiTile *t = &R.tiles[(w1 >> 24) & 7];

            t->uls = (w0 >> 12) & 0xFFF;
            t->ult = w0 & 0xFFF;
            t->lrs = (w1 >> 12) & 0xFFF;
            t->lrt = w1 & 0xFFF;
            sModeDirty = 1;
            break;
        }

        case G_LOADBLOCK:
        {
            const GbiTile *t = &R.tiles[(w1 >> 24) & 7];
            uint32_t texels = ((w1 >> 12) & 0xFFF) + 1;
            uint32_t dxt = w1 & 0xFFF;
            uint32_t bpp = (4u << R.timg_siz); /* bits */
            /* TMEM words used; 32-bit texels occupy both halves in parallel */
            uint32_t words = (texels * ((bpp == 32) ? 16u : bpp) + 63) / 64;
            uint32_t uls = (w0 >> 12) & 0xFFF, ult = w0 & 0xFFF;
            const uint8_t *src = R.timg_addr + ((ult * R.timg_width + uls) * bpp) / 8;
            uint32_t row_words = dxt ? (2048 + dxt - 1) / dxt : 0;

            record_load(t->tmem, (uint16_t)words, src, row_words * 8, R.timg_siz, 0, dxt == 0);
            sModeDirty = 1;
            break;
        }

        case G_LOADTILE:
        {
            const GbiTile *t = &R.tiles[(w1 >> 24) & 7];
            uint32_t uls = ((w0 >> 12) & 0xFFF) >> 2, ult = (w0 & 0xFFF) >> 2;
            uint32_t lrt = (w1 & 0xFFF) >> 2;
            uint32_t bpp = (4u << R.timg_siz);
            uint32_t pitch = (R.timg_width * bpp) / 8;
            const uint8_t *src = R.timg_addr + ult * pitch + (uls * bpp) / 8;
            uint32_t rows = lrt - ult + 1;

            record_load(t->tmem, (uint16_t)(t->line * rows), src, pitch, R.timg_siz, 0, 0);
            sModeDirty = 1;
            break;
        }

        case G_LOADTLUT:
        {
            const GbiTile *t = &R.tiles[(w1 >> 24) & 7];
            uint32_t count = ((w1 >> 14) & 0x3FF) + 1;

            record_load(t->tmem, (uint16_t)count, R.timg_addr, 0, G_IM_SIZ_16b, 1, 0);
            sModeDirty = 1;
            break;
        }

        case G_FILLRECT:
            fill_rect((w1 >> 12) & 0xFFF, w1 & 0xFFF, (w0 >> 12) & 0xFFF, w0 & 0xFFF);
            break;

        case G_TEXRECT:
        case G_TEXRECTFLIP:
        {
            /* F3DEX2: the texture coordinates follow as RDPHALF_1/2. */
            uint32_t h1 = dl[0].words.w1;
            uint32_t h2 = dl[1].words.w1;

            dl += 2;
            tex_rect(w0, w1, h1, h2, op == G_TEXRECTFLIP);
            sModeDirty = 1;
            break;
        }

        default:
            gPS2RenderStats.unknown_cmds++;
            break;
        }
    }
    batch_close();
    diag_finish_frame();
}

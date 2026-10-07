/*
 * Asset manager: serves N64 ROM reads from SSB64.DAT.
 *
 * The game keeps addressing data by (virtual) ROM address; this module maps
 * each read onto a converted region of the pack. Only the region index and
 * the RESIDENT metadata live in EE RAM; bulk data is read straight into the
 * caller's buffer (which on the game side is the scene arena), so nothing is
 * loaded that the current scene did not ask for.
 */
#include <ps2/platform.h>
#include <ps2/assetpack.h>

#include <kernel.h>
#include <stdio.h>
#include <string.h>

#define PACK_NAME "SSB64.DAT"

static int sFd = -1;
static char sPackPath[300];
static int sLastSeekRc;
static int sLastReadGot;
static int sStreamReopens;
static PS2PackHeader sHeader;
static PS2PackRegion *sRegions;
static uint8_t **sResidentPtr; /* per region: EE copy or NULL */
static uint8_t *sResidentBlob;
static int sReadSema = -1;

static uint32_t sBytesRead;
static uint32_t sReads;
static uint32_t sMisses;

static int reopen_pack_stream(void)
{
    int new_fd;

    if (sPackPath[0] == '\0')
        return 0;

    /*
     * Reopen through the same public filesystem path. This is intentionally
     * backend-agnostic: a removable/network/MMCE transport that loses one
     * descriptor can recover without changing drivers or rebooting the IOP.
     */
    new_fd = ps2_file_open_read(sPackPath);
    if (new_fd < 0)
        return 0;

    if (sFd >= 0)
        ps2_file_close(sFd);
    sFd = new_fd;
    sStreamReopens++;
    ps2_log("assets: reopened persistent pack stream (%d)", sStreamReopens);
    return 1;
}

static int read_exact(void *dst, uint32_t offset, uint32_t size)
{
    extern void ps2_delay_vblanks(int n);
    int attempt;

    sLastSeekRc = 0;
    sLastReadGot = 0;

    /*
     * Every attempt starts from the absolute pack offset. A failed descriptor
     * is reopened through the same device/path before the next attempt; no
     * backend swap, IOP reset, or MMCE-specific descriptor handoff occurs.
     */
    for (attempt = 0; attempt < 4; attempt++)
    {
        sLastSeekRc = ps2_file_seek(sFd, offset);
        if (sLastSeekRc < 0)
        {
            ps2_log("assets: seek failed off=0x%08x rc=%d attempt=%d",
                    (unsigned)offset, sLastSeekRc, attempt + 1);
        }
        else
        {
            sLastReadGot = ps2_file_read(sFd, dst, size);
            if (sLastReadGot == (int)size)
                return 1;

            ps2_log("assets: short/read fail off=0x%08x want=%u got=%d attempt=%d",
                    (unsigned)offset, (unsigned)size, sLastReadGot, attempt + 1);
        }

        if (attempt + 1 < 4)
        {
            ps2_delay_vblanks(2);
            reopen_pack_stream();
        }
    }

    return 0;
}

int ps2_assets_init(void)
{
    char path[300];
    uint32_t i, table_bytes;
    ee_sema_t sema = { 0 };

    sema.init_count = 1;
    sema.max_count = 1;
    sReadSema = CreateSema(&sema);

    /*
     * Wait for the real pack by opening the handle we will keep for the
     * entire run. Do not probe-open/close it in boot.c and then reopen it:
     * removable/network/MMCE transports can still be settling, and MMCE in
     * particular should not be forced through redundant file-open traffic.
     */
    {
        extern void ps2_delay_vblanks(int n);
        int attempt;

        sFd = -1;
        for (attempt = 0; attempt < 200; attempt++)
        {
            ps2_storage_resolve_data_root(PACK_NAME);
            ps2_storage_path(path, sizeof(path), PACK_NAME);
            snprintf(sPackPath, sizeof(sPackPath), "%s", path);
            sFd = ps2_file_open_read(sPackPath);

            /* PCSX2 Run ELF can lose argv[0]'s directory separators. */
            if (sFd < 0 && ps2_storage_boot_device() == PS2_BOOT_HOST)
            {
                snprintf(path, sizeof(path), "host:%s", PACK_NAME);
                snprintf(sPackPath, sizeof(sPackPath), "%s", path);
                sFd = ps2_file_open_read(sPackPath);
            }

            if (sFd >= 0)
            {
                ps2_log("assets: persistent %s opened after %d ms", path, attempt * 100);
                break;
            }

            if (ps2_storage_data_device() == PS2_BOOT_HOST)
                break;

            /* Generic massN: brings up USB first. If ~5 s pass without the
             * pack on any mass slot, add the other local BDM transports to
             * the same live IOP and keep probing. */
            if (attempt == 50 && ps2_storage_data_device() == PS2_BOOT_BDM)
            {
                ps2_boot_stage("assets: SSB64.DAT not on USB after 5 s; adding other BDM transports", 0);
                ps2_iop_load_bdm_fallback_transports();
            }
            ps2_delay_vblanks(6);
        }
    }
    if (sFd < 0)
    {
        ps2_log("assets: cannot open %s", path);
        return 0;
    }
    if (!read_exact(&sHeader, 0, sizeof(sHeader)) || memcmp(sHeader.magic, PS2PACK_MAGIC, 8) != 0 ||
        sHeader.version != PS2PACK_VERSION)
    {
        ps2_log("assets: %s is not a v%d SSB64 pack", path, PS2PACK_VERSION);
        return 0;
    }

    table_bytes = sHeader.region_count * sizeof(PS2PackRegion);
    sRegions = ps2_mem_alloc(PS2_MEM_GAME_HEAP, table_bytes, 64);
    sResidentPtr = ps2_mem_alloc(PS2_MEM_GAME_HEAP, sHeader.region_count * sizeof(uint8_t *), 64);
    if (!read_exact(sRegions, sHeader.region_table_offset, table_bytes))
    {
        ps2_log("assets: region table read failed");
        return 0;
    }

    /* All resident regions are stored back to back: one read at boot. */
    sResidentBlob = ps2_mem_alloc(PS2_MEM_GAME_HEAP, sHeader.resident_bytes + 64, 64);
    if (sHeader.resident_bytes != 0 &&
        !read_exact(sResidentBlob, sHeader.resident_offset, sHeader.resident_bytes))
    {
        ps2_log("assets: resident block read failed");
        return 0;
    }
    for (i = 0; i < sHeader.region_count; i++)
    {
        sResidentPtr[i] = (sRegions[i].flags & PS2PACK_REGION_RESIDENT)
                              ? sResidentBlob + (sRegions[i].file_offset - sHeader.resident_offset)
                              : NULL;
    }

    ps2_log("assets: %s: %u regions, %u KiB resident, %u KiB total", path, (unsigned)sHeader.region_count,
            (unsigned)(sHeader.resident_bytes >> 10), (unsigned)(sHeader.total_size >> 10));
    return 1;
}

static int find_region(uint32_t addr)
{
    int lo = 0, hi = (int)sHeader.region_count - 1;

    while (lo <= hi)
    {
        int mid = (lo + hi) >> 1;
        const PS2PackRegion *r = &sRegions[mid];

        if (addr < r->vrom_start)
            hi = mid - 1;
        else if (addr >= r->vrom_start + r->size)
            lo = mid + 1;
        else
            return mid;
    }
    return -1;
}

void ps2_rom_read(uint32_t rom_addr, void *dst, uint32_t size)
{
    uint8_t *out = (uint8_t *)dst;

    sReads++;
    /* PI addresses of the cartridge domain (KSEG1 0xB0000000 + offset). */
    if ((rom_addr & 0xF0000000u) == 0xB0000000u)
    {
        rom_addr &= 0x0FFFFFFFu;
    }
    while (size > 0)
    {
        int idx = (sRegions != NULL) ? find_region(rom_addr) : -1;
        const PS2PackRegion *r;
        uint32_t off, n;

        if (idx < 0)
        {
            /* Unmapped ROM: only harmless reads (boot code, unused tables)
             * should end up here. */
            sMisses++;
            if (sMisses < 32)
            {
                ps2_log("assets: unmapped ROM read 0x%08x (+%u)", (unsigned)rom_addr, (unsigned)size);
            }
            memset(out, 0, size);
            return;
        }
        r = &sRegions[idx];
        off = rom_addr - r->vrom_start;
        n = r->size - off;
        if (n > size)
        {
            n = size;
        }
        if (sResidentPtr[idx] != NULL)
        {
            memcpy(out, sResidentPtr[idx] + off, n);
        }
        else
        {
            WaitSema(sReadSema);
            if (!read_exact(out, r->file_offset + off, n))
            {
                SignalSema(sReadSema);
                ps2_panic("asset read failed vrom=0x%08x file=0x%08x size=%u got=%d seek=%d reopens=%d",
                          (unsigned)rom_addr, (unsigned)(r->file_offset + off), (unsigned)n,
                          sLastReadGot, sLastSeekRc, sStreamReopens);
            }
            SignalSema(sReadSema);
            sBytesRead += n;
        }
        out += n;
        rom_addr += n;
        size -= n;
    }
}

uint32_t ps2_assets_bytes_read(void)
{
    return sBytesRead;
}

uint32_t ps2_assets_read_count(void)
{
    return sReads;
}

uint32_t ps2_assets_reloc_vrom(void)
{
    return sHeader.reloc_vrom;
}

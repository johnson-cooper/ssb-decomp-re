/*
 * Boot/data device detection and path normalisation.
 *
 * argv[0] tells us where the ELF was launched from.  That is not necessarily
 * where the 25 MiB SSB64.DAT lives: an optional --data=<directory> argument
 * can point at any supported filesystem.  Keeping launch and data devices
 * separate also lets us preserve host: (ps2link/PCSX2) while loading a real
 * storage stack for assets.
 *
 * Canonical examples:
 *   usb:/SSB64/ssb64.elf              -> mass0:/SSB64/
 *   mass1:/SSB64/ssb64.elf            -> massN:/SSB64/ (slot found by probe)
 *   mx4sio:/SSB64/ssb64.elf           -> mx4sio:/SSB64/
 *   hdd0:+OPL:pfs:/SSB64/ssb64.elf    -> pfs0:/SSB64/
 *                                         mount source hdd0:+OPL
 *   udpfs:/SSB64/ssb64.elf            -> udpfs:/SSB64/
 *
 * massN: is resolved after the IOP reset by probing every massN: slot for
 * the same relative path (USB first, then the other local BDM transports).
 *
 * A bare bdm: path is deliberately NOT guessed: it does not identify which
 * transport must be reconstructed after an IOP reset.
 */
#include <ps2/platform.h>

#define NEWLIB_PORT_AWARE
#include <fileXio_rpc.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>
#include <usbhdfsd-common.h>

#define PATH_BUF_MAX 256
#define HDD_SOURCE_MAX 128

static PS2BootDevice sLaunchDevice = PS2_BOOT_UNKNOWN;
static PS2BootDevice sDataDevice = PS2_BOOT_UNKNOWN;
static char sLaunchPath[PATH_BUF_MAX] = "";
static char sDataDir[PATH_BUF_MAX] = "host:";
static char sHddMountSource[HDD_SOURCE_MAX] = "";
static int sProgressive;
static int sDataNeedsExistingIop;
static int sDataNeedsBdmResolve;

static int starts_with_ci(const char *s, const char *prefix)
{
    return s != NULL && strncasecmp(s, prefix, strlen(prefix)) == 0;
}

static PS2BootDevice detect_device(const char *path)
{
    if (path == NULL || path[0] == '\0')
        return PS2_BOOT_UNKNOWN;
    if (starts_with_ci(path, "host"))
        return PS2_BOOT_HOST;
    if (starts_with_ci(path, "mass"))
        return PS2_BOOT_BDM;
    if (starts_with_ci(path, "usb"))
        return PS2_BOOT_USB;
    if (starts_with_ci(path, "mc"))
        return PS2_BOOT_MC;
    if (starts_with_ci(path, "ata"))
        return PS2_BOOT_ATA;
    if (starts_with_ci(path, "mx4sio") || starts_with_ci(path, "mx4:") ||
        starts_with_ci(path, "mx4/"))
        return PS2_BOOT_MX4SIO;
    if (starts_with_ci(path, "ilink"))
        return PS2_BOOT_ILINK;
    if (starts_with_ci(path, "udpbd"))
        return PS2_BOOT_UDPBD;
    if (starts_with_ci(path, "udpfs"))
        return PS2_BOOT_UDPFS;
    if (starts_with_ci(path, "mmce"))
        return PS2_BOOT_MMCE;
    if (starts_with_ci(path, "cdrom"))
        return PS2_BOOT_CDROM;
    if (starts_with_ci(path, "hdd") || starts_with_ci(path, "pfs") ||
        path[0] == '+' || starts_with_ci(path, "__") || strstr(path, ":pfs") != NULL)
        return PS2_BOOT_HDD;

    /* "bdm:" alone is intentionally ambiguous (USB/MX4SIO/iLink/ATA/UDPBD). */
    return PS2_BOOT_UNKNOWN;
}

static void strip_filename(char *path)
{
    const char *last = NULL;
    const char *p;
    size_t len;

    for (p = path; *p != '\0'; p++)
    {
        if (*p == '/' || *p == '\\' || *p == ':')
            last = p;
    }
    if (last == NULL)
    {
        path[0] = '\0';
        return;
    }
    len = (size_t)(last - path) + 1;
    path[len] = '\0';
}

static void ensure_directory_suffix(char *path, size_t size, PS2BootDevice dev)
{
    size_t len = strlen(path);
    char sep = (dev == PS2_BOOT_CDROM) ? '\\' : '/';

    if (len == 0 || path[len - 1] == ':' || path[len - 1] == '/' || path[len - 1] == '\\')
        return;
    if (len + 1 < size)
    {
        path[len] = sep;
        path[len + 1] = '\0';
    }
}

static int is_pfs_token(const char *p)
{
    const char *q;

    if (!starts_with_ci(p, "pfs"))
        return 0;
    q = p + 3;
    while (*q >= '0' && *q <= '9')
        q++;
    return *q == ':' || *q == '/' || *q == '\\' || *q == '\0';
}

static const char *find_pfs_token(const char *s)
{
    const char *p;

    for (p = s; p[0] != '\0' && p[1] != '\0' && p[2] != '\0'; p++)
    {
        if ((p[0] == 'p' || p[0] == 'P') &&
            (p[1] == 'f' || p[1] == 'F') &&
            (p[2] == 's' || p[2] == 'S') &&
            is_pfs_token(p))
            return p;
    }
    return NULL;
}

/* Resolve a canonical HDD/PFS runtime path and remember which APA partition
 * must be mounted on pfs0:.  A bare pfs0: path can be used only when the
 * caller preserved an already-mounted IOP; after our normal reset there is
 * no way to reconstruct the source partition, so iop.c rejects it. */
static int normalise_hdd_path(const char *path, char *out, size_t out_size, int path_is_file)
{
    char hdd_prefix[8] = "hdd0:";
    const char *p = path;
    const char *part_start;
    const char *part_end = NULL;
    const char *sub;
    const char *colon;
    const char *slash;
    const char *bslash;
    const char *pfs;
    size_t part_len;

    sHddMountSource[0] = '\0';

    if (starts_with_ci(p, "pfs"))
    {
        /* A bare pfsN: path contains no APA partition identity. It is only
         * usable while preserving the launcher's already-mounted IOP state. */
        sDataNeedsExistingIop = 1;
        strncpy(out, p, out_size - 1);
        out[out_size - 1] = '\0';
        if (path_is_file)
            strip_filename(out);
        else
            ensure_directory_suffix(out, out_size, PS2_BOOT_HDD);
        return 1;
    }

    if (starts_with_ci(p, "hdd"))
    {
        colon = strchr(p, ':');
        if (colon == NULL || (size_t)(colon - p + 1) >= sizeof(hdd_prefix))
            return 0;
        memcpy(hdd_prefix, p, (size_t)(colon - p + 1));
        hdd_prefix[colon - p + 1] = '\0';
        p = colon + 1;
    }

    while (*p == '/' || *p == '\\')
        p++;
    part_start = p;

    colon = strchr(part_start, ':');
    slash = strchr(part_start, '/');
    bslash = strchr(part_start, '\\');
    if (bslash != NULL && (slash == NULL || bslash < slash))
        slash = bslash;

    pfs = find_pfs_token(part_start);
    if (pfs == part_start)
        pfs = NULL;

    if (colon != NULL)
        part_end = colon;
    if (slash != NULL && (part_end == NULL || slash < part_end))
        part_end = slash;
    if (pfs != NULL && (part_end == NULL || pfs < part_end))
        part_end = pfs;
    if (part_end == NULL)
        part_end = part_start + strlen(part_start);

    part_len = (size_t)(part_end - part_start);
    if (part_len == 0 || strlen(hdd_prefix) + part_len >= sizeof(sHddMountSource))
        return 0;

    snprintf(sHddMountSource, sizeof(sHddMountSource), "%s%.*s",
             hdd_prefix, (int)part_len, part_start);

    sub = part_end;
    if (*sub == ':')
        sub++;
    if (starts_with_ci(sub, "pfs"))
    {
        sub += 3;
        while (*sub >= '0' && *sub <= '9')
            sub++;
        if (*sub == ':')
            sub++;
    }

    if (*sub == '\0')
        snprintf(out, out_size, "pfs0:/");
    else if (*sub == '/' || *sub == '\\')
        snprintf(out, out_size, "pfs0:%s", sub);
    else
        snprintf(out, out_size, "pfs0:/%s", sub);

    /* PFS accepts forward slashes consistently. */
    {
        char *q;
        for (q = out; *q != '\0'; q++)
            if (*q == '\\')
                *q = '/';
    }

    if (path_is_file)
        strip_filename(out);
    else
        ensure_directory_suffix(out, out_size, PS2_BOOT_HDD);
    return 1;
}

static int set_data_location(const char *path, int path_is_file)
{
    PS2BootDevice dev = detect_device(path);
    char tmp[PATH_BUF_MAX];

    if (dev == PS2_BOOT_UNKNOWN)
        return 0;

    if (dev == PS2_BOOT_HDD)
    {
        sDataNeedsExistingIop = 0;
        sDataNeedsBdmResolve = 0;
        if (!normalise_hdd_path(path, tmp, sizeof(tmp), path_is_file))
            return 0;
    }
    else if (dev == PS2_BOOT_BDM)
    {
        /* massN: is a connection-order alias, not a live handle we can
         * inherit: whether the launcher's mount survives to this ELF depends
         * on the launcher (RiptOPL's "Reboot IOP" leaves a bare ROM IOP with
         * no mass: device and no fileXio server). Reset once like every other
         * reconstructible device, bring up our own BDM stack, and locate the
         * volume that holds the same relative path. Never reset again after
         * that stack is up: hardware showed USB does not come back from a
         * second reset in the same boot. */
        sDataNeedsExistingIop = 0;
        sDataNeedsBdmResolve = 1;
        strncpy(tmp, path, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        if (path_is_file)
            strip_filename(tmp);
        else
            ensure_directory_suffix(tmp, sizeof(tmp), dev);
        sHddMountSource[0] = '\0';
    }
    else if (dev == PS2_BOOT_USB)
    {
        sDataNeedsExistingIop = 0;
        /* usbN: is a typed transport identity on older BDM stacks. Resolve
         * it to the matching massN: filesystem after the USB driver loads. */
        strncpy(tmp, path, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        sDataNeedsBdmResolve = 1;
        if (path_is_file)
            strip_filename(tmp);
        else
            ensure_directory_suffix(tmp, sizeof(tmp), dev);
        sHddMountSource[0] = '\0';
    }
    else
    {
        sDataNeedsExistingIop = (dev == PS2_BOOT_HOST);
        sDataNeedsBdmResolve = (dev == PS2_BOOT_ATA || dev == PS2_BOOT_MX4SIO ||
                                dev == PS2_BOOT_ILINK || dev == PS2_BOOT_UDPBD);
        strncpy(tmp, path, sizeof(tmp) - 1);
        tmp[sizeof(tmp) - 1] = '\0';
        if (path_is_file)
            strip_filename(tmp);
        else
            ensure_directory_suffix(tmp, sizeof(tmp), dev);
        sHddMountSource[0] = '\0';
    }

    sDataDevice = dev;
    strncpy(sDataDir, tmp, sizeof(sDataDir) - 1);
    sDataDir[sizeof(sDataDir) - 1] = '\0';
    return 1;
}

void ps2_storage_set_boot_path(const char *argv0)
{
    const char *p;

    sProgressive = 0;
    sHddMountSource[0] = '\0';
    sDataNeedsExistingIop = 0;
    sDataNeedsBdmResolve = 0;

    if (argv0 == NULL || argv0[0] == '\0')
    {
        sLaunchDevice = PS2_BOOT_HOST;
        sDataDevice = PS2_BOOT_HOST;
        strcpy(sLaunchPath, "host:");
        strcpy(sDataDir, "host:");
        return;
    }

    strncpy(sLaunchPath, argv0, sizeof(sLaunchPath) - 1);
    sLaunchPath[sizeof(sLaunchPath) - 1] = '\0';

    for (p = argv0; p[0] != '\0'; p++)
    {
        if (p[1] != '\0' && p[2] != '\0' && p[3] != '\0' &&
            p[0] == '2' && p[1] == '4' && p[2] == '0' && (p[3] == 'p' || p[3] == 'P'))
            sProgressive = 1;
    }

    sLaunchDevice = detect_device(argv0);
    if (!set_data_location(argv0, 1))
    {
        sDataDevice = PS2_BOOT_UNKNOWN;
        sDataDir[0] = '\0';
    }
}

int ps2_storage_set_data_path(const char *path)
{
    if (path == NULL || path[0] == '\0')
        return 0;
    return set_data_location(path, 0);
}

PS2BootDevice ps2_storage_launch_device(void)
{
    return sLaunchDevice;
}

PS2BootDevice ps2_storage_data_device(void)
{
    return sDataDevice;
}

PS2BootDevice ps2_storage_boot_device(void)
{
    return sDataDevice;
}

const char *ps2_storage_launch_path(void)
{
    return sLaunchPath;
}

const char *ps2_storage_hdd_mount_source(void)
{
    return sHddMountSource;
}

int ps2_storage_requires_iop_preserve(void)
{
    return sDataNeedsExistingIop;
}

static int bdm_driver_matches(PS2BootDevice dev, const char *driver)
{
    if (driver == NULL || driver[0] == '\0')
        return 0;

    switch (dev)
    {
    case PS2_BOOT_USB:
        return strcmp(driver, "usb") == 0;
    case PS2_BOOT_ATA:
        return strcmp(driver, "ata") == 0;
    case PS2_BOOT_MX4SIO:
        return strcmp(driver, "sdc") == 0 || strcmp(driver, "mx4sio") == 0;
    case PS2_BOOT_ILINK:
        return strcmp(driver, "sd") == 0 || strcmp(driver, "ilink") == 0;
    case PS2_BOOT_UDPBD:
        return strcmp(driver, "udp") == 0;
    default:
        return 0;
    }
}

int ps2_storage_resolve_data_root(const char *probe_name)
{
    char relative[PATH_BUF_MAX];
    const char *colon;
    int slot;

    if (!sDataNeedsBdmResolve)
        return 1;
    if (probe_name == NULL || probe_name[0] == '\0')
        return 0;

    colon = strchr(sDataDir, ':');
    if (colon == NULL)
        return 0;
    snprintf(relative, sizeof(relative), "%s", colon + 1);
    if (relative[0] == '\0')
        snprintf(relative, sizeof(relative), "/");

    for (slot = 0; slot < 10; slot++)
    {
        char root[16];
        char dir[PATH_BUF_MAX];
        char probe[PATH_BUF_MAX + 64];
        char driver[32];
        int fd, dfd, io;

        snprintf(root, sizeof(root), "mass%d:/", slot);
        if (relative[0] == '/' || relative[0] == '\\')
            snprintf(dir, sizeof(dir), "mass%d:%s", slot, relative);
        else
            snprintf(dir, sizeof(dir), "mass%d:/%s", slot, relative);
        ensure_directory_suffix(dir, sizeof(dir), sDataDevice);
        snprintf(probe, sizeof(probe), "%s%s", dir, probe_name);

        /* Opening the actual pack first both lazy-mounts FatFs and proves this
         * volume is live. Only then ask bdmfs for the driver token: querying
         * an empty mass slot with a return buffer can fault older bdmfs. */
        fd = open(probe, O_RDONLY);
        if (fd < 0)
            continue;
        close(fd);

        memset(driver, 0, sizeof(driver));
        io = -1;
        dfd = fileXioDopen(root);
        if (dfd >= 0)
        {
            io = fileXioIoctl2(dfd, USBMASS_IOCTL_GET_DRIVERNAME,
                               NULL, 0, driver, sizeof(driver) - 1);
            fileXioDclose(dfd);
        }
        /* Generic massN: does not name a transport; the relative path that
         * just opened already identifies the volume, and the driver token is
         * only logged. Typed paths must match their transport. */
        if (sDataDevice != PS2_BOOT_BDM && (io < 0 || !bdm_driver_matches(sDataDevice, driver)))
            continue;

        snprintf(sDataDir, sizeof(sDataDir), "%s", dir);
        sDataNeedsBdmResolve = 0;
        ps2_log("storage: %s resolved to %s (driver=%s)",
                ps2_storage_device_name(sDataDevice), sDataDir, driver);
        return 1;
    }

    return 0;
}


int ps2_video_progressive(void)
{
    return sProgressive;
}

const char *ps2_storage_boot_dir(void)
{
    return sDataDir;
}

const char *ps2_storage_device_name(PS2BootDevice dev)
{
    switch (dev)
    {
    case PS2_BOOT_HOST: return "host";
    case PS2_BOOT_BDM: return "bdm/mass";
    case PS2_BOOT_USB: return "usb";
    case PS2_BOOT_MC: return "memory card";
    case PS2_BOOT_ATA: return "ata-bdm";
    case PS2_BOOT_MX4SIO: return "mx4sio";
    case PS2_BOOT_ILINK: return "ilink";
    case PS2_BOOT_UDPBD: return "udpbd";
    case PS2_BOOT_UDPFS: return "udpfs";
    case PS2_BOOT_HDD: return "apa/pfs";
    case PS2_BOOT_MMCE: return "mmce";
    case PS2_BOOT_CDROM: return "cdrom";
    default: return "unknown";
    }
}

void ps2_storage_path(char *out, size_t out_size, const char *name)
{
    snprintf(out, out_size, "%s%s", sDataDir, name);
}

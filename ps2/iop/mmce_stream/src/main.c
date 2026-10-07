#include <errno.h>
#include <iomanX.h>
#include <irx.h>
#include <loadcore.h>
#include <stdio.h>
#include <thbase.h>
#include <types.h>

IRX_ID("ssbmmce", 1, 1);

typedef struct {
    int version;
    void **exports;
} modinfo_t;

static s64 (*mmcedrv_get_size_fn)(int fd);
static void (*mmcedrv_config_set_fn)(int setting, int value);
static int (*mmcedrv_read_fn)(int fd, int size, void *ptr);
static int (*mmcedrv_lseek_fn)(int fd, int offset, int whence);

enum {
    MMCEDRV_SETTING_PORT = 0,
    MMCEDRV_SETTING_ACK_WAIT_CYCLES = 1,
    MMCEDRV_SETTING_USE_ALARMS = 2,
};

static int get_mod_info(const char *name, modinfo_t *info)
{
    iop_library_t *lib = GetLoadcoreInternalData()->let_next;
    int i;

    while (lib != NULL) {
        for (i = 0; i < 8; i++) {
            if (lib->name[i] != name[i])
                break;
        }
        if (i == 8) {
            info->version = lib->version;
            info->exports = (void **)(((struct irx_export_table *)lib)->fptrs);
            return 1;
        }
        lib = lib->prev;
    }
    return 0;
}

static int parse_digit(const char **p)
{
    int v = 0, any = 0;
    while (**p >= '0' && **p <= '9') {
        v = v * 10 + (**p - '0');
        (*p)++;
        any = 1;
    }
    return any ? v : -1;
}

static int stream_init(iomanX_iop_device_t *d)
{
    modinfo_t info;
    (void)d;

    if (!get_mod_info("mmcedrv\0", &info))
        return -ENODEV;

    /* mmcedrv exports.tab: get_size=4, config_set=6, read=7, lseek=9. */
    mmcedrv_get_size_fn = (void *)info.exports[4];
    mmcedrv_config_set_fn = (void *)info.exports[6];
    mmcedrv_read_fn = (void *)info.exports[7];
    mmcedrv_lseek_fn = (void *)info.exports[9];

    if (!mmcedrv_get_size_fn || !mmcedrv_config_set_fn ||
        !mmcedrv_read_fn || !mmcedrv_lseek_fn)
        return -ENODEV;

    return 0;
}

static int stream_deinit(iomanX_iop_device_t *d) { (void)d; return 0; }
static int stream_format(iomanX_iop_file_t *f, const char *a, const char *b, void *c, int d)
{ (void)f; (void)a; (void)b; (void)c; (void)d; return -EIO; }

static int stream_open(iomanX_iop_file_t *f, const char *name, int flags, int mode)
{
    const char *p = name;
    int port, fd, tries;
    s64 size = -1;

    (void)flags;
    (void)mode;

    port = parse_digit(&p);
    if (port < 2 || port > 3 || *p != ',')
        return -EINVAL;
    p++;
    fd = parse_digit(&p);
    if (fd < 0)
        return -EINVAL;

    mmcedrv_config_set_fn(MMCEDRV_SETTING_PORT, port);
    mmcedrv_config_set_fn(MMCEDRV_SETTING_ACK_WAIT_CYCLES, 5);
    mmcedrv_config_set_fn(MMCEDRV_SETTING_USE_ALARMS, 1);

    /*
     * The card-side descriptor survives the deliberate IOP reset, but the
     * MMCE may need a short settle before the new SIO2/MMCEDRV stack can use
     * it. Validate it here so a bad handoff fails before gameplay.
     */
    for (tries = 0; tries < 100; tries++) {
        size = mmcedrv_get_size_fn(fd);
        if (size > 0)
            break;
        DelayThread(100 * 1000);
    }
    if (size <= 0) {
        printf("ssbmmce: fd %d never became ready on port %d\n", fd, port);
        return -EIO;
    }

    f->privdata = (void *)(u32)fd;
    printf("ssbmmce: fd %d ready on port %d, size=%lld\n", fd, port, size);
    return 0;
}

static int stream_close(iomanX_iop_file_t *f) { (void)f; return 0; }
static int stream_read(iomanX_iop_file_t *f, void *buf, int size)
{ return mmcedrv_read_fn((int)(u32)f->privdata, size, buf); }
static int stream_write(iomanX_iop_file_t *f, void *buf, int size)
{ (void)f; (void)buf; (void)size; return -EROFS; }
static int stream_lseek(iomanX_iop_file_t *f, int offset, int whence)
{ return mmcedrv_lseek_fn((int)(u32)f->privdata, offset, whence); }
static int stream_ioctl(iomanX_iop_file_t *f, int cmd, void *arg)
{ (void)f; (void)cmd; (void)arg; return -EIO; }

static int no_path(iomanX_iop_file_t *f, const char *path)
{ (void)f; (void)path; return -EIO; }
static int no_mkdir(iomanX_iop_file_t *f, const char *path, int mode)
{ (void)f; (void)path; (void)mode; return -EIO; }
static int no_dopen(iomanX_iop_file_t *f, const char *path)
{ (void)f; (void)path; return -EIO; }
static int no_dclose(iomanX_iop_file_t *f) { (void)f; return -EIO; }
static int no_dread(iomanX_iop_file_t *f, iox_dirent_t *dirent)
{ (void)f; (void)dirent; return -EIO; }
static int no_getstat(iomanX_iop_file_t *f, const char *path, iox_stat_t *stat)
{ (void)f; (void)path; (void)stat; return -EIO; }
static int no_chstat(iomanX_iop_file_t *f, const char *path, iox_stat_t *stat, unsigned int mask)
{ (void)f; (void)path; (void)stat; (void)mask; return -EIO; }
static int no_rename(iomanX_iop_file_t *f, const char *a, const char *b)
{ (void)f; (void)a; (void)b; return -EIO; }
static int no_sync(iomanX_iop_file_t *f, const char *dev, int flag)
{ (void)f; (void)dev; (void)flag; return -EIO; }
static int no_mount(iomanX_iop_file_t *f, const char *a, const char *b, int flag, void *arg, int arglen)
{ (void)f; (void)a; (void)b; (void)flag; (void)arg; (void)arglen; return -EIO; }
static int no_umount(iomanX_iop_file_t *f, const char *path)
{ (void)f; (void)path; return -EIO; }
static s64 stream_lseek64(iomanX_iop_file_t *f, s64 offset, int whence)
{
    if (offset > 0x7fffffffLL || offset < -0x7fffffffLL)
        return -EINVAL;
    return (s64)stream_lseek(f, (int)offset, whence);
}
static int no_devctl(iomanX_iop_file_t *f, const char *name, int cmd, void *arg, unsigned int arglen, void *buf, unsigned int buflen)
{ (void)f; (void)name; (void)cmd; (void)arg; (void)arglen; (void)buf; (void)buflen; return -EIO; }
static int no_symlink(iomanX_iop_file_t *f, const char *a, const char *b)
{ (void)f; (void)a; (void)b; return -EIO; }
static int no_readlink(iomanX_iop_file_t *f, const char *path, char *buf, unsigned int buflen)
{ (void)f; (void)path; (void)buf; (void)buflen; return -EIO; }
static int no_ioctl2(iomanX_iop_file_t *f, int cmd, void *data, unsigned int datalen, void *rdata, unsigned int rdatalen)
{ (void)f; (void)cmd; (void)data; (void)datalen; (void)rdata; (void)rdatalen; return -EIO; }

static iomanX_iop_device_ops_t ops = {
    stream_init, stream_deinit, stream_format, stream_open, stream_close,
    stream_read, stream_write, stream_lseek, stream_ioctl, no_path, no_mkdir,
    no_path, no_dopen, no_dclose, no_dread, no_getstat, no_chstat, no_rename,
    no_path, no_sync, no_mount, no_umount, stream_lseek64, no_devctl,
    no_symlink, no_readlink, no_ioctl2
};

static const char devname[] = "ssbmmce";
static iomanX_iop_device_t dev = {
    devname, IOP_DT_FSEXT | IOP_DT_FS, 1, devname, &ops
};

int _start(int argc, char *argv[])
{
    (void)argc; (void)argv;
    iomanX_DelDrv(devname);
    if (iomanX_AddDrv(&dev) != 0)
        return MODULE_NO_RESIDENT_END;
    printf("ssbmmce: in-game stream bridge ready\n");
    return MODULE_RESIDENT_END;
}

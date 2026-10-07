#include <loadcore.h>
#include <irx.h>

#include "main.h"
#include "xfer.h"

IRX_ID(MODNAME, 0x2, 0x1A);

extern struct irx_export_table _exp_smap __attribute__((section("data")));

int _start(int argc, char *argv[])
{
    int result;

    /* Claim the export namespace before smap_init installs IRQ/thread state.
     * A duplicate load must have no hardware side effects. */
    if (RegisterLibraryEntries(&_exp_smap) != 0) {
        M_DEBUG("smap: module already loaded\n");
        return MODULE_NO_RESIDENT_END;
    }

    result = smap_init(argc, argv);
    if (result < 0) {
        M_DEBUG("smap: smap_init -> %d\n", result);
        /* smap_init cleans up any thread/event setup that fails internally;
         * release the namespace so a later load can retry. */
        ReleaseLibraryEntries(&_exp_smap);
        return MODULE_NO_RESIDENT_END;
    }

    return MODULE_RESIDENT_END;
}

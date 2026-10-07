/*
 * Stops a USB host controller left running by the launcher, so the EE can
 * reset the IOP safely.
 *
 * The OHCI is a bus master that keeps walking its endpoint lists and writing
 * its HCCA in IOP RAM on its own. PS2SDK's usbd does not halt it on IOP
 * reboot (its reboot hook only pulses port reset), so after a reset it keeps
 * DMAing into memory the new IOP kernel and modules now occupy. Whether that
 * lands on something important depends on what the launcher had loaded,
 * which is why the reset hung from some launchers and not others.
 *
 * Loaded into the inherited IOP immediately before SifIopReset(); it does its
 * work in _start and never stays resident.
 */
#include <irx.h>
#include <intrman.h>
#include <loadcore.h>
#include <thbase.h>
#include <types.h>

IRX_ID("ssbusbq", 1, 0);

#define IOP_DPCR2            (*(volatile u32 *)0xBF801570)
#define IOP_DPCR2_USB        0x08000000u /* set by usbd before it touches the OHCI */
#define OHCI_BASE            0xBF801600u
#define OHCI_HC_CONTROL      (*(volatile u32 *)(OHCI_BASE + 0x04))
#define OHCI_HC_CMD_STATUS   (*(volatile u32 *)(OHCI_BASE + 0x08))
#define OHCI_HC_INT_DISABLE  (*(volatile u32 *)(OHCI_BASE + 0x14))
#define OHCI_COM_HCR         0x00000001u
#define OHCI_CTR_LISTS       0x0000003Cu /* PLE | IE | CLE | BLE */
#define IOP_IRQ_USB          22

int _start(int argc, char *argv[])
{
    int old;
    int i;

    (void)argc;
    (void)argv;

    /* Nothing ever enabled USB on this IOP: the controller is idle. */
    if ((IOP_DPCR2 & IOP_DPCR2_USB) == 0)
        return MODULE_NO_RESIDENT_END;

    DisableIntr(IOP_IRQ_USB, &old);

    /* Same sequence usbd's own initHardware() uses: stop list processing,
     * let the current frame finish, then software-reset the controller,
     * which leaves it suspended with no DMA. */
    OHCI_HC_INT_DISABLE = ~0u;
    OHCI_HC_CONTROL &= ~OHCI_CTR_LISTS;
    DelayThread(2000);
    OHCI_HC_CMD_STATUS = OHCI_COM_HCR;
    OHCI_HC_CONTROL = 0;
    for (i = 0; i < 1000 && (OHCI_HC_CMD_STATUS & OHCI_COM_HCR) != 0; i++)
        DelayThread(10);

    return MODULE_NO_RESIDENT_END;
}

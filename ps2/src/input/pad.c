/*
 * PS2 controllers -> logical N64 controllers.
 *
 * Up to four players: a multitap on port 1 gives slots 0-3; otherwise
 * players 1/2 are the two controller ports (and a multitap on port 2 extends
 * player 2..5 -> 2..4). The game never sees PS2 buttons; it reads N64
 * button masks and a signed stick through osContGetReadData (ps2/src/ultra/cont.c).
 *
 * The default mapping is one table (sDefaultMap) so it can be tuned or later
 * loaded from a config file without touching anything else.
 */
#include <ps2/platform.h>
#include <ps2/input.h>

#include <delaythread.h>
#include <kernel.h>
#include <libmtap.h>
#include <libpad.h>
#include <sifrpc.h>
#include <string.h>

/* libpad buttons (active-low in padButtonStatus.btns) */
#define PS2B_SELECT   0x0001
#define PS2B_L3       0x0002
#define PS2B_R3       0x0004
#define PS2B_START    0x0008
#define PS2B_UP       0x0010
#define PS2B_RIGHT    0x0020
#define PS2B_DOWN     0x0040
#define PS2B_LEFT     0x0080
#define PS2B_L2       0x0100
#define PS2B_R2       0x0200
#define PS2B_L1       0x0400
#define PS2B_R1       0x0800
#define PS2B_TRIANGLE 0x1000
#define PS2B_CIRCLE   0x2000
#define PS2B_CROSS    0x4000
#define PS2B_SQUARE   0x8000

typedef struct PadMapEntry
{
    uint16_t ps2;
    uint16_t n64;
} PadMapEntry;

/* Default Smash layout on a DualShock 2:
 *   Cross = A (attack)      Square = B (special)
 *   Triangle/Circle = C-up / C-right (jump, like the N64 C buttons)
 *   L1 / R2 = Z (shield)    R1 = R (grab/shield)   L2 = L
 *   Start = Start, D-pad = D-pad, right stick = C buttons.
 */
static const PadMapEntry sDefaultMap[] = {
    { PS2B_CROSS, N64_BTN_A },
    { PS2B_SQUARE, N64_BTN_B },
    { PS2B_TRIANGLE, N64_BTN_CU },
    { PS2B_CIRCLE, N64_BTN_CR },
    { PS2B_L1, N64_BTN_Z },
    { PS2B_R2, N64_BTN_Z },
    { PS2B_R1, N64_BTN_R },
    { PS2B_L2, N64_BTN_L },
    { PS2B_START, N64_BTN_START },
    { PS2B_UP, N64_BTN_DU },
    { PS2B_DOWN, N64_BTN_DD },
    { PS2B_LEFT, N64_BTN_DL },
    { PS2B_RIGHT, N64_BTN_DR },
};

#define STICK_DEADZONE 10     /* of 127 */
#define STICK_N64_MAX 80      /* full deflection of an N64 stick */
#define RSTICK_C_THRESHOLD 64 /* right stick -> C buttons */

typedef struct PadSlot
{
    int port, slot;
    int open;
    int analog_set;
    int has_actuator;
    int rumble_on;
    uint8_t buf[256] __attribute__((aligned(64)));
} PadSlot;

static PadSlot sSlots[PS2_INPUT_MAX_PLAYERS] __attribute__((aligned(64)));
static int sMtap[2];
static PS2InputState sState[PS2_INPUT_MAX_PLAYERS];
static uint16_t sRawHeld[PS2_INPUT_MAX_PLAYERS];
static int sInitDone;

static void assign_slots(void)
{
    int i;

    if (sMtap[0])
    {
        for (i = 0; i < 4; i++)
        {
            sSlots[i].port = 0;
            sSlots[i].slot = i;
        }
    }
    else
    {
        sSlots[0].port = 0;
        sSlots[0].slot = 0;
        for (i = 1; i < 4; i++)
        {
            sSlots[i].port = 1;
            sSlots[i].slot = i - 1;
        }
        if (!sMtap[1])
        {
            sSlots[2].port = sSlots[3].port = -1;
        }
    }
}

static void setup_pad_mode(PadSlot *s);

static int detect_multitap(int port)
{
    int slots;

    if (mtapPortOpen(port) != 1 || mtapGetConnection(port) != 1)
    {
        mtapPortClose(port);
        return 0;
    }

    /* mtapman can successfully probe a plain controller port and still
     * report one available slot. A real multitap exposes multiple slots;
     * require that before remapping players 2-4 onto the port. */
    slots = padGetSlotMax(port);
    if (slots <= 1)
    {
        mtapPortClose(port);
        return 0;
    }
    return 1;
}

void ps2_input_init(void)
{
    int i;

    mtapInit();
    padInit(0);
    sMtap[0] = detect_multitap(0);
    sMtap[1] = detect_multitap(1);
    assign_slots();

    for (i = 0; i < PS2_INPUT_MAX_PLAYERS; i++)
    {
        if (sSlots[i].port < 0)
        {
            continue;
        }
        sSlots[i].open = padPortOpen(sSlots[i].port, sSlots[i].slot, sSlots[i].buf) != 0;
    }
    sInitDone = 1;

    /* The N64 game checks for controllers once at boot ("No Controller"
     * screen otherwise). Pads need a few frames after padPortOpen before
     * they report STABLE, so give them up to ~1.5 s to settle. */
    {
        extern void ps2_delay_vblanks(int n);
        int tries, n = 0;

        for (tries = 0; tries < 90; tries++)
        {
            int all_settled = 1;

            n = 0;
            for (i = 0; i < PS2_INPUT_MAX_PLAYERS; i++)
            {
                int st;

                if (!sSlots[i].open)
                    continue;
                st = padGetState(sSlots[i].port, sSlots[i].slot);
                if (st == PAD_STATE_STABLE && !sSlots[i].analog_set)
                {
                    /* switch to analog now; the pad is busy for a few
                     * frames and must settle again before the game asks */
                    setup_pad_mode(&sSlots[i]);
                    all_settled = 0;
                }
                else if (st == PAD_STATE_STABLE || st == PAD_STATE_FINDCTP1)
                    n++;
                else if (st != PAD_STATE_DISCONN)
                    all_settled = 0;
            }
            if (n > 0 && all_settled)
                break;
            ps2_delay_vblanks(1);
        }
        ps2_input_poll();
        ps2_log("input: multitap port1=%d port2=%d, %d pad(s) ready after %d frames", sMtap[0], sMtap[1], n, tries);
    }
}

static int8_t stick_to_n64(uint8_t raw, int invert)
{
    int v = (int)raw - 128;

    if (invert)
    {
        v = -v;
    }
    if (v > -STICK_DEADZONE && v < STICK_DEADZONE)
    {
        return 0;
    }
    /* Rescale the remaining travel onto the N64's usable range. */
    v = (v > 0) ? (v - STICK_DEADZONE) : (v + STICK_DEADZONE);
    v = (v * STICK_N64_MAX) / (127 - STICK_DEADZONE);
    if (v > STICK_N64_MAX)
        v = STICK_N64_MAX;
    if (v < -STICK_N64_MAX)
        v = -STICK_N64_MAX;
    return (int8_t)v;
}

static void setup_pad_mode(PadSlot *s)
{
    int type = padInfoMode(s->port, s->slot, PAD_MODECURID, 0);

    if (type == PAD_TYPE_DUALSHOCK || type == PAD_TYPE_ANALOG)
    {
        s->analog_set = 1;
    }
    else
    {
        /* Ask for analog (DualShock) mode, locked so the ANALOG button
         * cannot switch it off mid-game. */
        padSetMainMode(s->port, s->slot, PAD_MMODE_DUALSHOCK, PAD_MMODE_LOCK);
        s->analog_set = 1;
    }
    if (padInfoAct(s->port, s->slot, -1, 0) > 0)
    {
        static const char align[6] = { 0, 1, 0xFF, 0xFF, 0xFF, 0xFF };

        padSetActAlign(s->port, s->slot, align);
        s->has_actuator = 1;
    }
}

void ps2_input_poll(void)
{
    int i;

    if (!sInitDone)
    {
        return;
    }
    for (i = 0; i < PS2_INPUT_MAX_PLAYERS; i++)
    {
        PadSlot *s = &sSlots[i];
        PS2InputState *st = &sState[i];
        struct padButtonStatus pad;
        int state;
        uint16_t held;
        uint16_t n64 = 0;
        size_t m;

        if (!s->open)
        {
            st->connected = 0;
            continue;
        }
        state = padGetState(s->port, s->slot);
        if (state == PAD_STATE_EXECCMD)
        {
            continue; /* busy with a mode/actuator command: keep last state */
        }
        if (state != PAD_STATE_STABLE && state != PAD_STATE_FINDCTP1)
        {
            st->connected = 0;
            s->analog_set = 0;
            continue;
        }
        if (!s->analog_set && state == PAD_STATE_STABLE)
        {
            setup_pad_mode(s);
        }
        if (padRead(s->port, s->slot, &pad) == 0)
        {
            st->connected = 0;
            continue;
        }
        held = (uint16_t)(0xFFFF ^ pad.btns);
        sRawHeld[i] = held;

        for (m = 0; m < sizeof(sDefaultMap) / sizeof(sDefaultMap[0]); m++)
        {
            if (held & sDefaultMap[m].ps2)
            {
                n64 |= sDefaultMap[m].n64;
            }
        }

        /* Digital-only pads report 0x7F/0x80-ish axes or garbage; only trust
         * the sticks in analog modes (mode high nibble 7 = DualShock). */
        if ((pad.mode >> 4) == 0x7 || (pad.mode >> 4) == 0x5)
        {
            st->stick_x = stick_to_n64(pad.ljoy_h, 0);
            st->stick_y = stick_to_n64(pad.ljoy_v, 1); /* PS2 up = 0, N64 up = + */

            if (pad.rjoy_v < 128 - RSTICK_C_THRESHOLD)
                n64 |= N64_BTN_CU;
            if (pad.rjoy_v > 128 + RSTICK_C_THRESHOLD)
                n64 |= N64_BTN_CD;
            if (pad.rjoy_h < 128 - RSTICK_C_THRESHOLD)
                n64 |= N64_BTN_CL;
            if (pad.rjoy_h > 128 + RSTICK_C_THRESHOLD)
                n64 |= N64_BTN_CR;
        }
        else
        {
            st->stick_x = st->stick_y = 0;
        }
        st->buttons = n64;
        st->connected = 1;
    }
}

const PS2InputState *ps2_input_state(int player)
{
    return (player >= 0 && player < PS2_INPUT_MAX_PLAYERS) ? &sState[player] : NULL;
}

int ps2_input_raw_combo(int player, uint16_t combo)
{
    return (player >= 0 && player < PS2_INPUT_MAX_PLAYERS) && ((sRawHeld[player] & combo) == combo);
}

/* Debug overlay toggle: SELECT + R3 on player 1. Edge-triggered. */
int ps2_input_overlay_toggle_pressed(void)
{
    static int sLatched;
    int now = ps2_input_raw_combo(0, PS2B_SELECT | PS2B_R3);
    int fire = now && !sLatched;

    sLatched = now;
    return fire;
}

/* Select + L3: cycle the frame-clear diagnostic (render_thread.c). */
int ps2_input_clear_toggle_pressed(void)
{
    static int sLatched;
    int now = ps2_input_raw_combo(0, PS2B_SELECT | PS2B_L3);
    int fire = now && !sLatched;

    sLatched = now;
    return fire;
}

/* Select + R1: capture undrawn framebuffer rows to SSB64.LOG (gbi.c). */
int ps2_input_capture_pressed(void)
{
    static int sLatched;
    int now = ps2_input_raw_combo(0, PS2B_SELECT | PS2B_R1);
    int fire = now && !sLatched;

    sLatched = now;
    return fire;
}

int ps2_input_has_rumble(int player)
{
    return (player >= 0 && player < PS2_INPUT_MAX_PLAYERS) && sSlots[player].open && sSlots[player].has_actuator;
}

void ps2_input_set_rumble(int player, int on)
{
    PadSlot *s;
    char act[6] = { 0, 0, 0, 0, 0, 0 };

    if (!ps2_input_has_rumble(player))
    {
        return;
    }
    s = &sSlots[player];
    if (s->rumble_on == on)
    {
        return;
    }
    s->rumble_on = on;
    act[0] = on ? 1 : 0; /* small motor: on/off, closest to the Rumble Pak */
    padSetActDirect(s->port, s->slot, act);
}

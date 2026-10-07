/*
 * PS2 audio: libultra AI (audio interface) side.
 *
 * Sound is produced by SPU2 voices (spu.c) driven from the game's
 * synthesizer driver (ps2/src/game/ps2_synth.c); the N64 PCM output buffers
 * the audio thread still hands to the AI are never filled or played.
 */
#include <ps2/platform.h>
#include <ps2/spu.h>


static uint32_t sAiFrequency;
static uint32_t sAiQueuedBytes;
void ps2_audio_init(void)
{
    ps2_log("audio: init begin");

    /*
     * The original SDR/sdrdrv transport is intentionally gone here. Hardware
     * proved that path can wedge the shared SIF RPC fabric after the N64 logo.
     * Load the dedicated minimal server, then initialize the SPU2 backend
     * synchronously before gameplay so storage and audio never race each other
     * during the 4 MiB sample-set preload.
     */
    if (ps2_iop_load_audio_driver() < 0)
    {
        ps2_log("audio: dedicated IOP server failed to start; continuing silent");
        return;
    }

    if (ps2_spu_init() < 0)
    {
        ps2_log("audio: SPU2 backend unavailable; continuing silent");
        return;
    }

    ps2_log("audio: init complete");
}

int32_t ps2_audio_ai_set_frequency(uint32_t frequency)
{
    sAiFrequency = frequency;
    return (int32_t)frequency;
}

int32_t ps2_audio_ai_set_next_buffer(void *buf, uint32_t size)
{
    (void)buf;
    sAiQueuedBytes = size;
    return 0;
}

uint32_t ps2_audio_ai_get_length(void)
{
    /* Report an empty DMA FIFO so the audio thread never stalls on it. */
    return 0;
}

uint32_t ps2_audio_memory_used(void)
{
    return 0;
}

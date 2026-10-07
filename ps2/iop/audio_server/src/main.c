#include <irx.h>
#include <libsd.h>
#include <loadcore.h>
#include <sifrpc.h>
#include <stdio.h>
#include <thbase.h>
#include <types.h>

IRX_ID("ssbaudio", 1, 0);

#define SSB_AUDIO_RPC_ID 0x53424155u
#define SSB_AUDIO_RPC_WORDS 2048u
#define SSB_AUDIO_MAX_BATCH 512u
#define SSB_AUDIO_UPLOAD_HDR 64u

enum {
    SSB_AUDIO_CMD_INIT = 1,
    SSB_AUDIO_CMD_BATCH = 2,
    SSB_AUDIO_CMD_UPLOAD = 3,
    SSB_AUDIO_CMD_GET_PARAM = 4,
    SSB_AUDIO_CMD_GET_ADDR = 5
};

static SifRpcDataQueue_t sQueue;
static SifRpcServerData_t sServer;
static u32 sRpcBuffer[SSB_AUDIO_RPC_WORDS] __attribute__((aligned(64)));
static u32 sReply[4] __attribute__((aligned(64)));
static u32 sBatchReturns[SSB_AUDIO_MAX_BATCH] __attribute__((aligned(64)));

static void *rpc_server(u32 func, void *data, u32 size)
{
    u32 *u = (u32 *)data;

    sReply[0] = (u32)-1;
    sReply[1] = 0;

    switch (func)
    {
    case SSB_AUDIO_CMD_INIT:
        sReply[0] = (u32)sceSdInit(0);
        break;

    case SSB_AUDIO_CMD_BATCH:
    {
        u32 count;
        sceSdBatch *batch;

        if (size < 4)
            break;
        count = u[0];
        if (count == 0 || count > SSB_AUDIO_MAX_BATCH ||
            size < 4 + count * (u32)sizeof(sceSdBatch))
            break;

        batch = (sceSdBatch *)&u[1];
        sReply[0] = (u32)sceSdProcBatch(batch, sBatchReturns, count);
        break;
    }

    case SSB_AUDIO_CMD_UPLOAD:
    {
        u32 spu_addr;
        u32 bytes;
        u8 *payload;
        int ret;

        if (size < SSB_AUDIO_UPLOAD_HDR)
            break;
        spu_addr = u[0];
        bytes = u[1];
        if (bytes == 0 || SSB_AUDIO_UPLOAD_HDR + bytes > size)
            break;

        payload = (u8 *)data + SSB_AUDIO_UPLOAD_HDR;
        ret = sceSdVoiceTrans(0, SD_TRANS_WRITE | SD_TRANS_MODE_DMA,
                              payload, (u32 *)spu_addr, bytes);
        if (ret < 0)
        {
            sReply[0] = (u32)ret;
            break;
        }
        (void)sceSdVoiceTransStatus(0, 1);
        sReply[0] = 0;
        break;
    }

    case SSB_AUDIO_CMD_GET_PARAM:
        if (size >= 4)
            sReply[0] = (u32)sceSdGetParam((u16)u[0]);
        break;

    case SSB_AUDIO_CMD_GET_ADDR:
        if (size >= 4)
            sReply[0] = sceSdGetAddr((u16)u[0]);
        break;

    default:
        break;
    }

    return sReply;
}

static void rpc_thread(void *arg)
{
    (void)arg;
    sceSifInitRpc(0);
    sceSifSetRpcQueue(&sQueue, GetThreadId());
    sceSifRegisterRpc(&sServer, SSB_AUDIO_RPC_ID, (void *)rpc_server,
                      sRpcBuffer, NULL, NULL, &sQueue);
    printf("ssbaudio: RPC server ready\n");
    sceSifRpcLoop(&sQueue);
}

int _start(int argc, char *argv[])
{
    iop_thread_t thread;
    int tid;

    (void)argc;
    (void)argv;

    thread.attr = TH_C;
    thread.thread = rpc_thread;
    thread.priority = 39;
    thread.stacksize = 0x1000;
    thread.option = 0;

    tid = CreateThread(&thread);
    if (tid < 0)
        return MODULE_NO_RESIDENT_END;

    StartThread(tid, NULL);
    return MODULE_RESIDENT_END;
}

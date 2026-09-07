/* 接收检查复用正式驱动与解码，不创建控制线程。原始帧与诊断同步写入SD。 */
#include <errno.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/kernel.h>
#include "BspCan.h"
#include "BspCanZephyr.h"
#include "BspRc.h"
#include "CanReceive.h"
#include "ManualInputSnapshot.h"
#include "MotorInst.h"
#include "SdLog.h"

#define M_RECEIVE_SLOTS 64u
#define M_RECEIVE_BUSES 3u

typedef struct
{
    uint32_t bus;
    uint32_t id;
    uint32_t count;
    uint32_t lastTick;
    uint32_t dlc;
    uint8_t data[8];
} MReceiveFrame;

typedef struct
{
    uint32_t received;
    uint32_t dropped;
    uint32_t txRequests;
    uint32_t txFailed;
    uint32_t lastError;
    uint32_t rxErrors;
    uint32_t txErrors;
    uint32_t busOff;
} MReceiveBus;

typedef struct
{
    uint32_t magic;
    uint32_t sequence;
    uint32_t tick;
    uint32_t ready;
    uint32_t routeConflicts;
    uint32_t rcFrames;
    uint32_t rcRejected;
    uint32_t rcOnline;
    uint32_t rcAge;
    uint32_t rcUartErrors;
    uint32_t rcBadSize;
    uint32_t rcDropped;
    uint32_t tableFull;
    uint32_t txBlocked;
    int32_t channels[5];
    uint32_t switches[2];
    MReceiveBus buses[M_RECEIVE_BUSES];
    MReceiveFrame frames[M_RECEIVE_SLOTS];
} MReceiveStatus;

volatile MReceiveStatus MReceiveDiag = {.magic = 0x4d525843u};
static MReceiveStatus MReceiveWork = {.magic = 0x4d525843u};
static struct k_thread MReceiveThread;
K_THREAD_STACK_DEFINE(MReceiveStack, 4096);
extern volatile uint32_t BspCanReceiveOnlyBlocked;

static void MReceiveRecord(const BspCanFrame *frame)
{
    /* 先保存所有ID，再做配置匹配；未知ID和未配置电机也必须能离线核对。 */
    sdlog_can_rx_t sample = {.bus = frame->bus, .dlc = frame->dlc, .std_id = frame->std_id};
    memcpy(sample.data, frame->data, sizeof(sample.data));
    SdLogWrite(SDLOG_TAG_CAN_RX, &sample, sizeof(sample));
    for (uint32_t i = 0u; i < M_RECEIVE_SLOTS; i++) {
        MReceiveFrame *slot = &MReceiveWork.frames[i];
        if (slot->count == 0u || (slot->bus == frame->bus && slot->id == frame->std_id)) {
            slot->bus = frame->bus;
            slot->id = frame->std_id;
            slot->count++;
            slot->lastTick = k_uptime_get_32();
            slot->dlc = frame->dlc;
            memcpy(slot->data, frame->data, sizeof(slot->data));
            return;
        }
    }
    MReceiveWork.tableFull++;
}

static void MReceivePublish(void)
{
    ManualInputSnapshot input;
    BspRcDiag rc;
    MotorInstDiag motors;
    MReceiveWork.tick = k_uptime_get_32();
    MReceiveWork.ready = BspCanZephyrReady();
    MReceiveWork.rcFrames = ManualInputGetSbusFrameCount();
    MReceiveWork.rcRejected = ManualInputGetSbusRejectCount();
    MReceiveWork.txBlocked = BspCanReceiveOnlyBlocked;
    if (ManualInputSnapshotRead(&input)) {
        MReceiveWork.rcOnline = input.online;
        MReceiveWork.rcAge = input.sourceAgeMs;
        for (uint32_t i = 0u; i < 5u; i++) {
            MReceiveWork.channels[i] = input.manual.rc.ch[i];
        }
        MReceiveWork.switches[0] = input.manual.rc.s[0];
        MReceiveWork.switches[1] = input.manual.rc.s[1];
    } else {
        MReceiveWork.rcOnline = 0u;
        MReceiveWork.rcAge = UINT32_MAX;
    }
    BspRcGetDiag(&rc);
    MReceiveWork.rcUartErrors = rc.uart_error_cnt;
    MReceiveWork.rcBadSize = rc.rx_bad_size_cnt;
    MReceiveWork.rcDropped = BspRcSbusRxGetDropCount();
    if (MotorInstGetDiag(&motors)) {
        MReceiveWork.routeConflicts = motors.feedback_conflict_count;
    }
    for (uint8_t bus = 1u; bus <= M_RECEIVE_BUSES; bus++) {
        MReceiveBus *state = &MReceiveWork.buses[bus - 1u];
        state->received = BspCanRxGetCount(bus);
        state->dropped = BspCanRxGetDropCount(bus);
        state->txRequests = BspCanGetTxCount(bus);
        state->txFailed = BspCanGetTxFailCount(bus);
        state->lastError = BspCanGetLastError(bus);
        state->rxErrors = BspCanGetRxErrorCount(bus);
        state->txErrors = BspCanGetTxErrorCount(bus);
        state->busOff = BspCanGetProtocolBusOff(bus);
    }
    /* 奇数表示复制中，DAP前后核对序号，避免把半帧当成稳定记录。 */
    MReceiveWork.sequence += 2u;
    MReceiveDiag.sequence = MReceiveWork.sequence - 1u;
    __sync_synchronize();
    memcpy((uint8_t *)&MReceiveDiag + 8u, (const uint8_t *)&MReceiveWork + 8u,
           sizeof(MReceiveWork) - 8u);
    __sync_synchronize();
    MReceiveDiag.sequence = MReceiveWork.sequence;
    /* 保存驱动丢帧、遥控错误、发送计数；无需依赖DAP一直连着。 */
    SdLogWrite(SDLOG_TAG_RECEIVE_CHECK, &MReceiveWork, offsetof(MReceiveStatus, frames));
}

static void MReceiveRun(void *a, void *b, void *c)
{
    ARG_UNUSED(a);
    ARG_UNUSED(b);
    ARG_UNUSED(c);
    uint32_t nextReport = 0u;
    for (;;) {
        BspCanFrame frame;
        uint8_t rc[BSP_RC_SBUS_FRAME_LENGTH];
        /* 有界处理，给IMU及串口工作队列留出执行机会。 */
        for (uint32_t i = 0u; i < 96u && BspCanRxPop(&frame); i++) {
            MReceiveRecord(&frame);
            CAN_rx_process_frame(frame.bus, frame.std_id, frame.dlc, frame.data);
        }
        while (BspRcSbusRxPop(rc)) {
            ManualInputOnSbusFrame(rc);
        }
        uint32_t now = k_uptime_get_32();
        if ((int32_t)(now - nextReport) >= 0) {
            MReceivePublish();
            nextReport = now + 100u;
        }
        k_msleep(1);
    }
}

int MReceiveStart(void)
{
    MotorInstRefresh();
    can_filter_init();
    if (BspCanZephyrReady() == 0u) {
        return -EIO;
    }
    k_thread_create(&MReceiveThread, MReceiveStack, K_THREAD_STACK_SIZEOF(MReceiveStack),
                    MReceiveRun, NULL, NULL, NULL, K_PRIO_PREEMPT(6), 0, K_NO_WAIT);
    return 0;
}

/* 使用生产型号表、组帧、最终授权检查和急停函数；总线调用由记录器替代。 */
#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "MotorConfig.h"
#include "ChassisPowerLimiter.c"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
Config g_config = {
#include "ConfigHardware.inc"
};
#pragma GCC diagnostic pop

enum { CAN_RM_GROUP_0X1FF_ID = 0x1FF };
typedef struct { uint8_t dummy; } BspCanTxTicket;
typedef struct { uint8_t dummy; } sdlog_actuator_current_t;
typedef struct { MotorId motorId; uint8_t bus; const motor_node_param_t *node; } MotorRoute;
typedef struct { uint8_t bus; uint16_t std_id; } CanTxEmergencyMitRoute;
static MotorCmd s_can_tx_cmd_cache[MotorCount];
static ControlOutputStamp s_can_tx_owner_cache[MotorCount];
static uint8_t s_can_tx_cmd_cache_valid[MotorCount];
static uint8_t s_can_tx_cmd_expired[MotorCount], s_can_tx_cmd_unlock_blocked[MotorCount];
static uint32_t s_can_tx_authority_reject_count[MotorCount];
static uint8_t s_can_tx_rm_group_configured[2][5];
static uint8_t s_can_tx_emergency_rm_group[3][5];
static CanTxEmergencyMitRoute s_can_tx_emergency_mit[MotorCount];
static uint8_t last_can1ff_status;
static uint8_t Locked, EmergencyValid = 1;
static uint8_t Authorized[MotorCount];
static unsigned Sent, Queued, Forced;
static unsigned ScheduleDepth, RefreshEnabled, RevokeDuringRefresh, RouteAllowed = 1;
static MotorRoute TestRoutes[2];
static MotorCmd Latest[2];
static struct { uint8_t bus, data[8]; uint16_t id; } Frames[32];
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint8_t CanTxActuatorIdValid(MotorId id) { return id < MotorCount; }
static uint8_t RobotSafetyOutputLocked(void) { return Locked; }
static void vTaskSuspendAll(void) { assert(ScheduleDepth == 0); ScheduleDepth++; }
static int xTaskResumeAll(void) { assert(ScheduleDepth == 1); ScheduleDepth--; return 0; }
uint8_t LowCmdOutputSnapshotAuthorized(MotorId id, const MotorCmd *cmd, const ControlOutputStamp *owner)
{ (void)owner; return Authorized[id] && (!RefreshEnabled ||
    (id < 2 && cmd->seq == Latest[id].seq && cmd->current == Latest[id].current)); }
static const MotorRoute *MotorRouteFindByMotor(MotorId id)
{ return RefreshEnabled && id < 2 ? &TestRoutes[id] : NULL; }
uint8_t LowCmdGetMotorStamped(MotorId id, MotorCmd *cmd, ControlOutputStamp *owner)
{ assert(ScheduleDepth == 1 && id < 2); *cmd = Latest[id]; memset(owner, 0, sizeof(*owner)); return 1; }
static uint8_t CanTxRouteAllowed(const MotorRoute *route) { (void)route; return RouteAllowed; }
static void CanTxProcessInstance(const MotorRoute *route, uint8_t allowed, sdlog_actuator_current_t *log);
static void CanTxUpdateApplied(const MotorRoute *r, const MotorCmd *c, int16_t v, uint8_t f)
{ (void)r; (void)c; (void)v; (void)f; Forced++; }
static void CanTxLogMotorCmd(sdlog_actuator_current_t *l, const MotorRoute *r, int16_t v)
{ (void)l; (void)r; (void)v; }
static uint8_t CanTxPhysicalPrepare(MotorId id, const MotorCmd *c, MotorTxIdentity *i)
{ (void)id; (void)c; (void)i; return 1; }
static BspCanTxTicket CanTxPhysicalTicketNext(void) { BspCanTxTicket t = {1}; return t; }
void LowStateUpdateTxQueued(MotorId id, const MotorTxIdentity *i)
{ (void)id; (void)i; Queued++; }
static void CanTxPhysicalArm(MotorId id, const MotorTxIdentity *i, const BspCanTxTicket *t)
{ (void)id; (void)i; (void)t; }
static int BspCanTx(uint8_t bus, uint16_t id, const uint8_t *data, uint8_t len)
{
    assert(Sent < 32 && len == 8);
    Frames[Sent].bus = bus; Frames[Sent].id = id;
    memcpy(Frames[Sent++].data, data, 8);
    return 0;
}
static int BspCanTxTracked(uint8_t b, uint16_t id, const uint8_t *d, uint8_t n,
                           const BspCanTxTicket *t, uint8_t *tracked)
{ (void)t; *tracked = 1; return BspCanTx(b, id, d, n); }
static void BspCanFaultLock(void) {}
static int BspCanFaultTx(uint8_t b, uint16_t id, const uint8_t *d, uint8_t n)
{ return BspCanTx(b, id, d, n); }
static uint8_t CanTxEmergencyTableValid(uint8_t *m, uint8_t *r)
{ *m = 0; *r = 0; return EmergencyValid; }
#include "ExtractedRm.inc"

/* 周边换算留给既有协议测试；这里核对真实逐组刷新在暂停任务切换期间处理最新命令。 */
static void CanTxProcessInstance(const MotorRoute *route, uint8_t allowed, sdlog_actuator_current_t *log)
{
    (void)log;
    assert(ScheduleDepth == 1);
    const MotorId id = route->motorId;
    const MotorCmd *cmd = &s_can_tx_cmd_cache[id];
    const int16_t current = allowed && LowCmdOutputSnapshotAuthorized(id, cmd, &s_can_tx_owner_cache[id]) ?
                                MotorCfgLimitCurrentNode(route->node, cmd->current) : 0;
    assert(CanTxStoreRmCurrent(route->bus, route->node, current, id));
    if (RevokeDuringRefresh) Authorized[id] = 0;
}

static void CheckRoute(MotorModel model, uint8_t nodeId, uint8_t mode,
                       uint16_t expectedGroup, uint8_t expectedSlot, int16_t limit)
{
    motor_node_param_t node = {.model = model, .can_id = nodeId, .control_mode = mode};
    uint16_t group = 0;
    uint8_t slot = 9;
    assert(MotorCfgRmTxSlot(&node, &group, &slot));
    assert(group == expectedGroup && slot == expectedSlot);
    assert(MotorCfgLimitCurrentNode(&node, 32767) == limit);
    assert(MotorCfgLimitCurrentNode(&node, -32768) == -limit);
    assert(MotorCfgLimitCurrentNode(&node, 0) == 0);
}

int main(void)
{
    motor_node_param_t node = {.model = MOTOR_MODEL_6020, .can_id = 1};
    uint16_t group;
    uint8_t slot;
    fp32 amps;
    const uint8_t zero[8] = {0};
    const motor_node_param_t *gimbalNodes[] = {
        &g_config.motor.yaw, &g_config.motor.yaw_upper, &g_config.motor.pitch};
    for (unsigned i = 0; i < sizeof(gimbalNodes) / sizeof(gimbalNodes[0]); i++) {
        const motor_node_param_t *configured = gimbalNodes[i];
        if (configured->model == MOTOR_MODEL_6020 && configured->can_id != 0) {
            assert(MotorCfgControlMode(configured) == MOTOR_CONTROL_MODE_VOLTAGE);
            assert(MotorCfgLimitCurrentNode(configured, 32767) == 16384);
        }
    }
#if defined(RM_TEST_INFANTRY)
    assert(MotorCfgCanBus(1, &g_config.motor.yaw) == 1);
    assert(MotorCfgCanBus(1, &g_config.motor.pitch) == 2);
    assert(MotorCfgCanBus(1, &g_config.motor.trigger) == 2);
    assert(MotorCfgFeedbackId(&g_config.motor.pitch) == 0x206);
    assert(MotorCfgFeedbackId(&g_config.motor.trigger) == 0x207);
    CanTxClearRmFrames();
    CanTxMarkRmGroupConfigured(2, &g_config.motor.pitch);
    CanTxMarkRmGroupConfigured(2, &g_config.motor.trigger);
    assert(CanTxStoreRmCurrent(2, &g_config.motor.pitch, 1000, 0));
    assert(CanTxStoreRmCurrent(2, &g_config.motor.trigger, -2000, 1));
    s_can_tx_cmd_cache_valid[0] = s_can_tx_cmd_cache_valid[1] = 1;
    Authorized[0] = Authorized[1] = 1;
    CanTxEmitRmFrames(NULL, 0);
    const uint8_t sharedExpected[8] = {0, 0, 3, 0xE8, 0xF8, 0x30, 0, 0};
    assert(Sent == 1 && Frames[0].id == 0x1FF && Frames[0].bus == 2);
    assert(memcmp(Frames[0].data, sharedExpected, 8) == 0 && Queued == 2);
    Authorized[0] = 0; Sent = Queued = 0;
    CanTxEmitRmFrames(NULL, 0);
    assert(Sent == 1 && Queued == 1 && Frames[0].data[2] == 0 && Frames[0].data[3] == 0);
    assert(Frames[0].data[4] == 0xF8 && Frames[0].data[5] == 0x30);
    memset(s_can_tx_rm_group_configured, 0, sizeof(s_can_tx_rm_group_configured));
#endif
    for (uint8_t id = 1; id <= 7; id++) {
        CheckRoute(MOTOR_MODEL_6020, id, MOTOR_CONTROL_MODE_INHERIT,
                   id <= 4 ? 0x1FF : 0x2FF, (id - 1) % 4, 25000);
        CheckRoute(MOTOR_MODEL_6020, id, MOTOR_CONTROL_MODE_CURRENT,
                   id <= 4 ? 0x1FE : 0x2FE, (id - 1) % 4, 16384);
        assert(MotorCfgEncoderRange(MOTOR_MODEL_6020) == 8192);
    }
    for (uint8_t id = 1; id <= 8; id++) {
        CheckRoute(MOTOR_MODEL_3508, id, 0, id <= 4 ? 0x200 : 0x1FF, (id - 1) % 4, 16384);
        CheckRoute(MOTOR_MODEL_2006, id, 0, id <= 4 ? 0x200 : 0x1FF, (id - 1) % 4, 10000);
        CheckRoute(MOTOR_MODEL_3510, id, 0, id <= 4 ? 0x200 : 0x1FF, (id - 1) % 4, 16000);
    }
    CheckRoute(MOTOR_MODEL_6623, 4, 0, 0x1FF, 3, 5000);
    assert(!ChassisPowerLimiterModelCurrentToAmpere(&node, 16384, &amps));
    node.control_mode = MOTOR_CONTROL_MODE_CURRENT;
    assert(ChassisPowerLimiterModelCurrentToAmpere(&node, 8192, &amps) && fabsf(amps - 1.5f) < 0.0001f);
    node.control_mode = MOTOR_CONTROL_MODE_VOLTAGE;
    node.command_limit_abs = 16384;
    assert(MotorCfgLimitCurrentNode(&node, 30000) == 16384);
    node.command_limit_abs = 30000;
    assert(MotorCfgLimitCurrentNode(&node, 30000) == 25000);
    node.control_mode = MOTOR_CONTROL_MODE_CURRENT;
    assert(MotorCfgLimitCurrentNode(&node, 30000) == 16384);
    node.can_id = 8;
    assert(!MotorCfgRmTxSlot(&node, &group, &slot) && MotorCfgLimitCurrentNode(&node, 100) == 0);
    node.can_id = 0;
    assert(!MotorCfgRmTxSlot(&node, &group, &slot));
    node.can_id = 1; node.model = MOTOR_MODEL_3508; node.control_mode = MOTOR_CONTROL_MODE_VOLTAGE;
    assert(!MotorCfgRmTxSlot(&node, &group, &slot) && MotorCfgLimitCurrentNode(&node, 100) == 0);

    /* 每组分别走真实打包/最终授权检查，包括电流模式和高编号节点的零命令。 */
    const MotorModel models[5] = {MOTOR_MODEL_3508, MOTOR_MODEL_6020, MOTOR_MODEL_6020, MOTOR_MODEL_6020, MOTOR_MODEL_6020};
    const uint8_t ids[5] = {1, 1, 7, 1, 7};
    const uint8_t modes[5] = {0, MOTOR_CONTROL_MODE_VOLTAGE, MOTOR_CONTROL_MODE_VOLTAGE, MOTOR_CONTROL_MODE_CURRENT, MOTOR_CONTROL_MODE_CURRENT};
    for (uint8_t g = 0; g < 5; g++) {
        for (uint8_t bus = 1; bus <= 2; bus++) {
            node = (motor_node_param_t){.model = models[g], .can_id = ids[g], .control_mode = modes[g]};
            memset(s_can_tx_rm_group_configured, 0, sizeof(s_can_tx_rm_group_configured));
            CanTxClearRmFrames(); Sent = Queued = Forced = 0; Locked = 0;
            s_can_tx_cmd_cache_valid[0] = Authorized[0] = 1;
            CanTxMarkRmGroupConfigured(bus, &node);
            assert(CanTxStoreRmCurrent(bus, &node, -1234, 0));
            CanTxEmitRmFrames(NULL, 0);
            assert(Sent == 1 && Queued == 1 && Frames[0].bus == bus && Frames[0].id == s_can_tx_rm_group_ids[g]);
            slot = (ids[g] - 1) % 4;
            assert(Frames[0].data[slot * 2] == 0xFB && Frames[0].data[slot * 2 + 1] == 0x2E);
            if (ids[g] > 4) assert(Frames[0].data[6] == 0 && Frames[0].data[7] == 0);
            Authorized[0] = 0; Sent = Queued = 0;
            CanTxEmitRmFrames(NULL, 0);
            assert(Sent == 1 && Queued == 0 && Forced == 1 && memcmp(Frames[0].data, zero, 8) == 0);
            assert(CanTxStoreRmCurrent(bus, &node, 321, 0));
            Locked = 1; Sent = 0;
            CanTxEmitRmFrames(NULL, 0);
            assert(Sent == 1 && memcmp(Frames[0].data, zero, 8) == 0);
            memset(s_can_tx_emergency_rm_group, 0, sizeof(s_can_tx_emergency_rm_group));
            s_can_tx_emergency_rm_group[bus - 1][g] = 1; Sent = 0;
            CanTxEmergencyStopNow();
            assert(Sent == 1 && Frames[0].id == s_can_tx_rm_group_ids[g] && memcmp(Frames[0].data, zero, 8) == 0);
        }
    }
    /* 命令在整车快照之后正常更新：必须发新命令，不能插入零帧。 */
    RefreshEnabled = 1; Locked = 0; Sent = Queued = 0;
    memset(s_can_tx_rm_group_configured, 0, sizeof(s_can_tx_rm_group_configured));
    CanTxClearRmFrames();
    motor_node_param_t freshNodes[2] = {
        {.model = MOTOR_MODEL_6020, .can_id = 2, .control_mode = MOTOR_CONTROL_MODE_VOLTAGE},
        {.model = MOTOR_MODEL_2006, .can_id = 7},
    };
    for (unsigned i = 0; i < 2; i++) {
        TestRoutes[i] = (MotorRoute){.motorId = (MotorId)i, .bus = 2, .node = &freshNodes[i]};
        Latest[i] = (MotorCmd){.active=1, .mode=MotorModeCurrent, .seq=22, .current=(int16_t)(1200+i*400)};
        s_can_tx_cmd_cache[i] = Latest[i]; s_can_tx_cmd_cache[i].seq = 21;
        Authorized[i] = s_can_tx_cmd_cache_valid[i] = 1;
        assert(CanTxStoreRmCurrent(2, &freshNodes[i], 900, (MotorId)i));
        CanTxMarkRmGroupConfigured(2, &freshNodes[i]);
    }
    CanTxEmitRmFrames(NULL, 0);
    assert(Sent == 1 && ScheduleDepth == 0 && Queued == 2);
    const uint8_t freshExpected[8] = {0,0,4,0xB0,6,0x40,0,0};
    assert(memcmp(Frames[0].data, freshExpected, 8) == 0);
    /* 换算后撤销权限、全车锁定和本轮已锁定均仍发零。 */
    RevokeDuringRefresh = 1; Sent = Queued = 0;
    CanTxEmitRmFrames(NULL, 0);
    assert(Sent == 1 && Queued == 0 && ScheduleDepth == 0 && memcmp(Frames[0].data, zero, 8) == 0);
    RevokeDuringRefresh = 0; Authorized[0] = Authorized[1] = 1; Sent = 0;
    CanTxEmitRmFrames(NULL, 1);
    assert(Sent == 1 && ScheduleDepth == 0 && memcmp(Frames[0].data, zero, 8) == 0);
    RouteAllowed = 0; Sent = 0;
    CanTxEmitRmFrames(NULL, 0);
    assert(Sent == 1 && ScheduleDepth == 0 && memcmp(Frames[0].data, zero, 8) == 0);
    RouteAllowed = 1; Locked = 1; Sent = 0;
    CanTxEmitRmFrames(NULL, 0);
    assert(Sent == 1 && ScheduleDepth == 0 && memcmp(Frames[0].data, zero, 8) == 0);
    RefreshEnabled = 0;
    EmergencyValid = 0; Sent = 0;
    CanTxEmergencyStopNow();
    assert(Sent == 10);
    for (unsigned i = 0; i < Sent; i++) assert(memcmp(Frames[i].data, zero, 8) == 0);
    puts("PASS: RM modes, limits, all slots, byte packing, both buses, authority, stop frames, current units");
    return 0;
}

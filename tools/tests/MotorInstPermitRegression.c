/*
 * MotorInst 许可写入主机回归。
 * 直接包含生产实现，钉住输入检查、BestEffort 预过滤和单次原子发布语义。
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "MotorInst.c"

Config g_config;

static MotorCmd s_test_cmd[MotorCount];
static uint16_t s_test_inhibit[MotorCount];
static uint32_t s_test_seq;
static uint32_t s_test_motor_batch_calls;
static uint32_t s_test_current_batch_calls;
static uint32_t s_test_clear_batch_calls;
static uint8_t s_test_last_count;
static uint8_t s_test_force_batch_fail;
static motor_node_param_t s_test_nodes[MotorCount];

const MotorModelDbEntry *MotorModelDbGet(MotorModel model)
{
    (void)model;
    return NULL;
}

const MotorModelRxDesc *MotorModelDbGetRxDesc(MotorModel model)
{
    (void)model;
    return NULL;
}

static uint8_t TestPermitAccepts(const MotorId *ids,
                                 uint8_t count,
                                 const ControlOutputPermit *permit)
{
    uint32_t mask = 0u;

    if (permit == NULL || permit->stamp.valid != 1u || s_test_force_batch_fail != 0u)
    {
        return 0u;
    }
    for (uint8_t i = 0u; i < count; i++)
    {
        if ((uint32_t)ids[i] >= (uint32_t)MotorCount ||
            s_test_inhibit[ids[i]] > (uint16_t)LOWCMD_WRITER_CONTROL)
        {
            return 0u;
        }
        for (uint8_t j = 0u; j < i; j++)
        {
            if (ids[i] == ids[j])
            {
                return 0u;
            }
        }
        mask |= 1ul << (uint32_t)ids[i];
    }
    return ControlOutputPermitAllows(permit, mask);
}

uint8_t LowCmdSetMotorManyWithPermit(const MotorId *ids,
                                     const MotorCmd *cmds,
                                     uint8_t count,
                                     const ControlOutputPermit *permit)
{
    s_test_motor_batch_calls++;
    s_test_last_count = count;
    if (cmds == NULL || TestPermitAccepts(ids, count, permit) == 0u)
    {
        return 0u;
    }

    s_test_seq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        s_test_cmd[ids[i]] = cmds[i];
        s_test_cmd[ids[i]].seq = s_test_seq;
    }
    return 1u;
}

uint8_t LowCmdSetCurrentManyWithPermit(const MotorId *ids,
                                       const int16_t *currents,
                                       uint8_t count,
                                       const ControlOutputPermit *permit)
{
    s_test_current_batch_calls++;
    s_test_last_count = count;
    if (currents == NULL || TestPermitAccepts(ids, count, permit) == 0u)
    {
        return 0u;
    }

    s_test_seq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        (void)memset(&s_test_cmd[ids[i]], 0, sizeof(s_test_cmd[ids[i]]));
        s_test_cmd[ids[i]].active = 1u;
        s_test_cmd[ids[i]].mode = (uint8_t)MotorModeCurrent;
        s_test_cmd[ids[i]].current = currents[i];
        s_test_cmd[ids[i]].seq = s_test_seq;
    }
    return 1u;
}

uint8_t LowCmdClearManyWithPermit(const MotorId *ids,
                                  uint8_t count,
                                  const ControlOutputPermit *permit)
{
    s_test_clear_batch_calls++;
    s_test_last_count = count;
    if (TestPermitAccepts(ids, count, permit) == 0u)
    {
        return 0u;
    }

    s_test_seq++;
    for (uint8_t i = 0u; i < count; i++)
    {
        (void)memset(&s_test_cmd[ids[i]], 0, sizeof(s_test_cmd[ids[i]]));
        s_test_cmd[ids[i]].seq = s_test_seq;
    }
    return 1u;
}

uint8_t LowCmdGetInhibitWriter(MotorId id, uint16_t *out)
{
    if ((uint32_t)id >= (uint32_t)MotorCount || out == NULL)
    {
        return 0u;
    }
    *out = s_test_inhibit[id];
    return 1u;
}

/* 下列兼容接口只用来满足直接包含 MotorInst.c 时的链接，回归本身不调用它们。 */
void LowCmdClear(MotorId id)
{
    (void)id;
}

uint8_t LowCmdClearManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    (void)ids;
    (void)count;
    (void)writer;
    return 0u;
}

uint8_t LowCmdInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    (void)ids;
    (void)count;
    (void)writer;
    return 0u;
}

uint8_t LowCmdReleaseInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    (void)ids;
    (void)count;
    (void)writer;
    return 0u;
}

uint8_t LowCmdSetMotorMany(const MotorId *ids, const MotorCmd *cmds, uint8_t count)
{
    (void)ids;
    (void)cmds;
    (void)count;
    return 0u;
}

uint8_t LowCmdSetMotor(MotorId id, const MotorCmd *cmd)
{
    (void)id;
    (void)cmd;
    return 0u;
}

void LowCmdSetCurrent(MotorId id, int16_t current)
{
    (void)id;
    (void)current;
}

uint8_t LowCmdSetCurrentMany(const MotorId *ids, const int16_t *currents, uint8_t count)
{
    (void)ids;
    (void)currents;
    (void)count;
    return 0u;
}

uint8_t LowCmdGetMotor(MotorId id, MotorCmd *out)
{
    (void)id;
    (void)out;
    return 0u;
}

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    (void)id;
    (void)out;
    return 0u;
}

uint8_t LowStateGetMotorMany(const MotorId *ids, MotorState *out, uint8_t count)
{
    (void)ids;
    (void)out;
    (void)count;
    return 0u;
}

static int TestCheck(int condition, const char *message)
{
    if (condition == 0)
    {
        (void)fprintf(stderr, "FAIL: %s\n", message);
        return 0;
    }
    return 1;
}

static ControlOutputPermit TestPermit(void)
{
    ControlOutputPermit permit;

    (void)memset(&permit, 0, sizeof(permit));
    permit.stamp.authorityEpoch = 7u;
    permit.stamp.cycleSeq = 11u;
    permit.stamp.controllerId = 3u;
    permit.stamp.domain = 1u;
    permit.stamp.valid = 1u;
    permit.actuatorMask = 0xFFFFFFFFul;
    return permit;
}

static void TestReset(void)
{
    (void)memset(&g_config, 0, sizeof(g_config));
    (void)memset(sMotorInst, 0, sizeof(sMotorInst));
    (void)memset(sMotorById, 0, sizeof(sMotorById));
    (void)memset(sMotorRoute, 0, sizeof(sMotorRoute));
    (void)memset(sRouteByMotor, 0, sizeof(sRouteByMotor));
    (void)memset(s_test_nodes, 0, sizeof(s_test_nodes));
    (void)memset(s_test_cmd, 0, sizeof(s_test_cmd));
    (void)memset(s_test_inhibit, 0, sizeof(s_test_inhibit));
    sMotorInstCount = 0u;
    sMotorRouteCount = 0u;
    sMotorInstReady = 1u;
    s_test_seq = 0u;
    s_test_motor_batch_calls = 0u;
    s_test_current_batch_calls = 0u;
    s_test_clear_batch_calls = 0u;
    s_test_last_count = 0u;
    s_test_force_batch_fail = 0u;

    for (uint8_t i = 0u; i < 7u; i++)
    {
        s_test_nodes[i].model = MOTOR_MODEL_3508;
        s_test_nodes[i].can_id = (uint8_t)(i + 1u);
        s_test_nodes[i].can_bus = 1u;
        s_test_nodes[i].protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP;
        s_test_nodes[i].control_mode = (uint8_t)MOTOR_CONTROL_MODE_CURRENT;
        s_test_nodes[i].transport = (uint8_t)MOTOR_TRANSPORT_CAN;
        MotorInstAdd((MotorId)i,
                     MotorRoleChassis,
                     i,
                     1u,
                     MOTOR_INST_INVALID_DETECT_TOE,
                     0u,
                     "test.motor",
                     &s_test_nodes[i],
                     &sMotorMeasure[i]);
    }
    /* Motor6 保留实例但关闭节点；Motor7 及以后没有实例。 */
    s_test_nodes[6].can_id = 0u;
}

static int TestStrictBatchAndInputValidation(void)
{
    const MotorId ids[2] = {Motor0, Motor1};
    const MotorId duplicate_ids[2] = {Motor0, Motor0};
    MotorCmd cmds[2];
    ControlOutputPermit permit = TestPermit();

    TestReset();
    (void)memset(cmds, 0, sizeof(cmds));
    cmds[0].active = 1u;
    cmds[0].mode = (uint8_t)MotorModeCurrent;
    cmds[1] = cmds[0];

    if (!TestCheck(MotorInstSetIdsWithPermit(ids, cmds, 2u, &permit) != 0u,
                   "严格批量许可写入失败") ||
        !TestCheck(s_test_motor_batch_calls == 1u && s_test_last_count == 2u,
                   "严格批量应只发布一次") ||
        !TestCheck(s_test_cmd[Motor0].seq == s_test_cmd[Motor1].seq,
                   "严格批量的各轴应共用同一序号"))
    {
        return 0;
    }

    if (!TestCheck(MotorInstSetIdsWithPermit(duplicate_ids, cmds, 2u, &permit) == 0u &&
                       s_test_motor_batch_calls == 1u,
                   "重复 ID 应在 MotorInst 层整批拒绝") ||
        !TestCheck(MotorInstSetIdWithPermit(Motor7, &cmds[0], &permit) == 0u,
                   "严格写入不应接受无实例 ID") ||
        !TestCheck(MotorInstSetIdWithPermit(Motor0, &cmds[0], NULL) == 0u,
                   "严格写入不应接受空许可"))
    {
        return 0;
    }

    permit.stamp.valid = 0u;
    return TestCheck(MotorInstSetIdsWithPermit(ids, cmds, 2u, &permit) == 0u &&
                         s_test_cmd[Motor0].seq == s_test_cmd[Motor1].seq,
                     "失效许可应整批失败");
}

static int TestBestEffortFiltersThenPublishesOnce(void)
{
    const MotorId ids[5] = {Motor0, Motor7, Motor1, Motor6, Motor2};
    const MotorId duplicate_ids[2] = {Motor2, Motor2};
    MotorCmd cmds[5];
    MotorCmd before[MotorCount];
    ControlOutputPermit permit = TestPermit();

    TestReset();
    (void)memset(cmds, 0, sizeof(cmds));
    for (uint8_t i = 0u; i < 5u; i++)
    {
        cmds[i].active = 1u;
        cmds[i].mode = (uint8_t)MotorModeStateTorque;
        cmds[i].tau = (fp32)i;
    }
    s_test_inhibit[Motor1] = (uint16_t)LOWCMD_WRITER_SAFETY;

    if (!TestCheck(MotorInstSetIdsBestEffortWithPermit(ids, cmds, 5u, &permit) == 2u,
                   "BestEffort 应过滤无实例、关闭和高优先级禁写轴") ||
        !TestCheck(s_test_motor_batch_calls == 1u && s_test_last_count == 2u,
                   "BestEffort 过滤后应只提交一个批次") ||
        !TestCheck(s_test_cmd[Motor0].seq != 0u &&
                       s_test_cmd[Motor0].seq == s_test_cmd[Motor2].seq &&
                       s_test_cmd[Motor1].seq == 0u,
                   "BestEffort 健康轴应共用序号且禁写轴不变"))
    {
        return 0;
    }

    if (!TestCheck(MotorInstSetIdsBestEffortWithPermit(duplicate_ids, cmds, 2u, &permit) == 0u &&
                       s_test_motor_batch_calls == 1u,
                   "BestEffort 也应在发布前拒绝重复 ID"))
    {
        return 0;
    }

    (void)memcpy(before, s_test_cmd, sizeof(before));
    s_test_force_batch_fail = 1u;
    if (!TestCheck(MotorInstSetIdsBestEffortWithPermit(ids, cmds, 5u, &permit) == 0u,
                   "并发失败时 BestEffort 应返回 0") ||
        !TestCheck(s_test_motor_batch_calls == 2u,
                   "并发失败后不应逐轴重试") ||
        !TestCheck(memcmp(before, s_test_cmd, sizeof(before)) == 0,
                   "并发失败不应留下半批命令"))
    {
        return 0;
    }
    return 1;
}

static int TestCurrentBindingsAndModeHelpers(void)
{
    const MotorId ids[2] = {Motor0, Motor2};
    const int16_t currents[5] = {100, 200, 300, 400, 500};
    const MotorCurrentBind bindings[5] = {
        {Motor0, 1u},
        {Motor7, 1u},
        {Motor1, 1u},
        {Motor6, 0u},
        {Motor2, 1u},
    };
    const MotorCurrentBind duplicate_bindings[2] = {{Motor0, 1u}, {Motor0, 1u}};
    MotorCmd torque_cmds[2];
    ControlOutputPermit permit = TestPermit();

    TestReset();
    s_test_inhibit[Motor1] = (uint16_t)LOWCMD_WRITER_FAULT;
    if (!TestCheck(MotorInstSetCurrentBindsBestEffortWithPermit(bindings,
                                                                currents,
                                                                5u,
                                                                &permit) == 2u,
                   "绑定电流 BestEffort 过滤结果不正确") ||
        !TestCheck(s_test_current_batch_calls == 1u && s_test_last_count == 2u &&
                       s_test_cmd[Motor0].seq == s_test_cmd[Motor2].seq,
                   "绑定电流应一次批量发布") ||
        !TestCheck(MotorInstSetCurrentBindsBestEffortWithPermit(duplicate_bindings,
                                                                currents,
                                                                2u,
                                                                &permit) == 0u &&
                       s_test_current_batch_calls == 1u,
                   "绑定重复 ID 应在发布前拒绝"))
    {
        return 0;
    }

    if (!TestCheck(MotorInstSetCurrentIdsWithPermit(ids, currents, 2u, &permit) != 0u &&
                       s_test_cmd[Motor0].seq == s_test_cmd[Motor2].seq,
                   "严格电流批量应共用序号") ||
        !TestCheck(MotorInstSetCurrentIdWithPermit(Motor0, 321, &permit) != 0u &&
                       s_test_cmd[Motor0].current == 321,
                   "单轴电流许可接口失败"))
    {
        return 0;
    }

    (void)memset(torque_cmds, 0, sizeof(torque_cmds));
    torque_cmds[0].mode = (uint8_t)MotorModeCurrent;
    torque_cmds[1].mode = (uint8_t)MotorModeCurrent;
    if (!TestCheck(MotorInstSetStateTorqueIdsWithPermit(ids, torque_cmds, 2u, &permit) != 0u &&
                       s_test_cmd[Motor0].active == 1u &&
                       s_test_cmd[Motor0].mode == (uint8_t)MotorModeStateTorque &&
                       s_test_cmd[Motor0].seq == s_test_cmd[Motor2].seq,
                   "StateTorque 批量应统一补齐 active/mode") ||
        !TestCheck(MotorInstSetSpeedIdWithPermit(Motor0, 2.0f, 3.0f, 4.0f, &permit) != 0u &&
                       s_test_cmd[Motor0].mode == (uint8_t)MotorModeSpeed,
                   "Speed 许可接口失败") ||
        !TestCheck(MotorInstSetDampingIdWithPermit(Motor0, 5.0f, 6.0f, &permit) != 0u &&
                       s_test_cmd[Motor0].mode == (uint8_t)MotorModeDamping,
                   "Damping 许可接口失败") ||
        !TestCheck(MotorInstSetDisableIdWithPermit(Motor0, &permit) != 0u &&
                       s_test_cmd[Motor0].mode == (uint8_t)MotorModeDisable,
                   "Disable 许可接口失败"))
    {
        return 0;
    }

    if (!TestCheck(MotorInstClearIdsWithPermit(ids, 2u, &permit) != 0u &&
                       s_test_clear_batch_calls == 1u &&
                       s_test_cmd[Motor0].seq == s_test_cmd[Motor2].seq,
                   "Clear 批量许可接口应原子发布") ||
        !TestCheck(MotorInstClearIdWithPermit(Motor7, &permit) == 0u &&
                       s_test_clear_batch_calls == 1u,
                   "Clear 不应接受无实例 ID"))
    {
        return 0;
    }
    return 1;
}

int main(void)
{
    if (!TestStrictBatchAndInputValidation()) return 1;
    if (!TestBestEffortFiltersThenPublishesOnce()) return 1;
    if (!TestCurrentBindingsAndModeHelpers()) return 1;

    (void)puts("PASS: MotorInst permit publish regression");
    return 0;
}

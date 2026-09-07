#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlRuntimeTestPlatform.h"
#include "ControlRuntime.h"

static uint32_t s_now = 100u;
static ManualInputSnapshot s_manual;
static MotorState s_feedback[MotorCount];
static robot_run_mode_e s_mode = ROBOT_RUN_MODE_FULL;
static robot_run_variant_e s_variant = ROBOT_RUN_VARIANT_NORMAL;
static RobotTaskModule s_singleTask;
static uint8_t s_lifecycleAllowed = 1u;
static uint8_t s_revokeOnCommit;
static uint8_t s_inhibited;
static uint32_t s_stepCount;
static uint32_t s_commitCount;
static uint8_t s_commitAxisCount;
static uint8_t s_nanOutput;
static uint8_t s_imuAvailable = 1u;
static InsSnapshot s_imuSnapshot;
static ControlAlgorithmInput s_lastInput;

static const MotorInst s_motor[] = {
    {Motor0, 1u},
    {Motor1, 1u},
    {Motor2, 1u},
    {Motor3, 0u},
    {Motor4, 1u},
};
static const MotorRoute s_route = {1u};

void ControlMgrTestEnterCritical(void)
{
}

void ControlMgrTestExitCritical(void)
{
}

uint32_t BspTimeGetTickMs(void)
{
    return s_now;
}

uint8_t ManualInputSnapshotRead(ManualInputSnapshot *out)
{
    if (out == NULL)
    {
        return 0u;
    }
    *out = s_manual;
    return 1u;
}

uint8_t ControlInputSwitchIsPos(uint16_t raw, uint8_t pos)
{
    return (uint8_t)(raw == pos);
}

uint8_t RobotLifecycleOutputAllowed(void)
{
    return s_lifecycleAllowed;
}

robot_run_mode_e robot_mode_current(void)
{
    return s_mode;
}

robot_run_variant_e robot_mode_variant(void)
{
    return s_variant;
}

uint8_t robot_mode_is_single_task(RobotTaskModule module)
{
    return (uint8_t)(s_mode == ROBOT_RUN_MODE_SINGLE_TASK && s_singleTask == module);
}

uint8_t InsSnapshotRead(InsSnapshot *out)
{
    if (out == NULL || s_imuAvailable == 0u)
    {
        return 0u;
    }
    *out = s_imuSnapshot;
    return 1u;
}

const MotorInst *MotorInstFindByName(const char *name)
{
    if (name != NULL && strcmp(name, "motor.left") == 0)
    {
        return &s_motor[Motor0];
    }
    if (name != NULL && strcmp(name, "motor.left_alias") == 0)
    {
        return &s_motor[Motor0];
    }
    if (name != NULL && strcmp(name, "motor.right") == 0)
    {
        return &s_motor[Motor1];
    }
    if (name != NULL && strcmp(name, "motor.gimbal_alias_left") == 0)
    {
        return &s_motor[Motor0];
    }
    if (name != NULL && strcmp(name, "motor.gimbal_pitch") == 0)
    {
        return &s_motor[Motor2];
    }
    if (name != NULL && strcmp(name, "motor.disabled") == 0)
    {
        return &s_motor[Motor3];
    }
    if (name != NULL && strcmp(name, "motor.no_route") == 0)
    {
        return &s_motor[Motor4];
    }
    return NULL;
}

MotorId MotorInstId(const MotorInst *inst)
{
    return (inst != NULL) ? inst->actuator_id : MotorCount;
}

uint8_t MotorInstEnabled(const MotorInst *inst)
{
    return (inst != NULL) ? inst->enabled : 0u;
}

const MotorRoute *MotorRouteFindByMotor(MotorId id)
{
    if (id == Motor0 || id == Motor1 || id == Motor2)
    {
        return &s_route;
    }
    return NULL;
}

uint8_t LowStateGetMotor(MotorId id, MotorState *out)
{
    if (id >= MotorCount || out == NULL)
    {
        return 0u;
    }
    *out = s_feedback[id];
    return 1u;
}

uint8_t LowCmdInhibitManyFrom(const MotorId *ids, uint8_t count, uint16_t writer)
{
    (void)ids;
    s_inhibited = (uint8_t)(count == 2u && writer == (uint16_t)LOWCMD_WRITER_SAFETY);
    return s_inhibited;
}

uint8_t LowCmdRecoverSafetyInhibitManyWithPermit(const MotorId *ids,
                                                  uint8_t count,
                                                  const ControlOutputPermit *permit)
{
    (void)ids;
    if (count != 2u || ControlMgrOutputPermitValid(permit, 3u) == 0u)
    {
        return 0u;
    }
    s_inhibited = 0u;
    return 1u;
}

uint8_t MotorInstSetIdsWithPermit(const MotorId *ids,
                                  const MotorCmd *cmds,
                                  uint8_t count,
                                  const ControlOutputPermit *permit)
{
    (void)ids;
    (void)cmds;
    if (s_revokeOnCommit != 0u)
    {
        s_revokeOnCommit = 0u;
        (void)ControlMgrStop(ControlDomainChassis, ControlReasonEmergencyStop);
    }
    if (count != 2u || ControlMgrOutputPermitValid(permit, 3u) == 0u)
    {
        return 0u;
    }
    s_commitCount++;
    s_commitAxisCount = count;
    return 1u;
}

static ControlAlgorithmResult TestConfigure(void *state,
                                            const ControlParam *params,
                                            uint8_t paramCount)
{
    (void)params;
    (void)paramCount;
    *(uint32_t *)state = 0u;
    return ControlAlgorithmResultOk;
}

static void TestReset(void *state)
{
    *(uint32_t *)state = 0u;
}

static ControlAlgorithmResult TestStep(void *state,
                                       const ControlAlgorithmInput *input,
                                       ControlAlgorithmOutput *output)
{
    (void)state;
    s_stepCount++;
    s_lastInput = *input;
    memset(output, 0, sizeof(*output));
    output->outputCount = input->outputCount;
    output->motor[0].mode = (uint8_t)ControlMotorModeCurrent;
    output->motor[1].mode = s_nanOutput ? (uint8_t)ControlMotorModeSpeed :
                                              (uint8_t)ControlMotorModeCurrent;
    output->motor[0].current = 100;
    output->motor[1].current = -100;
    output->motor[1].dq = s_nanOutput ? NAN : 0.0f;
    return ControlAlgorithmResultOk;
}

static const ControlAlgorithm s_algorithm = {
    .name = "controller.test_differential",
    .domain = ControlAlgorithmDomainChassis,
    .stateBytes = sizeof(uint32_t),
    .outputCount = 2u,
    .configure = TestConfigure,
    .reset = TestReset,
    .step = TestStep,
};

static const ControlAlgorithm s_gimbalAlgorithm = {
    .name = "controller.test_gimbal",
    .domain = ControlAlgorithmDomainGimbal,
    .stateBytes = sizeof(uint32_t),
    .outputCount = 2u,
    .configure = TestConfigure,
    .reset = TestReset,
    .step = TestStep,
};

static int TestStart(void)
{
    static const char *const outputs[] = {"motor.left", "motor.right"};
    const ControlModuleSpec spec = {
        .algorithm = &s_algorithm,
        .outputNames = outputs,
        .outputCount = 2u,
        .periodMs = 2u,
    };
    ControlController controller;

    ControlMgrReset();
    if (ControlRuntimeConfigure(&spec, 1u) != ControlResultOk)
    {
        return 0;
    }
    controller = *ControlRuntimeController(0u);
    controller.actuator_mask = 3u;
    if (ControlMgrRegister(&controller) != ControlResultOk ||
        ControlRuntimeStartDefaults() != ControlResultOk)
    {
        return 0;
    }
    return 1;
}

static int TestStartGimbal(void)
{
    static const char *const outputs[] = {"motor.left", "motor.right"};
    const ControlModuleSpec spec = {
        .algorithm = &s_gimbalAlgorithm,
        .outputNames = outputs,
        .outputCount = 2u,
        .periodMs = 2u,
    };
    ControlController controller;

    ControlMgrReset();
    if (ControlRuntimeConfigure(&spec, 1u) != ControlResultOk)
    {
        return 0;
    }
    controller = *ControlRuntimeController(0u);
    controller.actuator_mask = 3u;
    if (ControlMgrRegister(&controller) != ControlResultOk ||
        ControlRuntimeStartDefaults() != ControlResultOk)
    {
        return 0;
    }
    return 1;
}

static int TestStartWithImu(void)
{
    static const char *const outputs[] = {"motor.left", "motor.right"};
    const ControlModuleSpec spec = {
        .algorithm = &s_algorithm,
        .outputNames = outputs,
        .outputCount = 2u,
        .periodMs = 2u,
        .requireImu = 1u,
    };
    ControlController controller;

    ControlMgrReset();
    if (ControlRuntimeConfigure(&spec, 1u) != ControlResultOk)
    {
        return 0;
    }
    controller = *ControlRuntimeController(0u);
    controller.actuator_mask = 3u;
    if (ControlMgrRegister(&controller) != ControlResultOk ||
        ControlRuntimeStartDefaults() != ControlResultOk)
    {
        return 0;
    }
    return 1;
}

static void TestStateReset(void)
{
    memset(&s_manual, 0, sizeof(s_manual));
    memset(s_feedback, 0, sizeof(s_feedback));
    memset(&s_imuSnapshot, 0, sizeof(s_imuSnapshot));
    memset(&s_lastInput, 0, sizeof(s_lastInput));
    s_now = 100u;
    s_manual.online = 1u;
    s_manual.dataValid = 1u;
    s_manual.sourceTimeoutMs = 100u;
    s_manual.semantics.ChassisSafePos = 1u;
    s_manual.semantics.GimbalSafePos = 1u;
    s_manual.control.sw[INPUT_SW_GIMBAL_MODE] = 2u;
    s_manual.control.sw[INPUT_SW_CHASSIS_MODE] = 2u;
    s_feedback[Motor0].online = 1u;
    s_feedback[Motor1].online = 1u;
    s_feedback[Motor0].lastRxTick = s_now;
    s_feedback[Motor1].lastRxTick = s_now;
    s_mode = ROBOT_RUN_MODE_FULL;
    s_variant = ROBOT_RUN_VARIANT_NORMAL;
    s_singleTask = 0;
    s_lifecycleAllowed = 1u;
    s_revokeOnCommit = 0u;
    s_inhibited = 0u;
    s_stepCount = 0u;
    s_commitCount = 0u;
    s_commitAxisCount = 0u;
    s_nanOutput = 0u;
    s_imuAvailable = 1u;
    s_imuSnapshot.quat[0] = 1.0f;
}

#define TEST_CHECK(condition)                                                        \
    do                                                                                \
    {                                                                                 \
        if (!(condition))                                                             \
        {                                                                             \
            printf("FAIL line %d: %s\n", __LINE__, #condition);                     \
            return 1;                                                                 \
        }                                                                             \
    } while (0)

int main(void)
{
    static const char *const duplicateOutputs[] = {"motor.left", "motor.left_alias"};
    static const char *const disabledOutputs[] = {"motor.left", "motor.disabled"};
    static const char *const noRouteOutputs[] = {"motor.left", "motor.no_route"};
    ControlModuleSpec duplicateSpec = {
        .algorithm = &s_algorithm,
        .outputNames = duplicateOutputs,
        .outputCount = 2u,
        .periodMs = 2u,
    };
    ControlModuleSpec disabledSpec = {
        .algorithm = &s_algorithm,
        .outputNames = disabledOutputs,
        .outputCount = 2u,
        .periodMs = 2u,
    };
    ControlModuleSpec noRouteSpec = {
        .algorithm = &s_algorithm,
        .outputNames = noRouteOutputs,
        .outputCount = 2u,
        .periodMs = 2u,
    };
    static const char *const chassisOutputs[] = {"motor.left", "motor.right"};
    static const char *const gimbalOutputs[] = {"motor.gimbal_alias_left", "motor.gimbal_pitch"};
    const ControlModuleSpec crossDomainSpecs[] = {
        {
            .algorithm = &s_algorithm,
            .outputNames = chassisOutputs,
            .outputCount = 2u,
            .periodMs = 2u,
        },
        {
            .algorithm = &s_gimbalAlgorithm,
            .outputNames = gimbalOutputs,
            .outputCount = 2u,
            .periodMs = 2u,
        },
    };

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_inhibited == 0u && s_stepCount == 0u && s_commitCount == 0u);
    s_now += 2u;
    s_feedback[Motor0].lastRxTick = s_now;
    s_feedback[Motor1].lastRxTick = s_now;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 1u && s_commitCount == 1u && s_commitAxisCount == 2u);

    s_manual.online = 0u;
    s_now += 2u;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 1u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_now += CONTROL_RUNTIME_FEEDBACK_TIMEOUT_MS + 1u;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStartGimbal() != 0);
    s_variant = ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainGimbal) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_feedback[Motor0].q = NAN;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    s_mode = ROBOT_RUN_MODE_SINGLE_MOTOR;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_now += 2u;
    s_feedback[Motor0].lastRxTick = s_now;
    s_feedback[Motor1].lastRxTick = s_now;
    s_nanOutput = 1u;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultCallbackFailed);
    TEST_CHECK(s_commitCount == 0u && s_inhibited != 0u);

    TestStateReset();
    s_imuSnapshot.quat[0] = 0.9f;
    s_imuSnapshot.quat[1] = 0.1f;
    s_imuSnapshot.gyro[2] = 1.5f;
    s_imuSnapshot.angle[0] = 0.4f;
    s_imuSnapshot.angle[1] = -0.2f;
    s_imuSnapshot.angle[2] = 0.3f;
    TEST_CHECK(TestStartWithImu() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_now += 2u;
    s_feedback[Motor0].lastRxTick = s_now;
    s_feedback[Motor1].lastRxTick = s_now;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 1u && s_lastInput.imu.valid != 0u &&
               s_lastInput.imu.quaternionW == 0.9f &&
               s_lastInput.imu.angularVelocityZRadPerSec == 1.5f &&
               s_lastInput.imu.yawRad == 0.4f && s_lastInput.imu.rollRad == -0.2f &&
               s_lastInput.imu.pitchRad == 0.3f);

    TestStateReset();
    TEST_CHECK(TestStartWithImu() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_imuSnapshot.age_ms = CONTROL_RUNTIME_IMU_TIMEOUT_MS + 1u;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStartWithImu() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_imuSnapshot.gyro[0] = NAN;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    TEST_CHECK(s_stepCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(TestStart() != 0);
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultOk);
    s_now += 2u;
    s_feedback[Motor0].lastRxTick = s_now;
    s_feedback[Motor1].lastRxTick = s_now;
    s_revokeOnCommit = 1u;
    TEST_CHECK(ControlRuntimeRunDomain(ControlDomainChassis) == ControlResultCallbackFailed);
    TEST_CHECK(s_commitCount == 0u && s_inhibited != 0u);

    TestStateReset();
    TEST_CHECK(ControlRuntimeConfigure(&duplicateSpec, 1u) == ControlResultDuplicate);
    TEST_CHECK(ControlRuntimeCount() == 0u);
    TEST_CHECK(ControlRuntimeConfigure(&disabledSpec, 1u) == ControlResultNotFound);
    TEST_CHECK(ControlRuntimeCount() == 0u);
    TEST_CHECK(ControlRuntimeConfigure(&noRouteSpec, 1u) == ControlResultNotFound);
    TEST_CHECK(ControlRuntimeCount() == 0u);
    TEST_CHECK(ControlRuntimeConfigure(crossDomainSpecs, 2u) == ControlResultResourceBusy);
    TEST_CHECK(ControlRuntimeCount() == 0u);

    puts("ControlRuntime regression passed");
    return 0;
}

/* 控制域输入恢复门控回归：旧的持续动作不能跨越失能或换权边界。 */

#include <stdint.h>
#include <stdio.h>

#include "ArmInputPolicy.h"
#include "ChassisInputPolicy.h"
#include "GimbalInputPolicy.h"
#include "ServoInputPolicy.h"

#define TEST_SERVO_Z     (1u << 11)
#define TEST_SERVO_X     (1u << 12)
#define TEST_SERVO_C     (1u << 13)
#define TEST_SERVO_V     (1u << 14)
#define TEST_SHIFT       (1u << 4)
#define TEST_SERVO_MASK  (TEST_SERVO_Z | TEST_SERVO_X | TEST_SERVO_C | TEST_SERVO_V)
#define TEST_ARM_G       (1u << 10)
#define TEST_ARM_B       (1u << 15)
#define TEST_ARM_MASK    (TEST_ARM_G | TEST_SERVO_MASK | TEST_ARM_B)
#define TEST_CHASSIS_W   (1u << 0)

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static int TestArmGate(void)
{
    ArmInputGate gate;

    ArmInputGateInit(&gate);
    if (!TestCheck(ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_CHASSIS_W | TEST_ARM_G) == 0u &&
                   ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_CHASSIS_W | TEST_ARM_G) == 0u,
                   "Arm 上电持续动作键必须保持遮蔽")) return 0;
    if (!TestCheck(ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_CHASSIS_W) == 0u &&
                   ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_CHASSIS_W | TEST_ARM_G) ==
                       (TEST_CHASSIS_W | TEST_ARM_G),
                   "Arm 只需释放本域动作键，其他域按键可以保持")) return 0;

    if (!TestCheck(ArmInputGateApply(&gate,
                                     0u,
                                     TEST_ARM_MASK,
                                     TEST_ARM_B) == 0u &&
                   ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_ARM_B) == 0u,
                   "Arm 掉线、锁定或失去控制权后，持续按键不能直接恢复")) return 0;
    if (!TestCheck(ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_SHIFT) == 0u &&
                   ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_SHIFT | TEST_ARM_B) ==
                       (TEST_SHIFT | TEST_ARM_B),
                   "Arm 重获控制权后必须释放再进入")) return 0;
    if (!TestCheck(ArmInputGateApply(NULL, 1u, TEST_ARM_MASK, 0xffffu) == 0u,
                   "Arm 空门控状态必须保持安全")) return 0;

    ArmInputGateSync(&gate, 0u, 0u);
    if (!TestCheck(ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_ARM_G) == TEST_ARM_G,
                   "Arm 的 0 控制权威代不得误重置已放行状态")) return 0;
    ArmInputGateSync(&gate, 1u, 1u);
    if (!TestCheck(ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_ARM_G) == 0u &&
                   ArmInputGateApply(&gate, 1u, TEST_ARM_MASK, 0u) == 0u &&
                   ArmInputGateApply(&gate,
                                     1u,
                                     TEST_ARM_MASK,
                                     TEST_ARM_G) == TEST_ARM_G,
                   "Arm 换权后持续动作必须遮蔽，真实释放后才能重新进入")) return 0;
    ArmInputGateSync(&gate, 1u, 2u);
    if (!TestCheck(ArmInputGateApply(&gate, 1u, TEST_ARM_MASK, TEST_ARM_G) == 0u,
                   "Arm 输入解释变化后也必须重新等待动作键释放")) return 0;

    return 1;
}

static int TestServoGate(void)
{
    ServoInputGate gate;

    ServoInputGateInit(&gate);
    if (!TestCheck(ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SHIFT | TEST_SERVO_Z) == 0u &&
                   ServoInputGateReady(&gate) == 0u,
                   "Servo 上电持续动作键必须保持遮蔽")) return 0;
    if (!TestCheck(ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SHIFT) == 0u &&
                   ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SHIFT | TEST_SERVO_Z) ==
                       (TEST_SHIFT | TEST_SERVO_Z) &&
                   ServoInputGateReady(&gate) != 0u,
                   "Servo 只需释放动作键，修饰键可以保持")) return 0;

    if (!TestCheck(ServoInputGateApply(&gate,
                                       0u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == 0u &&
                   ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == 0u &&
                   ServoInputGateReady(&gate) == 0u,
                   "Servo 掉线或整机锁定后，持续动作键不能直接恢复")) return 0;
    if (!TestCheck(ServoInputGateApply(&gate, 1u, TEST_SERVO_MASK, 0u) == 0u &&
                   ServoInputGateReady(&gate) != 0u &&
                   ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == TEST_SERVO_X,
                   "Servo 重新允许后必须释放再进入")) return 0;
    if (!TestCheck(ServoInputGateApply(NULL,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_V) == 0u,
                   "Servo 空门控状态必须保持安全")) return 0;
    if (!TestCheck(ServoInputGateReady(NULL) == 0u,
                   "Servo 空门控状态不能允许恢复旧输出")) return 0;

    ServoInputGateSync(&gate, 0u, 0u);
    if (!TestCheck(ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == TEST_SERVO_X,
                   "Servo 的 0 控制权威代不得误重置已放行状态")) return 0;
    ServoInputGateSync(&gate, 7u, 1u);
    if (!TestCheck(ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == 0u &&
                   ServoInputGateReady(&gate) == 0u &&
                   ServoInputGateApply(&gate, 1u, TEST_SERVO_MASK, 0u) == 0u &&
                   ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == TEST_SERVO_X,
                   "Servo 换权后必须先释放动作键再恢复输出")) return 0;
    ServoInputGateSync(&gate, 7u, 2u);
    if (!TestCheck(ServoInputGateApply(&gate,
                                       1u,
                                       TEST_SERVO_MASK,
                                       TEST_SERVO_X) == 0u,
                   "Servo 输入解释变化后也必须重新等待动作键释放")) return 0;

    return 1;
}

static int TestChassisGate(void)
{
    ChassisInputGate gate = {0};

    if (!TestCheck(ChassisInputGateApply(&gate, 1u, 1u, 1u, 1u, 1u) == 0u &&
                       ChassisInputGateApply(&gate, 1u, 1u, 1u, 1u, 1u) == 0u,
                   "Chassis 换权后持续 spin/模式键不能直接进入")) return 0;
    if (!TestCheck(ChassisInputGateApply(&gate, 1u, 1u, 1u, 0u, 0u) == 0u &&
                       ChassisInputGateApply(&gate, 1u, 1u, 1u, 1u, 0u) != 0u,
                   "Chassis 必须先离开 spin 且释放模式键，再允许新的动作")) return 0;
    if (!TestCheck(ChassisInputGateApply(&gate, 1u, 1u, 0u, 0u, 0u) == 0u &&
                       ChassisInputGateApply(&gate, 1u, 1u, 0u, 1u, 1u) == 0u &&
                       ChassisInputGateApply(&gate, 1u, 1u, 1u, 1u, 1u) == 0u,
                   "Chassis 锁定期间的释放或再按不得完成重新武装")) return 0;
    if (!TestCheck(ChassisInputGateApply(&gate, 1u, 1u, 1u, 0u, 0u) == 0u &&
                       ChassisInputGateApply(&gate, 1u, 1u, 1u, 1u, 0u) != 0u,
                   "Chassis 解锁后必须重新看到释放帧")) return 0;
    return TestCheck(ChassisInputGateApply(&gate, 1u, 2u, 1u, 1u, 0u) == 0u,
                     "Chassis 输入解释变化必须重新门控模式动作");
}

static int TestGimbalGate(void)
{
    GimbalInputGate gate = {0};
    uint8_t effective = 1u;

    if (!TestCheck(GimbalInputGateApplyTurn(&gate, 1u, 1u, 1u, 1u, &effective) != 0u &&
                       effective == 0u,
                   "Gimbal 换权时必须取消旧掉头并遮蔽持续按键")) return 0;
    if (!TestCheck(GimbalInputGateApplyTurn(&gate, 1u, 1u, 1u, 0u, &effective) == 0u &&
                       effective == 0u &&
                       GimbalInputGateApplyTurn(&gate, 1u, 1u, 1u, 1u, &effective) == 0u &&
                       effective != 0u,
                   "Gimbal 释放后再次按下才能触发掉头")) return 0;
    GimbalInputGateBlock(&gate);
    if (!TestCheck(GimbalInputGateApplyTurn(&gate, 1u, 1u, 0u, 0u, &effective) != 0u &&
                       GimbalInputGateApplyTurn(&gate, 1u, 1u, 1u, 1u, &effective) == 0u &&
                       effective == 0u,
                   "Gimbal 锁定期间释放不能替代解锁后的真实释放")) return 0;
    return TestCheck(GimbalInputGateApplyTurn(&gate, 2u, 1u, 1u, 1u, &effective) != 0u &&
                         effective == 0u,
                     "Gimbal 控制来源变化必须取消正在执行的掉头");
}

int main(void)
{
    if (TestArmGate() == 0 || TestServoGate() == 0 ||
        TestChassisGate() == 0 || TestGimbalGate() == 0)
    {
        return 1;
    }

    (void)puts("PASS: 服务任务输入恢复门控回归");
    return 0;
}

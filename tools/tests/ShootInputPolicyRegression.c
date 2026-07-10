/* Shoot 输入重启锁回归：验证持续拨杆/按键经过安全帧后不会被当成新指令。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ShootInputPolicy.h"

enum
{
    TEST_SWITCH_STOP = 1u,
    TEST_SWITCH_FIRE = 2u,
    TEST_SWITCH_READY = 3u,
};

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }

    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static uint16_t TestSwitch(ShootInputGateState *state,
                           uint16_t rawSwitch,
                           uint8_t manualOnline)
{
    return ShootInputGateSwitch(state,
                                rawSwitch,
                                manualOnline,
                                TEST_SWITCH_STOP,
                                TEST_SWITCH_READY,
                                TEST_SWITCH_FIRE);
}

int main(void)
{
    ShootInputGateState state;
    uint8_t pressLeft;
    uint8_t pressRight;

    (void)memset(&state, 0, sizeof(state));
    ShootInputGateReset(&state, TEST_SWITCH_STOP);
    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY,
                   "上电持续处于开火档时必须保持预备，不能自动形成进入边沿")) return 1;
    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_STOP, 1u) == TEST_SWITCH_STOP &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_FIRE &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_FIRE,
                   "离开再进入开火档后才允许开火，并可保持该档")) return 1;

    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_FIRE, 0u) == TEST_SWITCH_STOP &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY,
                   "掉线必须复位拨杆门控，恢复时持续开火档不能直接开火")) return 1;
    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_STOP, 1u) == TEST_SWITCH_STOP &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_FIRE,
                   "掉线恢复后仍可通过真实离开和重新进入完成解锁")) return 1;

    if (!TestCheck(ShootInputGateSyncSemantics(&state, 1u, TEST_SWITCH_STOP) != 0u &&
                   ShootInputGateSyncSemantics(&state, 1u, TEST_SWITCH_STOP) == 0u,
                   "首次语义代必须重置一次，同代重复帧不得反复重置")) return 1;
    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY &&
                   TestSwitch(&state, TEST_SWITCH_STOP, 1u) == TEST_SWITCH_STOP &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_FIRE,
                   "语义代建立后仍须离开再进入开火档")) return 1;
    if (!TestCheck(ShootInputGateSyncSemantics(&state, 2u, TEST_SWITCH_FIRE) != 0u &&
                   ShootInputGateSwitch(&state,
                                        TEST_SWITCH_STOP,
                                        1u,
                                        TEST_SWITCH_FIRE,
                                        TEST_SWITCH_READY,
                                        TEST_SWITCH_STOP) == TEST_SWITCH_READY,
                   "热改拨杆语义后，旧物理档位不能被当成新的开火边沿")) return 1;
    if (!TestCheck(ShootInputGateSwitch(&state,
                                        TEST_SWITCH_FIRE,
                                        1u,
                                        TEST_SWITCH_FIRE,
                                        TEST_SWITCH_READY,
                                        TEST_SWITCH_STOP) == TEST_SWITCH_FIRE &&
                   ShootInputGateSwitch(&state,
                                        TEST_SWITCH_STOP,
                                        1u,
                                        TEST_SWITCH_FIRE,
                                        TEST_SWITCH_READY,
                                        TEST_SWITCH_STOP) == TEST_SWITCH_STOP,
                   "新语义下真实离开并重新进入后才能恢复开火")) return 1;

    if (!TestCheck(ShootInputGateSyncAction(&state, 10u, TEST_SWITCH_STOP) != 0u &&
                   ShootInputGateSyncAction(&state, 10u, TEST_SWITCH_STOP) == 0u &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY,
                   "动作贡献代变化必须清除旧开火边沿，同代帧不得反复重置")) return 1;
    if (!TestCheck(TestSwitch(&state, TEST_SWITCH_STOP, 1u) == TEST_SWITCH_STOP &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_FIRE,
                   "换权后必须真实离开开火档再重新进入")) return 1;
    if (!TestCheck(ShootInputGateSyncAction(&state, 11u, TEST_SWITCH_STOP) != 0u &&
                   TestSwitch(&state, TEST_SWITCH_FIRE, 1u) == TEST_SWITCH_READY,
                   "连续第二次换权也必须再次建立开火重启锁")) return 1;

    ShootInputGateBlockMouse(&state);
    pressLeft = 1u;
    pressRight = 0u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 0u,
                   "上电持续按住鼠标必须被遮蔽")) return 1;
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 1u,
                   "已观察到释放的一侧可以重新按下，另一侧仍须等待释放")) return 1;
    pressLeft = 0u;
    pressRight = 0u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    pressLeft = 1u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 1u,
                   "持续按住的一侧释放后，下一次真实按下应恢复有效")) return 1;

    ShootInputGateSyncSafeMouse(&state, 1u, 1u, 0u);
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 1u,
                   "安全帧只应锁住当时仍按住的按键")) return 1;
    ShootInputGateSyncSafeMouse(&state, 0u, 0u, 0u);
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyMouse(&state, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 0u,
                   "输入不可确认时必须保守锁住两侧鼠标")) return 1;

    /* 普通 Runtime 的离线帧同样不能把合成零值误当成真实释放。 */
    pressLeft = 0u;
    pressRight = 0u;
    ShootInputGateApplyFrameMouse(&state, 0u, &pressLeft, &pressRight);
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 0u,
                   "普通离线帧后持续按住的鼠标必须等待真实释放")) return 1;
    pressLeft = 0u;
    pressRight = 0u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    pressLeft = 1u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 1u,
                   "普通离线恢复后观察到真实释放，下一次按下才可生效")) return 1;

    ShootInputGateSyncAction(&state, 12u, TEST_SWITCH_STOP);
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 0u && pressRight == 0u,
                   "换权时持续按住的左右鼠标都必须被遮蔽")) return 1;
    pressLeft = 0u;
    pressRight = 1u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    pressLeft = 1u;
    pressRight = 1u;
    ShootInputGateApplyFrameMouse(&state, 1u, &pressLeft, &pressRight);
    if (!TestCheck(pressLeft == 1u && pressRight == 0u,
                   "换权后左右鼠标必须分别真实释放再恢复")) return 1;

    (void)puts("PASS: Shoot 输入安全恢复门控回归");
    return 0;
}

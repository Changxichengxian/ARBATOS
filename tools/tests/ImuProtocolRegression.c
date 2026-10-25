/*
 * IMU 客户端协议与校准坐标格式主机回归。
 * 直接包含生产头文件，验证命令边界和 v1/v2/v3 方向兼容。
 */

#include <stdint.h>
#include <stdio.h>

#include "ImuFrame.h"
#include "ImuServiceProtocol.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0) return 1;
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

static int TestNear(float actual, float expected)
{
    const float delta = actual - expected;
    return delta > -1.0e-6f && delta < 1.0e-6f;
}

static int TestVector(const float actual[3], const float expected[3], const char *message)
{
    return TestCheck(TestNear(actual[0], expected[0]) &&
                         TestNear(actual[1], expected[1]) &&
                         TestNear(actual[2], expected[2]),
                     message);
}

static int TestCommandParsing(void)
{
    uint32_t id = 0u;

    if (!TestCheck(ImuCommandParse("!IMU2,watch,1", &id) == ImuCmdWatch && id == 1u,
                   "watch 命令解析失败")) return 0;
    if (!TestCheck(ImuCommandParse("!IMU2,stop,42", &id) == ImuCmdStop && id == 42u,
                   "stop 命令解析失败")) return 0;
    if (!TestCheck(ImuCommandParse("!IMU2,calibrate,4294967295", &id) == ImuCmdCalibrate &&
                       id == UINT32_MAX,
                   "最大请求编号解析失败")) return 0;
    if (!TestCheck(ImuCommandParse("!IMU2,cancel,7", &id) == ImuCmdCancel && id == 7u,
                   "cancel 命令解析失败")) return 0;

    const char *invalid[] = {
        "",
        "IMU2,watch,1",
        "!IMU1,watch,1",
        "!IMU2,watch,0",
        "!IMU2,watch,-1",
        "!IMU2,watch,+1",
        "!IMU2,watch,01x",
        "!IMU2,watch,4294967296",
        "!IMU2,calibrate,1,extra",
        "!IMU2,unknown,1",
    };
    for (unsigned i = 0u; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        id = 0x5a5a5a5au;
        if (!TestCheck(ImuCommandParse(invalid[i], &id) == ImuCmdNone,
                       "非法命令被协议解析器接受")) return 0;
        if (!TestCheck(id == 0x5a5a5a5au, "拒绝命令时不应改写请求编号")) return 0;
    }
    return 1;
}

static int TestFrameRoundTrip(void)
{
    const float raw[3] = {0.0125f, -0.03125f, 0.004f};
    float body[3];
    float restored[3];

    ImuFrameRotate(body, raw);
    ImuFrameUnrotate(restored, body);
    return TestVector(restored, raw, "任意正交安装方向的逆变换未还原原始传感器坐标");
}

static int TestStoredVersions(void)
{
    const float raw[3] = {0.0125f, -0.03125f, 0.004f};
    const float legacy[3] = {raw[1], -raw[0], raw[2]};
    float expected[3];
    float actual[3];

    ImuFrameRotate(expected, raw);
    if (!TestCheck(ImuFrameBiasConvert(actual, legacy, 1u) != 0,
                   "v1 校准记录被拒绝") ||
        !TestVector(actual, expected, "v1 旧方向记录未先还原 raw 再应用当前方向")) return 0;
    if (!TestCheck(ImuFrameBiasConvert(actual, raw, 2u) != 0,
                   "v2 校准记录被拒绝") ||
        !TestVector(actual, expected, "v2 raw 记录应用当前方向错误")) return 0;
    if (!TestCheck(ImuFrameBiasConvert(actual, raw, 3u) != 0,
                   "v3 校准记录被拒绝") ||
        !TestVector(actual, expected, "v3 raw 记录应用当前方向错误")) return 0;

    actual[0] = 11.0f;
    actual[1] = 12.0f;
    actual[2] = 13.0f;
    if (!TestCheck(ImuFrameBiasConvert(actual, raw, 0u) == 0,
                   "未知旧版本不应被接受")) return 0;
    return TestCheck(actual[0] == 11.0f && actual[1] == 12.0f && actual[2] == 13.0f,
                     "拒绝未知版本时不应污染输出");
}

int main(void)
{
    if (!TestCommandParsing()) return 1;
    if (!TestFrameRoundTrip()) return 1;
    if (!TestStoredVersions()) return 1;
    (void)printf("PASS: IMU protocol and frame regression (mount %d)\n", TEST_IMU_MOUNT);
    return 0;
}

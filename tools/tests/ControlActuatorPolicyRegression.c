/* 执行器所有权纯策略回归：不依赖运行配置和 RTOS。 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "ControlActuatorPolicy.h"

static int TestCheck(int condition, const char *message)
{
    if (condition != 0)
    {
        return 1;
    }
    (void)fprintf(stderr, "FAIL: %s\n", message);
    return 0;
}

#define TEST_CHECK(condition, message) \
    do { if (!TestCheck((condition), (message))) return 0; } while (0)

static int TestSourceIdPolicy(void)
{
    MotorId resolved = MotorCount;

    TEST_CHECK(ControlActuatorResolveSourceId(0u, Motor4, &resolved) != 0u &&
               resolved == Motor0,
               "Motor0 必须是合法显式 source_id");
    TEST_CHECK(ControlActuatorResolveSourceId(17u, Motor4, &resolved) != 0u &&
               resolved == Motor17,
               "Motor17 必须是合法显式 source_id");
    TEST_CHECK(ControlActuatorResolveSourceId(UINT16_MAX, Motor4, &resolved) != 0u &&
               resolved == Motor4,
               "只有 0xFFFF 可以继承角色默认执行器");
    TEST_CHECK(ControlActuatorResolveSourceId(18u, Motor4, &resolved) == 0u &&
               resolved == MotorCount,
               "MotorCount 不能借默认值冒充真实所有权");
    TEST_CHECK(ControlActuatorResolveSourceId(0xFFFEu, Motor4, &resolved) == 0u &&
               resolved == MotorCount,
               "0xFFFE 不能借默认值冒充真实所有权");
    TEST_CHECK(ControlActuatorResolveSourceId(UINT16_MAX, MotorCount, &resolved) == 0u &&
               resolved == MotorCount,
               "无合法角色默认值时必须保持未解析");
    TEST_CHECK(ControlActuatorResolveSourceId(0u, Motor4, NULL) == 0u,
               "source_id 解析必须拒绝空输出");
    return 1;
}

static int TestDeclarationPolicy(void)
{
    const uint32_t motor0 = ((uint32_t)1u << 0u);
    const uint32_t motor17 = ((uint32_t)1u << 17u);
    ControlActuatorAudit audit;
    uint32_t mask = 0u;

    (void)memset(&audit, 0, sizeof(audit));
    ControlActuatorAuditDeclare(&audit, &mask, 17u, 0u);
    ControlActuatorAuditDeclare(&audit, &mask, 0u, 1u);
    ControlActuatorAuditDeclare(&audit, &mask, 17u, 1u);
    ControlActuatorAuditDeclare(&audit, &mask, 18u, 0u);

    TEST_CHECK(mask == (motor0 | motor17),
               "合法但缺实例的 Motor17 仍必须进入声明掩码");
    TEST_CHECK(audit.unresolvedOutputCount == 1u,
               "缺实例必须单独累计未解析，非法 ID 不得重复计数");
    TEST_CHECK(audit.invalidIdCount == 1u,
               "越界执行器必须进入非法 ID 诊断");
    TEST_CHECK(audit.duplicateMask == motor17,
               "重复声明必须保留对应物理执行器位");
    return 1;
}

static int TestRoutePolicy(void)
{
    ControlActuatorRouteView route;

    (void)memset(&route, 0, sizeof(route));
    route.enabled = 1u;
    route.transport = (uint8_t)MotorTransportCAN;
    route.protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP;
    route.isRmGroup = 1u;
    route.bus = 1u;
    route.canId = 0x201u;
    route.canBusCount = 2u;
    route.rs485PortCount = 2u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "RM 路由必须对应真实 0x201..0x208 分组帧");
    route.canId = 0x208u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "RM 最大合法电机帧应可发送");
    route.canId = 0x209u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "RM 分组外标准帧不能算可发送");
    route.canId = 0x201u;
    route.isRmGroup = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "RM 协议和分组标记不一致时必须拒绝");

    route.protocol = (uint8_t)MOTOR_PROTOCOL_DM_EXT_V2;
    route.canId = 0x141u;
    route.bus = 2u;
    route.hasLimits = 1u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "带型号限幅的 CAN MIT 路由应可发送");
    route.hasLimits = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "缺型号限幅的 CAN MIT 路由最终会被发送层跳过");
    route.hasLimits = 1u;
    route.bus = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "CAN 总线必须落在板级 1..N 能力内");
    route.bus = 3u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "超过板级 CAN 总线数的路由必须拒绝");
    route.bus = 1u;
    route.canId = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "CAN 0 号帧不能算可发送");
    route.canId = 0x800u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "扩展范围 ID 不能冒充 CAN 标准帧");

    (void)memset(&route, 0, sizeof(route));
    route.enabled = 1u;
    route.transport = (uint8_t)MotorTransportRS485;
    route.protocol = (uint8_t)MOTOR_PROTOCOL_UNITREE_RS485;
    route.bus = 0u;
    route.deviceId = 1u;
    route.hasLimits = 1u;
    route.canBusCount = 2u;
    route.rs485PortCount = 2u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "Unitree 路由不依赖 CAN ID");
    route.bus = 1u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "第二个 RS485 端口应可发送");
    route.bus = 2u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "RS485 端口必须落在板级 0..N-1 能力内");
    route.bus = 0u;
    route.rs485PortCount = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "没有 RS485 端口的板不能把误配路由算作可发送");
    route.rs485PortCount = 2u;
    route.hasLimits = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "Unitree 缺限幅时只能发 BRAKE，不能算活动命令可路由");
    route.hasLimits = 1u;
    route.deviceId = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "Unitree 零设备号不能算活动路由");

    route.protocol = (uint8_t)MOTOR_PROTOCOL_N6014B_RS485;
    route.deviceId = 15u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "N6014B 最大合法设备号应可发送");
    route.deviceId = 16u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "N6014B 设备号必须限制在 0..15");
    route.deviceId = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) != 0u,
               "显式反馈设备号 0 对 N6014B 是合法值");
    route.protocol = (uint8_t)MOTOR_PROTOCOL_RM_GROUP;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "RS485 只允许发送层真实支持的协议");

    route.transport = (uint8_t)MotorTransportNone;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "未知传输类型不能算可发送");
    route.transport = 0xFEu;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u,
               "越界传输类型不能算可发送");
    route.enabled = 0u;
    TEST_CHECK(ControlActuatorRouteRoutable(&route) == 0u &&
               ControlActuatorRouteRoutable(NULL) == 0u,
               "禁用或空路由不能算可发送");
    return 1;
}

int main(void)
{
    if (!TestSourceIdPolicy()) return 1;
    if (!TestDeclarationPolicy()) return 1;
    if (!TestRoutePolicy()) return 1;
    (void)puts("PASS: Control actuator policy regression");
    return 0;
}

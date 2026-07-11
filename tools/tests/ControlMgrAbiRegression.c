/* 调试器可见结构的布局回归；ARM32 数值是现有 Watch ABI。 */

#include <stddef.h>
#include <stdint.h>

#include "Watch.h"

#define ABI_ASSERT(condition_, name_) \
    typedef char abi_assert_##name_[((condition_) != 0) ? 1 : -1]

ABI_ASSERT(sizeof(ControlMgrDiag) == 40u, control_mgr_diag_size);
ABI_ASSERT(sizeof(ControlActuatorAudit) == 12u, control_actuator_audit_size);
ABI_ASSERT(sizeof(ControlActuatorDiag) == 32u, control_actuator_diag_size);

#if UINTPTR_MAX == UINT32_MAX
/* ControlStatus 含编译器枚举，不作为 Watch 原始块 ABI；这里只钉固定宽度的观察结构。 */
ABI_ASSERT(sizeof(WatchRuntimeDomain) == 28u, arm32_runtime_domain_size);
ABI_ASSERT(sizeof(WatchRuntimeController) == 16u, arm32_runtime_controller_size);
#else
ABI_ASSERT(sizeof(ControlStatus) == 56u, host_control_status_size);
ABI_ASSERT(offsetof(ControlStatus, update_count) == 44u, host_update_count_offset);
ABI_ASSERT(offsetof(ControlStatus, transition_count) == 48u, host_transition_count_offset);
ABI_ASSERT(offsetof(ControlStatus, reject_count) == 52u, host_reject_count_offset);
ABI_ASSERT(sizeof(WatchRuntimeDomain) == 32u, host_runtime_domain_size);
ABI_ASSERT(sizeof(WatchRuntimeController) == 24u, host_runtime_controller_size);
#endif

ABI_ASSERT(offsetof(WatchRuntime, control_mgr) ==
               offsetof(WatchRuntime, domain) + sizeof(((WatchRuntime *)0)->domain),
           control_mgr_diag_tail_offset);
ABI_ASSERT(offsetof(WatchRuntime, control_actuator) ==
               offsetof(WatchRuntime, control_mgr) + sizeof(((WatchRuntime *)0)->control_mgr),
           control_actuator_diag_tail_offset);

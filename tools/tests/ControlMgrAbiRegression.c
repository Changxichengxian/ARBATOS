/* 调试器可见结构的布局回归；ARM32 数值是现有 Watch ABI。 */

#include <stddef.h>
#include <stdint.h>

#include "Watch.h"

#define ABI_ASSERT(condition_, name_) \
    typedef char abi_assert_##name_[((condition_) != 0) ? 1 : -1]

ABI_ASSERT(sizeof(ControlMgrDiag) == 40u, control_mgr_diag_size);
ABI_ASSERT(sizeof(ControlActuatorAudit) == 12u, control_actuator_audit_size);
ABI_ASSERT(sizeof(ControlActuatorDiag) == 32u, control_actuator_diag_size);
ABI_ASSERT(sizeof(ControlOutputStamp) == 12u, control_output_stamp_size);
ABI_ASSERT(offsetof(ControlOutputStamp, authorityEpoch) == 0u, control_output_stamp_epoch_offset);
ABI_ASSERT(offsetof(ControlOutputStamp, cycleSeq) == 4u, control_output_stamp_cycle_offset);
ABI_ASSERT(offsetof(ControlOutputStamp, controllerId) == 8u, control_output_stamp_id_offset);
ABI_ASSERT(offsetof(ControlOutputStamp, domain) == 10u, control_output_stamp_domain_offset);
ABI_ASSERT(offsetof(ControlOutputStamp, valid) == 11u, control_output_stamp_valid_offset);
ABI_ASSERT(sizeof(ControlOutputPermit) == 16u, control_output_permit_size);
ABI_ASSERT(offsetof(ControlOutputPermit, actuatorMask) == 12u,
           control_output_permit_actuator_offset);
ABI_ASSERT(sizeof(LowCmdDiag) == 52u, low_cmd_diag_size);
ABI_ASSERT(offsetof(LowCmdDiag, last_reject_tick) == 32u,
           low_cmd_diag_legacy_tail_offset);
ABI_ASSERT(offsetof(LowCmdDiag, permit_reject_count) == 44u,
           low_cmd_diag_permit_tail_offset);
ABI_ASSERT(offsetof(ControlCtx, outputPermit) ==
               offsetof(ControlCtx, output) + sizeof(((ControlCtx *)0)->output),
           control_context_output_permit_tail_offset);

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

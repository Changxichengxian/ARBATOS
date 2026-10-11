/* SPDX-License-Identifier: Apache-2.0
 * 仅在独立测试库中注入后段故障，不编入仿真运行库或固件。
 */
#include "WbrLearningBridge.c"

#if defined(_WIN32)
#define TEST_EXPORT __declspec(dllexport)
#else
#define TEST_EXPORT __attribute__((visibility("default")))
#endif

TEST_EXPORT void *TestWbrCreate(float *config) { return OwnWbrCreate(config); }
TEST_EXPORT void TestWbrDestroy(void *handle) { OwnWbrDestroy(handle); }
TEST_EXPORT int TestWbrSetGasSpring(void *handle, float force)
{ return OwnWbrSetGasSpring(handle, force); }
TEST_EXPORT int TestWbrSetJointLimits(void *handle, float continuous, float peak)
{ return OwnWbrSetJointLimits(handle, continuous, peak); }
TEST_EXPORT int TestWbrCommandStep(void *handle, const float *sensor,
                                  const float *command, float *out)
{ return OwnWbrCommandStep(handle, sensor, command, out); }
TEST_EXPORT int TestWbrReferenceStep(void *handle, const float *sensor,
                                    const float *command, const float *action, float *out)
{ return WbrLearningReferenceStep(handle, sensor, command, action, out); }
TEST_EXPORT int TestWbrSplitVelocityStep(void *handle, const float *sensor,
                                        const float *command, const float *action, float *out)
{ return WbrLearningSplitVelocityStep(handle, sensor, command, action, out); }

TEST_EXPORT int TestWbrInjectLateFailure(void *handle, float v_ref, float x_ref)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || !isfinite(v_ref) || !isfinite(x_ref)) {
        return 0;
    }
    state->v_ref = v_ref;
    state->x_ref = x_ref;
    /* 参考更新之后，髋虚拟力矩变为非有限数，最终输出检查返回错误并清零。 */
    state->tp0 = NAN;
    return 1;
}

TEST_EXPORT int TestWbrReadTuningState(void *handle, float *out)
{
    OwnWbrState *state = (OwnWbrState *)handle;
    if (state == NULL || out == NULL) {
        return 0;
    }
    out[0] = state->v_ref;
    out[1] = state->x_ref;
    memcpy(out + 2, state->k, sizeof(state->k));
    return 1;
}

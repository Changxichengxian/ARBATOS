"""检查真实小陀螺指令链在整圈旋转中保持平移速度、方向和轮速边界。"""
import argparse
from pathlib import Path
import re
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def extract(path, name):
    source = (ROOT / path).read_text(encoding="utf-8")
    match = re.search(r"(?m)^(?:static\s+)?(?:inline\s+)?[\w*]+\s+" + name
                      + r"\s*\([^;{}]*\)\s*\{", source)
    assert match, name
    pos = source.index("{", match.start()) + 1
    depth = 1
    end = pos
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end] + "\n"


FIXTURE = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <math.h>
typedef float fp32;
#define PI 3.14159265358979323846f
#define MAX_WHEEL_SPEED 4.0f
#define MOTOR_DISTANCE_TO_CENTER .395f
#define CHASSIS_FOLLOW_YAW_STOP_ERR_RAD .01f
#define CHASSIS_FOLLOW_YAW_STOP_WZ_RADPS .01f
#define CHASSIS_CORE_WHEEL_COUNT 4u
#define CHASSIS_CORE_WHEEL_TYPE_XDRIVE 1u
#define CHASSIS_CORE_WHEEL_TYPE_MECANUM 0u
enum { CHASSIS_ZERO_FORCE, CHASSIS_NO_MOVE, CHASSIS_INFANTRY_FOLLOW_GIMBAL_YAW,
       CHASSIS_SWING, CHASSIS_GYRO_SPIN, CHASSIS_GYRO_SPIN_VAR,
       CHASSIS_ENGINEER_FOLLOW_CHASSIS_YAW, CHASSIS_NO_FOLLOW_YAW, CHASSIS_OPEN };
enum { CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW, CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
       CHASSIS_VECTOR_NO_FOLLOW_YAW, CHASSIS_VECTOR_RAW };
enum { EXTERNAL_MOTION_MODE_STOP, EXTERNAL_MOTION_MODE_FOLLOW_GIMBAL,
       EXTERNAL_MOTION_MODE_NO_FOLLOW, EXTERNAL_MOTION_MODE_GYRO_SPIN };
#define EXTERNAL_MOTION_FLAG_YAW_OFFSET_VALID 1u
#define EXTERNAL_MOTION_FLAG_WZ_VALID 2u
#define EXTERNAL_MOTION_FRAME_GIMBAL 1u
#define EXTERNAL_MOTION_FRAME_CHASSIS 2u
typedef struct { float yaw; } GimbalMotorState;
typedef struct { float out; } PidTypeDef;
typedef struct {
    uint16_t key_mask, front_key_mask, back_key_mask, left_key_mask, right_key_mask;
    float swing_no_move_angle, swing_move_angle;
} ChassisControlSnapshot;
typedef struct {
    int mode;
    float vx_set, vy_set, wz_set, vx_min_speed, vx_max_speed, vy_min_speed, vy_max_speed;
    float wz, ChassisYawOffsetSet, ChassisYawSet, ChassisYaw;
    PidTypeDef ChassisAnglePid, ChassisCmdSlowSetVx, ChassisCmdSlowSetVy;
    ChassisControlSnapshot fast;
    uint8_t spinTranslationInGimbalFrame;
} ChassisMove;
typedef struct {
    struct { GimbalMotorState yaw; } gimbal;
    float max_wheel_speed, motor_distance_to_center;
} ChassisRuntimeSnapshot;
typedef struct { uint8_t wheel_type; float motor_distance_to_center, max_wheel_speed; } ChassisCoreKinematicConfig;
typedef struct { uint8_t enabled, raw_output; float vx_mps, vy_mps, wz_radps; } ChassisCoreCmd;
typedef struct { float wheel_speed_set[4]; } ChassisCoreOutput;
typedef struct { uint8_t mode, flags; float yaw_offset_rad, wz_radps, vx, vy; } ExternalMotionIntent;
static int ChassisBehaviourMode = CHASSIS_GYRO_SPIN;
static float input_x, input_y, input_wz, frame_yaw, slew_start_x, slew_start_y;
static uint8_t external_active, turnaround;
static ExternalMotionIntent external_cmd;
static uint32_t tick;
static uint32_t BspTimeGetTickMs(void) { return tick; }
static float fp32_constrain(float x, float lo, float hi) { return fminf(hi, fmaxf(lo, x)); }
static float rad_format(float x) { return remainderf(x, 2*PI); }
static float arm_sin_f32(float x) { return sinf(x); }
static float arm_cos_f32(float x) { return cosf(x); }
static void PID_clear(PidTypeDef *p) { p->out = 0; }
static float ChassisFollowYawPdCalc(PidTypeDef *p, float e, float w) { p->out = -3.5f*e-w; return p->out; }
static float ChassisGetGimbalYawRelativeAngle(const GimbalMotorState *m) { return m ? m->yaw : 0; }
static int ChassisGetTurnaroundFrameYaw(const ChassisRuntimeSnapshot *s, float *yaw)
{ (void)s; if (turnaround) *yaw = frame_yaw; return turnaround; }
static float ChassisAlgorithmYawFrame(const ChassisMove *m) { (void)m; return frame_yaw; }
static void ChassisRcToControlVector(float *x, float *y, ChassisMove *m) { (void)m; *x=input_x; *y=input_y; }
static void OtherControl(float *x, float *y, float *w, ChassisMove *m)
{ (void)m; *x=input_x; *y=input_y; *w=input_wz; }
#define ChassisZeroForceControl OtherControl
#define ChassisNoMoveControl OtherControl
#define ChassisInfantryFollowGimbalYawControl OtherControl
#define ChassisSwingControl OtherControl
#define ChassisEngineerFollowChassisYawControl OtherControl
#define ChassisNoFollowYawControl OtherControl
#define ChassisOpenSetControl OtherControl
static void ChassisAlgorithmMoveResetSlew(void) {}
static int ChassisAlgorithmMoveGetActive(ExternalMotionIntent *c) { *c=external_cmd; return external_active; }
static void ChassisAlgorithmResolveVxy(const ExternalMotionIntent *c, ChassisMove *m, uint8_t frame, float *x, float *y)
{ (void)m; (void)frame; *x=c->vx; *y=c->vy; }
static void ChassisAlgorithmDebugRecord(float x, float y, float w) { (void)x; (void)y; (void)w; }
static void ChassisAlgorithmApplySlew(const ExternalMotionIntent *c, ChassisMove *m, float *x, float *y, float *w,
                                    float sx, float sy, float sw, uint8_t spin)
{ (void)c; (void)m; (void)x; (void)y; (void)w; (void)sw; (void)spin; slew_start_x=sx; slew_start_y=sy; }
static void near(float actual, float expected) { assert(fabsf(actual-expected) < .0003f); }
'''

TESTS = r'''
static void WheelMotion(const ChassisCoreKinematicConfig *cfg, const ChassisMove *m, float *x, float *y, float *w)
{
    ChassisCoreCmd cmd = {.enabled=1, .vx_mps=m->vx_set, .vy_mps=m->vy_set, .wz_radps=m->wz_set};
    ChassisCoreOutput out;
    ChassisCoreStepVelocity(cfg, &cmd, &out);
    float *v = out.wheel_speed_set;
    for (int k=0; k<4; k++) assert(fabsf(v[k]) <= cfg->max_wheel_speed + .0001f);
    if (cfg->wheel_type == 1) {
        *x=(v[0]-v[1]-v[2]+v[3])*.25f;
        *y=(v[0]+v[1]-v[2]-v[3])*.25f;
        *w=(v[0]+v[1]+v[2]+v[3])*.25f/cfg->motor_distance_to_center;
    } else {
        *x=(v[0]+v[1]+v[2]+v[3])*.25f;
        *y=(-v[0]+v[1]+v[2]-v[3])*.25f;
        *w=(-v[0]+v[1]-v[2]+v[3])*.25f/cfg->motor_distance_to_center;
    }
}
int main(void)
{
    ChassisMove m = {.mode=CHASSIS_VECTOR_NO_FOLLOW_YAW, .vx_min_speed=-2, .vx_max_speed=2,
        .vy_min_speed=-1.5f, .vy_max_speed=1.5f, .fast={.swing_move_angle=3.5f,.swing_no_move_angle=3.5f}};
    ChassisRuntimeSnapshot snap = {.max_wheel_speed=4, .motor_distance_to_center=.395f};
    const float requests[][2] = {{.8f,0},{0,.7f},{3.96f,0},{-3.96f,0},{3.96f,3.3f},{0,0}};
    /* 经真实行为、坐标转换、混控、最终轮速限幅，再反解回云台坐标，应整圈不变。 */
    for (int type=0; type<2; type++) for (int sign=-1; sign<=1; sign+=2) {
        ChassisCoreKinematicConfig cfg = {type,.395f,4};
        m.fast.swing_move_angle = m.fast.swing_no_move_angle = sign*3.5f;
        for (unsigned r=0; r<sizeof(requests)/sizeof(requests[0]); r++) {
            input_x=requests[r][0]; input_y=requests[r][1];
            float first_x=0, first_y=0;
            for (int deg=0; deg<=360; deg++) {
                frame_yaw=deg*PI/180;
                snap.gimbal.yaw.yaw=frame_yaw;
                ChassisSetControl(&m,&snap);
                float x,y,w; WheelMotion(&cfg,&m,&x,&y,&w);
                /* 本车实测：顺时针车身角 = 正编码器角，云台前进投影为 (cos a, -sin a)。 */
                const float basis_yaw=frame_yaw*TEST_FRAME_SIGN;
                float gx=cosf(basis_yaw)*x+sinf(basis_yaw)*y;
                float gy=-sinf(basis_yaw)*x+cosf(basis_yaw)*y;
                if (deg==0) { first_x=gx; first_y=gy; }
                near(gx,first_x); near(gy,first_y); near(w,sign*3.5f);
                if (r==0) { near(gx,.8f); near(gy,0); }
                if (r==1) { near(gx,0); near(gy,.7f); }
                if (r==2) { assert(gx>1.84f && gx<1.86f); near(gy,0); }
                if (r==4) near(gy/gx,.75f);
            }
        }
    }
    /* 变速模式使用最终转速留余量；一键转身仍采用已冻结的平移参考。 */
    input_x=3.96f; input_y=0; ChassisBehaviourMode=CHASSIS_GYRO_SPIN_VAR;
    for (tick=1;tick<10000;tick+=71) {
        ChassisSetControl(&m,&snap);
        ChassisCoreKinematicConfig cfg={1,.395f,4};
        float x,y,w; WheelMotion(&cfg,&m,&x,&y,&w); near(w,m.wz_set);
    }
    ChassisBehaviourMode=CHASSIS_GYRO_SPIN; input_x=.8f; turnaround=1; frame_yaw=PI/2;
    snap.gimbal.yaw.yaw=0; ChassisSetControl(&m,&snap); near(m.vx_set,0); near(m.vy_set,.8f*TEST_FRAME_SIGN);
    turnaround=0;
    /* 本车顺时针转过 90 度时，继续朝云台前方走必须给底盘负横移。 */
    snap.gimbal.yaw.yaw=PI/2;
    ChassisSetControl(&m,&snap);
    if (TEST_FRAME_SIGN < 0) { near(m.vx_set,0); near(m.vy_set,-.8f); }
    /* 外部覆盖保留底盘坐标及斜坡起点；退出旋转不能遗留坐标标记。 */
    external_active=1; external_cmd=(ExternalMotionIntent){.mode=EXTERNAL_MOTION_MODE_NO_FOLLOW,.vx=3,.vy=-2};
    ChassisSetControl(&m,&snap); near(slew_start_x,0); near(slew_start_y,.8f);
    near(m.vx_set,2); near(m.vy_set,-1.5f); assert(!m.spinTranslationInGimbalFrame);
    external_active=0;
    ChassisBehaviourMode=CHASSIS_NO_FOLLOW_YAW; input_x=3; input_y=-2;
    m.spinTranslationInGimbalFrame=1; ChassisSetControl(&m,&snap);
    assert(!m.spinTranslationInGimbalFrame); near(m.vx_set,2); near(m.vy_set,-1.5f);
    ChassisBehaviourMode=CHASSIS_OPEN; m.mode=CHASSIS_VECTOR_RAW;
    ChassisSetControl(&m,&snap); near(m.vx_set,3); near(m.vy_set,-2);
    float x=1,y=1; snap.max_wheel_speed=1;
    ChassisLimitSpinTranslation(&x,&y,3.5f,&snap); near(x,0); near(y,0);
    x=1; y=1; snap.max_wheel_speed=0;
    ChassisLimitSpinTranslation(&x,&y,3.5f,&snap); near(x,1); near(y,1);
    ChassisSetControl(NULL,NULL);
    puts("Chassis spin translation: whole-turn speed/direction, both mixers, limits and mode transitions passed");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定编译器")
    parts = [FIXTURE]
    for path, names in [
        ("ChassisBehaviour.c", ["ChassisAlgorithmGimbalToChassis", "ChassisAlgorithmMoveOverride",
                                "ChassisGyroSpinControl", "ChassisGyroSpinVarControl", "ChassisBehaviourControlSet"]),
        ("ChassisCoreControl.inc", ["ChassisLimitSpinTranslation", "ChassisSetControl"]),
        ("ChassisCore.h", ["ChassisCoreAbsFp32", "ChassisCoreVectorToWheelSpeed",
                            "ChassisCoreLimitWheelSpeed", "ChassisCoreStepVelocity"]),
    ]:
        parts.extend(extract("shared/application/chassis/" + path, name) for name in names)
    out = ROOT / "local/build/host-tests/chassis-spin-translation"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "ExtractedSpin.c"
    source.write_text("\n".join(parts) + TESTS, encoding="utf-8")
    task = (ROOT / "shared/application/chassis/ChassisControlTask.c").read_text(encoding="utf-8")
    target = (ROOT / "Robotconfig/INFANTRY-TIAN-A/RobotConfig.h").read_text(encoding="utf-8")
    def frame_reverse(text):
        return int(re.search(r"#define CHASSIS_SPIN_FRAME_REVERSE (\d+)u", text).group(1))
    for name, reverse, expected_sign in [("default", frame_reverse(task), 1),
                                          ("infantry", frame_reverse(target), -1),
                                          ("wrong-frame", 0, -1)]:
        exe = out / f"{name}.exe"
        subprocess.run([args.zig, "cc", "-std=c99", "-Wall", "-Wextra", "-Werror",
                        f"-DCHASSIS_SPIN_FRAME_REVERSE={reverse}", f"-DTEST_FRAME_SIGN={expected_sign}",
                        str(source), "-o", str(exe)], check=True)
        if name == "wrong-frame":
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            assert result.returncode != 0 and "Assertion failed" in result.stderr
            print("Original frame direction fails the confirmed clockwise vehicle geometry, as expected")
        else:
            subprocess.run([str(exe)], check=True)


if __name__ == "__main__":
    main()

"""编译实际校准请求、状态选择、限位与输出检查函数；不连接硬件。"""
import argparse
from pathlib import Path
import shutil
import subprocess
import re


ROOT = Path(__file__).resolve().parents[2]


def function(path, name):
    text = (ROOT / path).read_text(encoding="utf-8")
    match = re.search(r"(?m)^(?:static\s+)?[\w*]+\s+" + name + r"\s*\([^;{}]*\)\s*\{", text)
    if match is None:
        raise ValueError(name)
    end = match.end()
    depth = 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[match.start():end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zig", default=shutil.which("zig"))
    args = parser.parse_args()
    if not args.zig:
        parser.error("请用 --zig 指定 Zig 编译器")
    task = "shared/application/gimbal/GimbalControlTask.c"
    behaviour = "shared/application/gimbal/GimbalBehaviour.c"
    text = (ROOT / behaviour).read_text(encoding="utf-8")
    macro = text[text.index("#define GimbalCaliGyroJudge"):text.index("/**", text.index("#define GimbalCaliGyroJudge"))]
    fixture = r'''
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <stdio.h>
typedef float fp32;
typedef uint8_t bool_t;
enum { ROBOT_RUN_MODE_FULL, ROBOT_RUN_MODE_CALIBRATION };
enum { ROBOT_RUN_VARIANT_NORMAL, ROBOT_RUN_VARIANT_GIMBAL_DUAL,
ROBOT_RUN_VARIANT_GIMBAL_YAW_ONLY, ROBOT_RUN_VARIANT_GIMBAL_PITCH_ONLY, ROBOT_RUN_VARIANT_GIMBAL_YAW_EASY };
typedef unsigned robot_run_variant_e;
typedef unsigned GimbalBehaviour;
enum { GIMBAL_ZERO_FORCE, GIMBAL_CALI, GIMBAL_INIT, GIMBAL_ANGLE, GIMBAL_MOTIONLESS, GIMBAL_PITCH_CALI };
#define INPUT_SW_GIMBAL_MODE 0
#define PI 3.1415926f
#define INIT_YAW_SET 0
#define INIT_PITCH_SET 0
#define GIMBAL_INIT_ANGLE_ERROR 0.1f
#define GIMBAL_INIT_STOP_TIME 3
#define GIMBAL_INIT_TIME 10
#define GIMBAL_CALI_START_STEP 1
#define GIMBAL_CALI_PITCH_MAX_STEP 1
#define GIMBAL_CALI_PITCH_MIN_STEP 2
#define GIMBAL_CALI_YAW_MAX_STEP 3
#define GIMBAL_CALI_YAW_MIN_STEP 4
#define GIMBAL_CALI_END_STEP 5
#define GIMBAL_CALI_MOTOR_SET 4000
#define GIMBAL_CALI_GYRO_LIMIT .1f
#define GIMBAL_CALI_STEP_TIME 3
#define GIMBAL_CALI_REDUNDANT_ANGLE .05f
#define GIMBAL_OUTPUT_MOTOR_COUNT 2
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define GimbalTotalPidClear(c) ((void)(c))
#define GimbalPidClear(c) ((void)(c))
#define PID_clear(c) ((void)(c))
typedef struct { unsigned ecd; } Measure;
typedef struct { float max_yaw,min_yaw,max_pitch,min_pitch;
uint16_t max_yaw_ecd,min_yaw_ecd,max_pitch_ecd,min_pitch_ecd; volatile uint8_t step;
uint16_t stableTicks; uint32_t startedMs; } GimbalStepCali;
typedef struct { float angle,max_angle,min_angle,motor_gyro; unsigned offset_ecd;
int GimbalMotorAnglePid,GimbalMotorGyroPid; Measure *GimbalMotorMeasure; } GimbalMotor;
typedef struct { unsigned manual_offline,mode_sw,safe_pos,run_variant,PitchCaliMode,GimbalCaliMode,control_allowed; } GimbalControlSnapshot;
typedef struct { GimbalMotor GimbalYawMotor,GimbalPitchMotor; GimbalControlSnapshot fast; GimbalStepCali GimbalCali; } GimbalControl;
typedef struct { GimbalControl control; int outputBindings[2]; } GimbalTaskContext;
typedef struct { unsigned online,dataValid,sourceTimeoutMs,sourceAgeMs,authoritySeq,semanticsSeq;
struct { unsigned sw[1]; } control; struct { unsigned GimbalSafePos; } semantics; } ManualInputSnapshot;
typedef struct { unsigned run_mode,PitchCaliMode; robot_run_variant_e run_variant; } GimbalRuntimeSnapshot;
static GimbalTaskContext s_gimbalTaskContext;
static volatile uint8_t GimbalCaliRequested,GimbalCaliControlReady;
static int16_t GimbalWatchYawCurrent,GimbalWatchPitchCurrent;
static uint32_t GimbalCaliAuthoritySeq,GimbalCaliSemanticsSeq;
static ManualInputSnapshot manual;
static unsigned mode,target=1,lifecycle=1,inputRead=1;
static uint32_t tick;
static unsigned s_gimbal_behaviour,GimbalBehaviourWatch;
static int GimbalYawEasytestCurrent;
static unsigned invalidResult;
static unsigned inhibits;
static void GimbalControlInhibitOutputs(GimbalTaskContext *ctx,const int *bindings,unsigned count)
{ (void)ctx; (void)bindings; assert(count==2); inhibits++; }
static unsigned robot_mode_current(void) { return mode; }
static unsigned robot_mode_gimbal_cali(void) { return mode==ROBOT_RUN_MODE_CALIBRATION && target==1; }
static unsigned robot_mode_gimbal_pitch_cali(void) { return mode==ROBOT_RUN_MODE_CALIBRATION && target==2; }
static unsigned robot_mode_variant(void) { return ROBOT_RUN_VARIANT_NORMAL; }
static unsigned RobotLifecycleOutputAllowed(void) { return lifecycle; }
static unsigned ManualInputSnapshotRead(ManualInputSnapshot *out) { *out=manual; return inputRead; }
static unsigned ControlInputSwitchIsPos(unsigned a,unsigned b) { return a==b; }
static uint32_t BspTimeGetTickMs(void) { return tick; }
void GimbalCalibrationCancel(void);
static void GimbalCalcCali(const GimbalStepCali *c,uint16_t *yo,uint16_t *po,float *ymax,float *ymin,float *pmax,float *pmin)
{ (void)c; *yo=100; *po=200; *ymax=invalidResult ? NAN : 1; *ymin=-1; *pmax=1; *pmin=-1; }
'''
    parts = [fixture, macro]
    parts += [function(task, name) for name in ("GimbalCalibrationInputReady", "GimbalCalibrationCancel",
              "GimbalCalibrationRequest", "GimbalCalibrationPending", "GimbalCalibrationOutputAllowed")]
    parts.append(function("shared/application/gimbal/GimbalCalibration.c", "GimbalCalibrationSet"))
    parts += [function(task, name) for name in ("GimbalCalibrationRestoreAllowed", "GimbalCalibrationRestore")]
    parts.append(function("shared/application/gimbal/GimbalCalibration.c", "GimbalCalibrationStep"))
    parts.append(function(task, "cmd_cali_gimbal_hook"))
    parts += [function(behaviour, name) for name in ("GimbalBehavourSet", "GimbalCaliControl")]
    parts.append(function("shared/application/gimbal/GimbalRuntime.c", "GimbalApplyOperationMode"))
    parts.append(r'''
int main(void)
{
    GimbalControl *c=&s_gimbalTaskContext.control;
    Measure yaw={100},pitch={200};
    uint16_t yo=0,po=0; float ymax=0,ymin=0,pmax=0,pmin=0;
    float y=0,p=0; int16_t yi=100,pi=100;
    c->GimbalYawMotor.GimbalMotorMeasure=&yaw; c->GimbalPitchMotor.GimbalMotorMeasure=&pitch;
    c->fast=(GimbalControlSnapshot){.mode_sw=2,.safe_pos=1,.GimbalCaliMode=1,.control_allowed=1};
    manual=(ManualInputSnapshot){.online=1,.dataValid=1,.sourceTimeoutMs=100,.authoritySeq=1,.semanticsSeq=1};
    manual.control.sw[0]=2; manual.semantics.GimbalSafePos=1;
    assert(GimbalCalibrationRestore(100,200,1,-1,1,-1)==0); /* 初始化前不能恢复。 */
    GimbalCaliControlReady=1;
    assert(GimbalCalibrationRestore(100,200,1,-1,1,-1)==0); /* 正常运行不能更换角度零点。 */
    manual.control.sw[0]=1;
    GimbalWatchYawCurrent=100;
    assert(GimbalCalibrationRestore(100,200,1,-1,1,-1)==0); /* 还有旧控制输出。 */
    GimbalWatchYawCurrent=0;
    assert(GimbalCalibrationRestore(100,200,1,-1,1,-1)==1);
    manual.control.sw[0]=2;
    assert(GimbalCalibrationRequest()==0); /* 正常模式不能发起机械校准。 */
    mode=ROBOT_RUN_MODE_CALIBRATION;
    GimbalBehavourSet(c); assert(s_gimbal_behaviour==GIMBAL_ZERO_FORCE);
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0 && c->GimbalCali.step==0);
    assert(GimbalCalibrationRequest()==1);
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0 && c->GimbalCali.step==1);
    GimbalBehavourSet(c); assert(s_gimbal_behaviour==GIMBAL_CALI);
    c->GimbalPitchMotor.motor_gyro=-2;
    for(unsigned i=0;i<10;i++) GimbalCaliControl(&y,&p,c);
    assert(c->GimbalCali.step==1 && c->GimbalCali.stableTicks==0); /* 负速度也在运动。 */
    c->GimbalPitchMotor.motor_gyro=0;
    GimbalCaliControl(&y,&p,c); assert(c->GimbalCali.stableTicks==1);
    c->GimbalPitchMotor.motor_gyro=2;
    GimbalCaliControl(&y,&p,c); assert(c->GimbalCali.stableTicks==0);
    c->GimbalPitchMotor.motor_gyro=0;
    for(unsigned i=0;i<4;i++) GimbalCaliControl(&y,&p,c);
    assert(c->GimbalCali.step==2);
    assert(GimbalCalibrationOutputAllowed()==1);
    manual.online=0; assert(GimbalCalibrationOutputAllowed()==0);
    GimbalApplyOperationMode(&s_gimbalTaskContext,NULL,&yi,&pi); assert(yi==0 && pi==0);
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0);
    manual.online=1; assert(GimbalCalibrationPending()==0 && c->GimbalCali.step==0);
    assert(inhibits>0); /* 取消还必须撤销已发布或尚未提交的旧电流。 */
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0 && c->GimbalCali.step==0);
    assert(GimbalCalibrationRequest()==1);
    manual.authoritySeq++; assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0);
    assert(GimbalCalibrationPending()==0);
    assert(GimbalCalibrationRequest()==1);
    manual.sourceAgeMs=101; assert(GimbalCalibrationOutputAllowed()==0);
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0);
    manual.sourceAgeMs=0; target=0; assert(GimbalCalibrationRequest()==0);
    target=1; assert(GimbalCalibrationRequest()==1);
    cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin);
    tick=60001; GimbalCaliControl(&y,&p,c); assert(y==0 && p==0 && GimbalCalibrationPending()==0);
    tick=0; assert(GimbalCalibrationRequest()==1);
    cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin);
    c->GimbalCali.step=GIMBAL_CALI_END_STEP;
    assert(GimbalCalibrationPending()==1 && GimbalCalibrationOutputAllowed()==0);
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==1);
    assert(GimbalCalibrationPending()==0 && yo==100 && po==200 && ymax>ymin);
    assert(GimbalCalibrationRequest()==1);
    cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin);
    c->GimbalCali.step=GIMBAL_CALI_END_STEP; invalidResult=1;
    assert(cmd_cali_gimbal_hook(&yo,&po,&ymax,&ymin,&pmax,&pmin)==0);
    assert(GimbalCalibrationPending()==0 && c->GimbalYawMotor.max_angle>.9f);
    manual.dataValid=0; assert(GimbalCalibrationRequest()==0); manual.dataValid=1;
    manual.control.sw[0]=1; assert(GimbalCalibrationRequest()==0); manual.control.sw[0]=2;
    c->fast.GimbalCaliMode=0; assert(GimbalCalibrationRequest()==0);
    target=0; c->fast.GimbalCaliMode=0; GimbalBehavourSet(c); assert(s_gimbal_behaviour==GIMBAL_ZERO_FORCE);
    puts("PASS: explicit gimbal calibration, cancellation, limit detection and final output guard");
    return 0;
}
''')
    out = ROOT / "local/build/host-tests/gimbal-calibration"
    out.mkdir(parents=True, exist_ok=True)
    source = out / "regression.c"
    source.write_text("\n".join(parts), encoding="utf-8")
    executable = out / "regression.exe"
    subprocess.run([args.zig, "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", str(source), "-o", str(executable)], check=True)
    subprocess.run([str(executable)], check=True)


if __name__ == "__main__":
    main()

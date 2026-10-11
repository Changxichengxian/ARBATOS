// wbr_mujoco 控制器的 C ABI 桥。
//
// 编译时同时加入 controller/src/{balance,leg,chassis_fsm}.cpp，并把
// controller/include 加入头文件路径。这里明确采用 runtime 的 IMU bypass
// 路径，直接使用 MuJoCo 提供的真值四元数。

#include "controller/chassis_fsm.hpp"

#include <cstring>
#include <cmath>
#include <new>

#if defined(_WIN32)
#define WBR_EXPORT extern "C" __declspec(dllexport)
#else
#define WBR_EXPORT extern "C" __attribute__((visibility("default")))
#endif

namespace
{

constexpr int kMotorCount = 6;
constexpr int kSensorCount = 28;
constexpr int kCommandCount = 7;
constexpr int kOutputCount = 32;

controller::chassis_config MakeWbrConfig(float rise_min_length = 0.16f)
{
    // 完整对应 wbr_mujoco 提交 81a1035 的 config/robots/wbr.yaml。
    // YAML 未覆盖的物理参数也逐项写明，便于脱离配置加载器独立核查。
    controller::chassis_config cfg{};
    cfg.l1 = 0.220f;
    cfg.l2 = 0.260f;
    cfg.lmin = 0.16f;
    cfg.rise_min_length = rise_min_length;
    cfg.lmid = 0.24f;
    cfg.lmax = 0.36f;
    cfg.lfly = 0.22f;
    cfg.lswitch = 0.24f;
    cfg.rwheel = 0.06f;
    cfg.mwheel = 0.21f;
    cfg.tk_wheel = 1400.0f;
    cfg.thip_max = 40.0f;
    cfg.twheel_max = 5.2f;
    cfg.gff = 67.865f;
    cfg.fspring = 450.0f;
    cfg.dspring1 = 0.03f;
    cfg.dspring2 = 0.05f;
    cfg.ang_spring = 0.2164f;
    cfg.alpha_eq_coeff[0] = 0.280918f;
    cfg.alpha_eq_coeff[1] = -1.101757f;
    cfg.alpha_eq_coeff[2] = 1.232768f;
    cfg.imu_offset_x = 0.0f;
    cfg.control_dt = 0.001f;

    cfg.max_cmd_velocity = 0.8f;
    cfg.max_cmd_yaw_rate = 1.5f;
    cfg.force_relax = false;

    cfg.motor_zero_rad[0] = -0.06f;
    cfg.motor_zero_rad[1] = -0.20f;
    cfg.motor_zero_rad[2] = 0.0f;
    cfg.motor_zero_rad[3] = 0.06f;
    cfg.motor_zero_rad[4] = 0.20f;
    cfg.motor_zero_rad[5] = 0.0f;

    cfg.leg_pid.len = {6000.0f, 0.0f, -1500.0f, 100.0f, 0.0f, controller::pid_mode::dvel};
    cfg.leg_pid.phi = {0.7f, 0.0f, 1.4f, 20.0f, 0.005f, controller::pid_mode::position};
    cfg.fsm_pid.roll = {0.7f, 0.0001f, 1.4f, 0.05f, 0.005f, controller::pid_mode::position};
    cfg.fsm_pid.len_balance =
        {2000.0f, 0.0f, -400.0f, 125.0f, 0.0f, controller::pid_mode::position};
    cfg.fsm_pid.recover = {20.0f, 10.0f, 0.005f};
    cfg.fsm_pid.recover_kick_torque = 15.0f;
    cfg.fsm_pid.flatten_high = {10.0f, 5.0f, 0.01f};
    cfg.fsm_pid.flatten_low = {5.0f, 10.0f, 0.002f};
    cfg.fsm_pid.neutral = {30.0f, 10.0f, 0.003f};

    cfg.fsm.enter_normal_alpha = 0.58f;
    cfg.fsm.enter_normal_ticks = 8;
    cfg.fsm.exit_relax_alpha = 1.05f;
    cfg.fsm.exit_relax_ticks = 250;
    cfg.fsm.offground_min_air_ticks = 800;
    return cfg;
}

bool KeepsPlanarOdometry(controller::chassis_state state)
{
    return state == controller::chassis_state::normal;
}

class WbrController
{
public:
    explicit WbrController(float rise_min_length = 0.16f)
        : cfg_(MakeWbrConfig(rise_min_length)), left_(true, 0, 1, cfg_), right_(false, 3, 4, cfg_)
    {
        fsm_.init(cfg_);
    }

    int Step(const float* sensor, const float* command, float* output)
    {
        if (sensor == nullptr || command == nullptr || output == nullptr)
        {
            return -1;
        }

        controller::msg_raw_state_t raw{};
        for (int i = 0; i < 4; ++i)
        {
            raw.quat_gt[i] = sensor[i];
        }
        for (int i = 0; i < 3; ++i)
        {
            raw.gyro[i] = sensor[4 + i];
            raw.accel[i] = sensor[7 + i];
        }
        for (int i = 0; i < kMotorCount; ++i)
        {
            raw.motors[i].q = sensor[10 + i * 3];
            raw.motors[i].dq = sensor[11 + i * 3];
            raw.motors[i].tau = sensor[12 + i * 3];
        }

        // 与 runtime --imu-mode bypass 的处理一致。
        controller::msg_ins_t ins{};
        std::memcpy(ins.quaternion, raw.quat_gt, sizeof(ins.quaternion));
        controller::quat_to_euler(ins.quaternion, ins.roll, ins.pitch, ins.yaw);
        ins.total_yaw = ins.yaw;
        ins.gyro_r = raw.gyro[0];
        ins.gyro_p = raw.gyro[1];
        ins.gyro_y = raw.gyro[2];
        std::memcpy(ins.accel, raw.accel, sizeof(ins.accel));

        // 直接输入 msg_cmd_t；原 command_fusion 负责的位置和偏航参考更新由调用方完成。
        controller::msg_cmd_t cmd{};
        cmd.x = command[0];
        cmd.v = command[1];
        cmd.len = command[2];
        cmd.yaw = command[3];
        cmd.dyaw = command[4];
        cmd.move = command[5] > 0.5f;
        cmd.spin = command[6] > 0.5f;

        const float vl = raw.motors[2].dq * cfg_.rwheel * left_.wheel_sign();
        const float vr = raw.motors[5].dq * cfg_.rwheel * right_.wheel_sign();
        odom_.update(ins.quaternion, ins.accel, (vl + vr) * 0.5f, ins.total_yaw, cfg_.control_dt);
        if (!KeepsPlanarOdometry(fsm_.state()))
        {
            odom_.reset();
        }

        left_.link().solve(ins.pitch, ins.gyro_p, odom_.az, raw.motors[0], raw.motors[1]);
        right_.link().solve(ins.pitch, ins.gyro_p, odom_.az, raw.motors[3], raw.motors[4]);
        const auto& ll = left_.link();
        const auto& rl = right_.link();

        controller::msg_odometry_t odom_msg{};
        odom_msg.x = odom_.x;
        odom_msg.v = odom_.v;
        odom_msg.a_z = odom_.az;

        controller::fsm_inputs fin{ins, cmd, odom_msg, left_, right_};
        fin.observed_x[0] = odom_.x;
        fin.observed_x[1] = odom_.v;
        fin.observed_x[2] = ins.total_yaw;
        fin.observed_x[3] = ins.gyro_y;
        fin.observed_x[4] = ll.alpha_;
        fin.observed_x[5] = ll.dalpha_;
        fin.observed_x[6] = rl.alpha_;
        fin.observed_x[7] = rl.dalpha_;
        fin.observed_x[8] = ins.pitch;
        fin.observed_x[9] = ins.gyro_p;
        fin.n_total = ll.n_ + rl.n_;
        fin.chassis_dead = false;

        controller::fsm_outputs fout{};
        fsm_.step(fin, fout);
        fout.pendulum.planar_valid = KeepsPlanarOdometry(fsm_.state());
        if (!cmd.move || cfg_.force_relax)
        {
            std::memset(&fout.motor, 0, sizeof(fout.motor));
        }

        for (int i = 0; i < kMotorCount; ++i)
        {
            output[i] = fout.motor.tau[i];
        }
        output[6] = ins.roll;
        output[7] = ins.pitch;
        output[8] = ins.yaw;
        output[9] = odom_.x;
        output[10] = odom_.v;
        output[11] = odom_.az;
        output[12] = ll.len_;
        output[13] = ll.dlen_;
        output[14] = ll.alpha_;
        output[15] = ll.dalpha_;
        output[16] = ll.phi_;
        output[17] = ll.n_;
        output[18] = rl.len_;
        output[19] = rl.dlen_;
        output[20] = rl.alpha_;
        output[21] = rl.dalpha_;
        output[22] = rl.phi_;
        output[23] = rl.n_;
        output[24] = static_cast<float>(static_cast<unsigned int>(fsm_.state()));
        output[25] = fin.n_total;
        output[26] = cmd.x;
        output[27] = cmd.v;
        output[28] = cmd.len;
        output[29] = cmd.yaw;
        output[30] = cmd.dyaw;
        output[31] = fout.pendulum.planar_valid ? 1.0f : 0.0f;
        return 0;
    }

private:
    controller::chassis_config cfg_;
    controller::leg_controller left_;
    controller::leg_controller right_;
    controller::odometry odom_;
    controller::chassis_fsm fsm_;
};

}  // namespace

// sensor[28]：quat(w,x,y,z)、gyro(r,p,y)、accel(x,y,z)，然后是
//             6 * (q,dq,tau)。电机顺序为
//             ljoint1、ljoint4、lwheel、rjoint1、rjoint4、rwheel。
// command[7]：x、v、len、yaw、dyaw、move、spin。
// output[32]：tau[6]、roll、pitch、yaw、odom(x,v,az)、
//             左腿(len,dlen,alpha,dalpha,phi,n)、
//             右腿(len,dlen,alpha,dalpha,phi,n)、
//             fsm、n_total、command(x,v,len,yaw,dyaw)、planar_valid。
// 成功返回 0，参数为空返回 -1。

WBR_EXPORT void* WbrCreate()
{
    return new (std::nothrow) WbrController();
}

// 仅参考模型适配调用；保留 WbrCreate 的原作者默认配置。
WBR_EXPORT void* WbrCreateWithRiseLength(float length)
{
    if (!std::isfinite(length) || length < 0.16f || length > 0.24f)
    {
        return nullptr;
    }
    return new (std::nothrow) WbrController(length);
}

WBR_EXPORT void WbrDestroy(void* handle)
{
    delete static_cast<WbrController*>(handle);
}

WBR_EXPORT int WbrStep(void* handle, const float* sensor, const float* command, float* output)
{
    if (handle == nullptr)
    {
        return -1;
    }
    return static_cast<WbrController*>(handle)->Step(sensor, command, output);
}

static_assert(kSensorCount == 4 + 3 + 3 + kMotorCount * 3, "sensor ABI size mismatch");
static_assert(kCommandCount == 7, "command ABI size mismatch");
static_assert(kOutputCount == 32, "output ABI size mismatch");

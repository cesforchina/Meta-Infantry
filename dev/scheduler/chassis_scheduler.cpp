//
// Created by Administrator on 2019/1/11 0011.
//

#include "chassis_scheduler.h"

ChassisSKD::mode_t ChassisSKD::mode = STOP_MODE;

float ChassisSKD::target_vx;
float ChassisSKD::target_vy;
float ChassisSKD::target_theta;

PIDController ChassisSKD::a2v_pid;
PIDController ChassisSKD::v2i_pid[MOTOR_COUNT];

float ChassisSKD::target_w;
float ChassisSKD::target_velocity[MOTOR_COUNT];
int ChassisSKD::target_current[MOTOR_COUNT];

float ChassisSKD::w_to_v_ratio_ = 0.0f;
float ChassisSKD::v_to_wheel_angular_velocity_ = 0.0f;

ChassisSKD::SKDThread ChassisSKD::skdThread;



void ChassisSKD::start(float wheel_base, float wheel_tread, float wheel_circumference, tprio_t thread_prio) {
    w_to_v_ratio_ = (wheel_base + wheel_tread) / 2.0f / 360.0f * 3.14159f;
    v_to_wheel_angular_velocity_ = (360.0f / wheel_circumference);

    skdThread.start(thread_prio);
}

void ChassisSKD::load_pid_params(PIDControllerBase::pid_params_t a2v_pid_params,
                                 PIDControllerBase::pid_params_t v2i_pid_params) {
    a2v_pid.change_parameters(a2v_pid_params);
    for (int i = 0; i < MOTOR_COUNT; i++) {
        v2i_pid[i].change_parameters(v2i_pid_params);
    }
}

void ChassisSKD::set_mode(ChassisSKD::mode_t skd_mode) {
    mode = skd_mode;
}

void ChassisSKD::set_target(float vx, float vy, float theta) {
    target_vx = vx;
    target_vy = vy;
    target_theta = theta;
}

void ChassisSKD::velocity_decompose(float vx, float vy, float w) {

    // FR, -vx, +vy, +w
    // FL, -vx, -vy, +w, since the motor is installed in the opposite direction
    // BL, +vx, -vy, +w, since the motor is installed in the opposite direction
    // BR, +vx, +vy, +w

    target_velocity[FR] = (-vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FR] = (int) v2i_pid[FR].calc(ChassisIF::feedback[FR].actual_velocity, target_velocity[FR]);

    target_velocity[FL] = (-vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[FL] = (int) v2i_pid[FL].calc(ChassisIF::feedback[FL].actual_velocity, target_velocity[FL]);

    target_velocity[BL] = (+vx - vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BL] = (int) v2i_pid[BL].calc(ChassisIF::feedback[BL].actual_velocity, target_velocity[BL]);

    target_velocity[BR] = (+vx + vy + w * w_to_v_ratio_) * v_to_wheel_angular_velocity_;
    target_current[BR] = (int) v2i_pid[BR].calc(ChassisIF::feedback[BR].actual_velocity, target_velocity[BR]);
}

void ChassisSKD::SKDThread::main() {
    setName("chassis_skd");
    while (!shouldTerminate()) {

        if (mode == GIMBAL_COORDINATE_MODE) {

            float theta = GimbalIF::feedback[GimbalIF::YAW].actual_angle;
            target_w = a2v_pid.calc(theta, target_theta);
            velocity_decompose(target_vx * cosf(theta) - target_vy * sinf(theta),
                               target_vx * sinf(theta) + target_vy * cosf(theta),
                               target_w);

        } else if (mode == PARAM_ADJUST_MODE) {

            // TODO: write code for PID parameter adjustment mode

        } else if (mode == STOP_MODE) {

            for (int &i : ChassisIF::target_current) {
                i = 0;
            }

        }

        // Send currents
        for (size_t i = 0; i < MOTOR_COUNT; i++) {
            GimbalIF::target_current[i] = target_current[i];
        }
        ChassisIF::send_chassis_currents();

        sleep(TIME_MS2I(SKD_THREAD_INTERVAL));
    }
}
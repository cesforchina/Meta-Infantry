//
// Created by Administrator on 2019/1/11 0011.
//

#include "chassis_calculator.h"
#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "serial_shell.h"
#include "can_interface.h"

ChassisController::motor_t ChassisController::motor[CHASSIS_MOTOR_COUNT];

void ChassisController::calc(float *measured_angular_velocity, float target_vx, float target_vy, float target_w) {

    // FR, -vx, +vy, +w
    // FL, -vx, -vy, +w, since the motor is installed in the opposite direction
    // BL, +vx, -vy, +w, since the motor is installed in the opposite direction
    // BR, +vx, +vy, +w

    motor[CHASSIS_FR].target_velocity = (-target_vx + target_vy + target_w * w_to_v_ratio) * v_to_wheel_angular_velocity;
    motor[CHASSIS_FR].target_current = motor[CHASSIS_FR].pid.calc(measured_angular_velocity[CHASSIS_FR],
                                                                  motor[CHASSIS_FR].target_velocity);
    motor[CHASSIS_FL].target_velocity = (-target_vx - target_vy + target_w * w_to_v_ratio) * v_to_wheel_angular_velocity;
    motor[CHASSIS_FL].target_current = motor[CHASSIS_FL].pid.calc(measured_angular_velocity[CHASSIS_FL],
                                                                  motor[CHASSIS_FL].target_velocity);
    motor[CHASSIS_BL].target_velocity = (+target_vx - target_vy + target_w * w_to_v_ratio) * v_to_wheel_angular_velocity;
    motor[CHASSIS_BL].target_current = motor[CHASSIS_BL].pid.calc(measured_angular_velocity[CHASSIS_BL],
                                                                  motor[CHASSIS_BL].target_velocity);
    motor[CHASSIS_BR].target_velocity = (+target_vx + target_vy + target_w * w_to_v_ratio) * v_to_wheel_angular_velocity;
    motor[CHASSIS_BR].target_current = motor[CHASSIS_BR].pid.calc(measured_angular_velocity[CHASSIS_BR],
                                                                  motor[CHASSIS_BR].target_velocity);
}

void ChassisController::rotateAroundWheel(chassis_motor_id_t pivotWheel, bool isClockwise, float angular_velocity) {

}
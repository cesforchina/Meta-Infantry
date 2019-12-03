//
// Created by liuzikai on 2019-04-22.
// This file contains common parameters for infantry
//

#ifndef META_INFANTRY_VEHICLE_INFANTRY_H
#define META_INFANTRY_VEHICLE_INFANTRY_H



/// AHRS Configurations
#define ON_BOARD_AHRS_MATRIX {{0.0f, 0.0f, 1.0f}, \
                              {1.0f, 0.0f, 0.0f}, \
                              {0.0f, 1.0f, 0.0f}}


/// Gimbal and Shoot Installation Configurations
#define GIMBAL_YAW_MOTOR_TYPE     (GimbalIF::RM6623)
#define GIMBAL_PITCH_MOTOR_TYPE   (GimbalIF::RM6623)
#define SHOOT_BULLET_MOTOR_TYPE   (GimbalIF::M2006)
#define FW_MOTOR_TYPE             (GimbalIF::M3508)

#define GIMBAL_YAW_INSTALL_DIRECTION    (GimbalSKD::POSITIVE)
#define GIMBAL_PITCH_INSTALL_DIRECTION  (GimbalSKD::POSITIVE)
#define SHOOT_BULLET_INSTALL_DIRECTION  (ShootSKD::POSITIVE)
#define SHOOT_FW_LEFT_INSTALL_DIRECTION  (ShootSKD::POSITIVE)
#define SHOOT_FW_RIGHT_INSTALL_DIRECTION  (ShootSKD::NEGATIVE)
#define SHOOT_DEGREE_PER_BULLET 40.0f  // rotation degree of bullet loader for each bullet

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 1.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  -1.0f, 0.0f}, \
                                         {0.0f,  0.0f,  1.0f}, \
                                         {-1.0f, 0.0f,  0.0f}}

#define MPU6500_BIAS_DATA_ID 0x0001

/// Gimbal and Shoot PID Parameters
#define GIMBAL_PID_YAW_A2V_KP 5.3f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 0.18f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 720.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 720.0f
#define GIMBAL_PID_YAW_A2V_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_YAW_V2I_KP 18.0f
#define GIMBAL_PID_YAW_V2I_KI 0.5f
#define GIMBAL_PID_YAW_V2I_KD 0.0f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 1000.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 3000.0f
#define GIMBAL_PID_YAW_V2I_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 10.0f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.75f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 60.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 75.0f
#define GIMBAL_PID_PITCH_A2V_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 19.0f
#define GIMBAL_PID_PITCH_V2I_KI 0.35f
#define GIMBAL_PID_PITCH_V2I_KD 0.00f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 200.0f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 2500.0f
#define GIMBAL_PID_PITCH_V2I_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define GIMBAL_RESTRICT_YAW_MIN_ANGLE -90
#define GIMBAL_RESTRICT_YAW_MAX_ANGLE 90
#define GIMBAL_RESTRICT_YAW_VELOCITY 20

#define SHOOT_PID_BULLET_LOADER_A2V_KP 10.0f  // a number large enough, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_KI 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT 360.0f  // will be replaced, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_V2I_KP 25.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KI 0.7f
#define SHOOT_PID_BULLET_LOADER_V2I_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT 3000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT 4000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_PARAMS \
    {SHOOT_PID_BULLET_LOADER_V2I_KP, SHOOT_PID_BULLET_LOADER_V2I_KI, SHOOT_PID_BULLET_LOADER_V2I_KD, \
    SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT, SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT}

#define SHOOT_PID_FW_LEFT_V2I_KP 45.0f
#define SHOOT_PID_FW_LEFT_V2I_KI 0.1f
#define SHOOT_PID_FW_LEFT_V2I_KD 0.02f
#define SHOOT_PID_FW_LEFT_V2I_I_LIMIT 2000.0f
#define SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT 6000.0f
#define SHOOT_PID_FW_LEFT_V2I_PARAMS \
    {SHOOT_PID_FW_LEFT_V2I_KP, SHOOT_PID_FW_LEFT_V2I_KI, SHOOT_PID_FW_LEFT_V2I_KD, \
     SHOOT_PID_FW_LEFT_V2I_I_LIMIT, SHOOT_PID_FW_LEFT_V2I_OUT_LIMIT}

#define SHOOT_PID_FW_RIGHT_V2I_KP 45.0f
#define SHOOT_PID_FW_RIGHT_V2I_KI 0.1f
#define SHOOT_PID_FW_RIGHT_V2I_KD 0.02f
#define SHOOT_PID_FW_RIGHT_V2I_I_LIMIT 2000.0f
#define SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT 6000.0f
#define SHOOT_PID_FW_RIGHT_V2I_PARAMS \
    {SHOOT_PID_FW_RIGHT_V2I_KP, SHOOT_PID_FW_RIGHT_V2I_KI, SHOOT_PID_FW_RIGHT_V2I_KD, \
     SHOOT_PID_FW_RIGHT_V2I_I_LIMIT, SHOOT_PID_FW_RIGHT_V2I_OUT_LIMIT}    
    
/// Chassis Mechanism Parameters
#define CHASSIS_WHEEL_BASE  420.0f                     // distance between front axle and the back axle [mm]
#define CHASSIS_WHEEL_TREAD 372.0f                     // distance between left and right wheels [mm]
#define CHASSIS_WHEEL_CIRCUMFERENCE 478.0f             // circumference of wheels [mm]


/// Chassis PID Parameters
#define CHASSIS_PID_V2I_KP 26.0f
#define CHASSIS_PID_V2I_KI 0.1f
#define CHASSIS_PID_V2I_KD 0.02f
#define CHASSIS_PID_V2I_I_LIMIT 2000.0f
#define CHASSIS_PID_V2I_OUT_LIMIT 6000.0f
#define CHASSIS_PID_V2I_PARAMS \
    {CHASSIS_PID_V2I_KP, CHASSIS_PID_V2I_KI, CHASSIS_PID_V2I_KD, \
    CHASSIS_PID_V2I_I_LIMIT, CHASSIS_PID_V2I_OUT_LIMIT}

#define CHASSIS_CLIP_PID_V2I_KP 26.0f
#define CHASSIS_CLIP_PID_V2I_KI 0.1f
#define CHASSIS_CLIP_PID_V2I_KD 0.02f
#define CHASSIS_CLIP_PID_V2I_I_LIMIT 2000.0f
#define CHASSIS_CLIP_PID_V2I_OUT_LIMIT 6000.0f
#define CHASSIS_CLIP_PID_V2I_PARAMS \
    {CHASSIS_CLIP_PID_V2I_KP, CHASSIS_CLIP_PID_V2I_KI, CHASSIS_CLIP_PID_V2I_KD, \
    CHASSIS_CLIP_PID_V2I_I_LIMIT, CHASSIS_CLIP_PID_V2I_OUT_LIMIT}               
    // Infantry clip PID params of no use that is set to handle conflicts between Hero and Infantry

#define CHASSIS_CLIP_PID_THETA2V_KP 10.0f
#define CHASSIS_CLIP_PID_THETA2V_KI 0.02f
#define CHASSIS_CLIP_PID_THETA2V_KD 0.0f
#define CHASSIS_CLIP_PID_THETA2V_I_LIMIT 60.0f
#define CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT 270.0f
#define CHASSIS_CLIP_PID_THETA2V_PARAMS \
    {CHASSIS_CLIP_PID_THETA2V_KP, CHASSIS_CLIP_PID_THETA2V_KI, CHASSIS_CLIP_PID_THETA2V_KD, \
    CHASSIS_CLIP_PID_THETA2V_I_LIMIT, CHASSIS_CLIP_PID_THETA2V_OUT_LIMIT}
    
#define CHASSIS_FOLLOW_PID_THETA2V_KP 10.0f
#define CHASSIS_FOLLOW_PID_THETA2V_KI 0.00f
#define CHASSIS_FOLLOW_PID_THETA2V_KD 0.0f
#define CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT 60.0f
#define CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT 270.0f
#define CHASSIS_FOLLOW_PID_THETA2V_PARAMS \
    {CHASSIS_FOLLOW_PID_THETA2V_KP, CHASSIS_FOLLOW_PID_THETA2V_KI, CHASSIS_FOLLOW_PID_THETA2V_KD, \
    CHASSIS_FOLLOW_PID_THETA2V_I_LIMIT, CHASSIS_FOLLOW_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_DODGE_PID_THETA2V_KP 10.0f
#define CHASSIS_DODGE_PID_THETA2V_KI 0.02f
#define CHASSIS_DODGE_PID_THETA2V_KD 0.0f
#define CHASSIS_DODGE_PID_THETA2V_I_LIMIT 60.0f
#define CHASSIS_DODGE_PID_THETA2V_OUT_LIMIT 270.0f
#define CHASSIS_DODGE_PID_THETA2V_PARAMS \
    {CHASSIS_DODGE_PID_THETA2V_KP, CHASSIS_DODGE_PID_THETA2V_KI, CHASSIS_DODGE_PID_THETA2V_KD, \
    CHASSIS_DODGE_PID_THETA2V_I_LIMIT, CHASSIS_DODGE_PID_THETA2V_OUT_LIMIT}

#define CHASSIS_DODGE_MODE_THETA   45
#define CHASSIS_DODGE_MODE_INTERVAL   700
#define CHASSIS_BIASED_ANGLE 0
    
/// Thread Priority List
#define THREAD_CAN1_PRIO                    (HIGHPRIO - 1)
#define THREAD_MPU_PRIO                     (HIGHPRIO - 2)
#define THREAD_IST_PRIO                     (HIGHPRIO - 3)
#define THREAD_AHRS_PRIO                    (HIGHPRIO - 4)
#define THREAD_CAN2_PRIO                    (HIGHPRIO - 5)
#define THREAD_GIMBAL_SKD_PRIO              (NORMALPRIO + 3)
#define THREAD_CHASSIS_SKD_PRIO             (NORMALPRIO + 2)
#define THREAD_SHOOT_SKD_PRIO               (NORMALPRIO + 1)
#define THREAD_USER_PRIO                    (NORMALPRIO)
#define THREAD_USER_ACTION_PRIO             (NORMALPRIO - 1)
#define THREAD_CHASSIS_LG_DODGE_PRIO        (NORMALPRIO - 2)
#define THREAD_SHOOT_LG_STUCK_DETECT_PRIO   (NORMALPRIO - 3)
#define THREAD_GIMBAL_LG_VISION_PRIO        (NORMALPRIO - 4)
#define THREAD_REFEREE_SENDING_PRIO         (NORMALPRIO - 5)
#define THREAD_INSPECTOR_PRIO               (NORMALPRIO - 10)
#define THREAD_SHOOT_BULLET_COUNTER_PRIO    (LOWPRIO + 7)
#define THREAD_USER_CLIENT_DATA_SEND_PRIO   (LOWPRIO + 6)
#define THREAD_SHELL_PRIO                   (LOWPRIO + 5)
#define THREAD_BUZZER_PRIO                  (LOWPRIO)

/// Dev Board LED Usage List
#define DEV_BOARD_LED_SYSTEM_INIT 1
#define DEV_BOARD_LED_CAN         2
#define DEV_BOARD_LED_AHRS        3
#define DEV_BOARD_LED_REMOTE      4
#define DEV_BOARD_LED_GIMBAL      5
#define DEV_BOARD_LED_CHASSIS     6
#define DEV_BOARD_LED_REFEREE     7  // used in infantry ShootLG BulletCounterThread
#define DEV_BOARD_LED_SD_CARD     8

/// User Client Usage List
#define USER_CLIENT_FW_STATE_LIGHT                  0
#define USER_CLIENT_DODGE_MODE_LIGHT                1
#define USER_CLIENT_SUPER_CAPACITOR_STATUS_LIGHT    2
#define USER_CLIENT_SPEED_LEVEL_3_LIGHT             3
#define USER_CLIENT_SPEED_LEVEL_2_LIGHT             4
#define USER_CLIENT_SPEED_LEVEL_1_LIGHT             5

#define USER_CLIENT_FW_SPEED_NUM                    2
//#define USER_CLIENT_REMAINING_HEAT_NUM              2
#define USER_CLIENT_ACQUIRED_BULLET_NUM             3
//#define USER_CLIENT_ACTUAL_POWER_NUM                2
#define USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM     1

/// Super Capacitor Configurations
#define SUPER_CAPACITOR_WARNING_VOLTAGE   15


#endif //META_INFANTRY_VEHICLE_INFANTRY_H

//
// Created by zhukerui on 2019/5/18.
// Copy from liuzikai
//

#ifndef META_SENTRY_VEHICLE_SENTRY_H
#define META_SENTRY_VEHICLE_SENTRY_H


/// Gimbal and Shoot Installation Configurations
#define GIMBAL_YAW_MOTOR_TYPE     (GimbalIF::GM6020)
#define GIMBAL_PITCH_MOTOR_TYPE   (GimbalIF::GM3510)
#define SHOOT_BULLET_MOTOR_TYPE   (GimbalIF::M2006)
#define FW_MOTOR_TYPE             (GimbalIF::M3508)

#define GIMBAL_YAW_INSTALL_DIRECTION      (GimbalSKD::NEGATIVE)
#define GIMBAL_YAW_DECELERATION_RATIO     1.0f
#define GIMBAL_PITCH_INSTALL_DIRECTION    (GimbalSKD::POSITIVE)
#define GIMBAL_PITCH_DECELERATION_RATIO   2.0f
#define SHOOT_BULLET_INSTALL_DIRECTION  (ShootSKD::POSITIVE)
#define SHOOT_DEGREE_PER_BULLET 40.0f  // rotation degree of bullet loader for each bullet
#define SHOOT_FW_LEFT_INSTALL_DIRECTION  (ShootSKD::POSITIVE)
#define SHOOT_FW_RIGHT_INSTALL_DIRECTION  (ShootSKD::NEGATIVE)

#define GIMBAL_ANGLE_INSTALLATION_MATRIX {{1.0f, 0.0f, 0.0f}, \
                                          {0.0f, 0.0f, -1.0f}, \
                                          {0.0f, 1.0f, 0.0f}}


#define GIMBAL_GYRO_INSTALLATION_MATRIX {{0.0f,  0.0f, -1.0f}, \
                                         {0.0f,  1.0f,  0.0f}, \
                                         {1.0f,  0.0f,  0.0f}}

#define EXT_AHRS_STORED_BIAS {-0.984146595, 1.359451293, 0.020426832}

#define MPU6500_BIAS_DATA_ID 0x0001

#define GIMBAL_YAW_FRONT_ANGLE_RAW 6785
#define GIMBAL_PITCH_FRONT_ANGLE_RAW 2486

#define MPU6500_STORED_GYRO_BIAS {-0.566247761, -0.634880125, 0.307405889}

/// Gimbal and Shoot PID Parameters
#define GIMBAL_PID_YAW_A2V_KP 13.0f
#define GIMBAL_PID_YAW_A2V_KI 0.0f
#define GIMBAL_PID_YAW_A2V_KD 0.1f
#define GIMBAL_PID_YAW_A2V_I_LIMIT 0.0f
#define GIMBAL_PID_YAW_A2V_OUT_LIMIT 450.0f
#define GIMBAL_PID_YAW_A2V_PARAMS \
    {GIMBAL_PID_YAW_A2V_KP, GIMBAL_PID_YAW_A2V_KI, GIMBAL_PID_YAW_A2V_KD, \
    GIMBAL_PID_YAW_A2V_I_LIMIT, GIMBAL_PID_YAW_A2V_OUT_LIMIT}

#define GIMBAL_PID_YAW_V2I_KP 21.0f
#define GIMBAL_PID_YAW_V2I_KI 1.25f
#define GIMBAL_PID_YAW_V2I_KD 0.0f
#define GIMBAL_PID_YAW_V2I_I_LIMIT 3000.0f
#define GIMBAL_PID_YAW_V2I_OUT_LIMIT 20000.0f
#define GIMBAL_PID_YAW_V2I_PARAMS \
    {GIMBAL_PID_YAW_V2I_KP, GIMBAL_PID_YAW_V2I_KI, GIMBAL_PID_YAW_V2I_KD, \
    GIMBAL_PID_YAW_V2I_I_LIMIT, GIMBAL_PID_YAW_V2I_OUT_LIMIT}

#define GIMBAL_PID_PITCH_A2V_KP 14.0f
#define GIMBAL_PID_PITCH_A2V_KI 0.0f
#define GIMBAL_PID_PITCH_A2V_KD 0.05f
#define GIMBAL_PID_PITCH_A2V_I_LIMIT 0.0f
#define GIMBAL_PID_PITCH_A2V_OUT_LIMIT 60.0f
#define GIMBAL_PID_PITCH_A2V_PARAMS \
    {GIMBAL_PID_PITCH_A2V_KP, GIMBAL_PID_PITCH_A2V_KI, GIMBAL_PID_PITCH_A2V_KD, \
    GIMBAL_PID_PITCH_A2V_I_LIMIT, GIMBAL_PID_PITCH_A2V_OUT_LIMIT}

#define GIMBAL_PID_PITCH_V2I_KP 27.0f
#define GIMBAL_PID_PITCH_V2I_KI 0.3f
#define GIMBAL_PID_PITCH_V2I_KD 0.0f
#define GIMBAL_PID_PITCH_V2I_I_LIMIT 5000.0f
#define GIMBAL_PID_PITCH_V2I_OUT_LIMIT 20000.0f
#define GIMBAL_PID_PITCH_V2I_PARAMS \
    {GIMBAL_PID_PITCH_V2I_KP, GIMBAL_PID_PITCH_V2I_KI, GIMBAL_PID_PITCH_V2I_KD, \
    GIMBAL_PID_PITCH_V2I_I_LIMIT, GIMBAL_PID_PITCH_V2I_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_A2V_KP 10.0f  // a number large enough, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_KI 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT 360.0f  // will be replaced, see shoot speed note at ShootSKD
#define SHOOT_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_V2I_KP 50.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KI 1.3f
#define SHOOT_PID_BULLET_LOADER_V2I_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT 8000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT 8000.0f
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
#define SENTRY_CHASSIS_PID_V2I_KP 40.0f
#define SENTRY_CHASSIS_PID_V2I_KI 0.25f
#define SENTRY_CHASSIS_PID_V2I_KD 0.22f
#define SENTRY_CHASSIS_PID_V2I_I_LIMIT 1000.0f
#define SENTRY_CHASSIS_PID_V2I_OUT_LIMIT 8000.0f
#define CHASSIS_PID_V2I_PARAMS \
    {SENTRY_CHASSIS_PID_V2I_KP, SENTRY_CHASSIS_PID_V2I_KI, SENTRY_CHASSIS_PID_V2I_KD, \
    SENTRY_CHASSIS_PID_V2I_I_LIMIT, SENTRY_CHASSIS_PID_V2I_OUT_LIMIT}

#define SENTRY_CHASSIS_PID_A2V_KP 2.95f
#define SENTRY_CHASSIS_PID_A2V_KI 0.0f
#define SENTRY_CHASSIS_PID_A2V_KD 0.0f
#define SENTRY_CHASSIS_PID_A2V_I_LIMIT 0.0f

#define SENTRY_CHASSIS_PID_P2V_KP 30.0f
#define SENTRY_CHASSIS_PID_P2V_KI 0.0f
#define SENTRY_CHASSIS_PID_P2V_KD 0.0f
#define SENTRY_CHASSIS_PID_P2V_I_LIMIT 0.0f

#define CRUISING_SPEED 30.0f

#define CHASSIS_PID_A2V_PARAMS \
    {SENTRY_CHASSIS_PID_A2V_KP, SENTRY_CHASSIS_PID_A2V_KI, SENTRY_CHASSIS_PID_A2V_KD, \
    SENTRY_CHASSIS_PID_A2V_I_LIMIT, CRUISING_SPEED}


#define POM_PID_P2V_PARAMS \
    {SENTRY_CHASSIS_PID_P2V_KP, SENTRY_CHASSIS_PID_P2V_KI, SENTRY_CHASSIS_PID_P2V_KD, \
    SENTRY_CHASSIS_PID_P2V_I_LIMIT, 25.0f}

/// Thread Priority List
#define THREAD_CAN1_PRIO                    (HIGHPRIO - 1)
#define THREAD_CAN2_PRIO                    (HIGHPRIO - 2)
#define THREAD_MPU_PRIO                     (HIGHPRIO - 3)
#define THREAD_IST_PRIO                     (HIGHPRIO - 4)
#define THREAD_AHRS_PRIO                    (HIGHPRIO - 5)
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
#define THREAD_BUZZER_SKD_PRIO              (LOWPRIO + 1)

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

#define USER_CLIENT_ACQUIRED_BULLET_NUM             1
#define USER_CLIENT_ACTUAL_POWER_NUM                2
#define USER_CLIENT_SUPER_CAPACITOR_VOLTAGE_NUM     3

#endif //META_INFANTRY_VEHICLE_SENTRY_H

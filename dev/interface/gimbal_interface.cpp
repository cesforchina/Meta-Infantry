//
// Created by liuzikai on 2018-12-29.
// Zhu Kerui wrote code about processing gimbal feedback.
// Feng Chuhao wrote code about sending gimbal currents.
// Mo Kanya wrote code about friction wheel's feedback function and current sending function
// Qian Chen wrote code about CAN channel distributing mechanism.
//

/**
 * @file    gimbal_interface.cpp
 * @brief   Interface to interact with low level driver of gimbal, including processing chassis motor feedback and
 *          sending target currents.
 *
 * @addtogroup gimbal
 * @{
 */

#include "gimbal_interface.h"
#include "common_macro.h"

GimbalIF::motor_feedback_t GimbalIF::feedback[MOTOR_COUNT];
int GimbalIF::target_current[MOTOR_COUNT] = {0, 0, 0};
//float GimbalIF::fw_duty_cycle = 0.0f;
CANInterface *GimbalIF::can1_ = nullptr;
CANInterface *GimbalIF::can2_ = nullptr;

//const PWMConfig FRICTION_WHEELS_PWM_CFG = {
//        50000,   // frequency
//        1000,    // period
//        nullptr, // callback
//        {
//                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH0
//                {PWM_OUTPUT_ACTIVE_HIGH, nullptr}, // CH1
//                {PWM_OUTPUT_DISABLED, nullptr},    // CH2
//                {PWM_OUTPUT_DISABLED, nullptr}     // CH3
//        },
//        0,
//        0
//};

void GimbalIF::init(CANInterface *can1_interface, CANInterface *can2_interface, uint16_t yaw_front_angle_raw, uint16_t pitch_front_angle_raw,
                    motor_feedback_t::can_channel_t yaw_channel , motor_feedback_t::can_channel_t bullet_channel,
                    motor_feedback_t::can_channel_t plate_channel,
                    motor_type_t yaw_type, motor_type_t pitch_type, motor_type_t bullet_type, motor_type_t fw_type, motor_type_t plate_type) {

    feedback[YAW].id = YAW;
    feedback[YAW].type = yaw_type;
    feedback[YAW].last_angle_raw = yaw_front_angle_raw;
    feedback[YAW].set_can_channel(yaw_channel);

    feedback[PITCH].id = PITCH;
    feedback[PITCH].type = pitch_type;
    feedback[PITCH].last_angle_raw = pitch_front_angle_raw;
    feedback[PITCH].set_can_channel(motor_feedback_t::can_channel_1);

    feedback[BULLET].id = BULLET;
    feedback[BULLET].type = bullet_type;
    feedback[BULLET].reset_front_angle();
    feedback[BULLET].set_can_channel(bullet_channel);

    feedback[PLATE].id = PLATE;
    feedback[PLATE].type = plate_type;
    feedback[PLATE].reset_front_angle();
    feedback[PLATE].set_can_channel(plate_channel);

    feedback[FW_LEFT].id = FW_LEFT;
    feedback[FW_LEFT].type = fw_type;
    feedback[FW_LEFT].reset_front_angle();
    feedback[FW_LEFT].set_can_channel(motor_feedback_t::can_channel_1);

    feedback[FW_RIGHT].id = FW_RIGHT;
    feedback[FW_RIGHT].type = fw_type;
    feedback[FW_RIGHT].reset_front_angle();
    feedback[FW_RIGHT].set_can_channel(motor_feedback_t::can_channel_1);

    can1_ = can1_interface;
    can2_ = can2_interface;

    // Start receiving messages in two can channels. For can1, we need to receive for motor[1-6] to cover all the motors
    // For can2, we only need to receive motor[1-4]. 5-8 is for chassis motor.
    // If a motor is on can2 (physically!), it will be heard on can2. Or if it is on can1, it will heard on can1.

    // TODO: Simplify the feedback code, as now we need two function to prevent multiple process.

    can1_->register_callback(0x201, 0x206, process_gimbal_can1_feedback);
    can2_->register_callback(0x201, 0x204, process_gimbal_can2_feedback);

    // Enable PWM and perform initialization on friction wheels

//    pwmStart(&PWMD8, &FRICTION_WHEELS_PWM_CFG);

//    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
//    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 1 * 500 + 500));
//    chThdSleep(TIME_MS2I(500));

//    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));
//    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, 0 * 500 + 500));

}

void GimbalIF::send_gimbal_currents() {

    CANTxFrame can_1_txmsg;
    CANTxFrame can_2_txmsg;

    // Fill the header
    can_1_txmsg.IDE = can_2_txmsg.IDE = CAN_IDE_STD;
    can_1_txmsg.SID = can_2_txmsg.SID = 0x200;
    can_1_txmsg.RTR = can_2_txmsg.RTR = CAN_RTR_DATA;
    can_1_txmsg.DLC = can_2_txmsg.DLC = 0x08;

    // Fill the current of Yaw
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[YAW], GIMBAL_INTERFACE_MAX_CURRENT);
#endif
    if(feedback[YAW].canChannel == motor_feedback_t::can_channel_1) {
        if (feedback[YAW].type != RM6623) {
            can_1_txmsg.data8[0] = (uint8_t) (target_current[YAW] >> 8); // upper byte
            can_1_txmsg.data8[1] = (uint8_t) target_current[YAW];        // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_1_txmsg.data8[0] = (uint8_t) (-target_current[YAW] >> 8); // upper byte
            can_1_txmsg.data8[1] = (uint8_t) -target_current[YAW];        // lower byte
        }
    } else if (feedback[YAW].canChannel == motor_feedback_t::can_channel_2){
        if (feedback[YAW].type != RM6623) {
            can_2_txmsg.data8[0] = (uint8_t) (target_current[YAW] >> 8); // upper byte
            can_2_txmsg.data8[1] = (uint8_t) target_current[YAW];        // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_2_txmsg.data8[0] = (uint8_t) (-target_current[YAW] >> 8); // upper byte
            can_2_txmsg.data8[1] = (uint8_t) -target_current[YAW];        // lower byte
        }
    }


    // Fill the current of Pitch
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PITCH], GIMBAL_INTERFACE_MAX_CURRENT);
#endif
    if(feedback[PITCH].canChannel == motor_feedback_t::can_channel_1) {
        if (feedback[PITCH].type != RM6623) {
            can_1_txmsg.data8[2] = (uint8_t) (target_current[PITCH] >> 8); // upper byte
            can_1_txmsg.data8[3] = (uint8_t) target_current[PITCH];        // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_1_txmsg.data8[2] = (uint8_t) (-target_current[PITCH] >> 8); // upper byte
            can_1_txmsg.data8[3] = (uint8_t) -target_current[PITCH];        // lower byte
        }
    } else if (feedback[PITCH].canChannel == motor_feedback_t::can_channel_2) {
        if (feedback[PITCH].type != RM6623) {
            can_2_txmsg.data8[2] = (uint8_t) (target_current[PITCH] >> 8); // upper byte
            can_2_txmsg.data8[3] = (uint8_t) target_current[PITCH];        // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_2_txmsg.data8[2] = (uint8_t) (-target_current[PITCH] >> 8); // upper byte
            can_2_txmsg.data8[3] = (uint8_t) -target_current[PITCH];        // lower byte
        }
    }

    // Fill the current of bullet loader
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[BULLET], GIMBAL_INTERFACE_BULLET_LOADER_MAX_CURRENT);
#endif
    if(feedback[BULLET].canChannel == motor_feedback_t::can_channel_1) {
        if (feedback[BULLET].type != RM6623) {
            can_1_txmsg.data8[4] = (uint8_t) (target_current[BULLET] >> 8); // upper byte
            can_1_txmsg.data8[5] = (uint8_t) target_current[BULLET];       // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_1_txmsg.data8[4] = (uint8_t) (-target_current[BULLET] >> 8); // upper byte
            can_1_txmsg.data8[5] = (uint8_t) -target_current[BULLET];       // lower byte
        }
    } else if (feedback[BULLET].canChannel == motor_feedback_t::can_channel_2) {
        if (feedback[BULLET].type != RM6623) {
            can_2_txmsg.data8[4] = (uint8_t) (target_current[BULLET] >> 8); // upper byte
            can_2_txmsg.data8[5] = (uint8_t) target_current[BULLET];       // lower byte
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_2_txmsg.data8[4] = (uint8_t) (-target_current[BULLET] >> 8); // upper byte
            can_2_txmsg.data8[5] = (uint8_t) -target_current[BULLET];       // lower byte
        }
    }
    // Fill the current of plate
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PLATE], GIMBAL_INTERFACE_BULLET_PLATE_MAX_CURRENT);
#endif
    if(feedback[PLATE].canChannel == motor_feedback_t::can_channel_1) {
        if (feedback[PLATE].type != RM6623) {
            can_1_txmsg.data8[6] = (uint8_t) (target_current[PLATE] >> 8);
            can_1_txmsg.data8[7] = (uint8_t) target_current[PLATE];
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_1_txmsg.data8[6] = (uint8_t) (-target_current[PLATE] >> 8);
            can_1_txmsg.data8[7] = (uint8_t) -target_current[PLATE];
        }
    } else if(feedback[PLATE].canChannel == motor_feedback_t::can_channel_2) {
        if (feedback[PLATE].type != RM6623) {
            can_2_txmsg.data8[6] = (uint8_t) (target_current[PLATE] >> 8);
            can_2_txmsg.data8[7] = (uint8_t) target_current[PLATE];
        } else {
            /**
             * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
             *       direction. In order to unified coordinate system, minus sign is applied here.
             */
            can_2_txmsg.data8[6] = (uint8_t) (-target_current[PLATE] >> 8);
            can_2_txmsg.data8[7] = (uint8_t) -target_current[PLATE];
        }
    }


    can1_->send_msg(&can_1_txmsg);
    can2_->send_msg(&can_2_txmsg);

    // Set the PWM of friction wheels
//    pwmEnableChannel(&PWMD8, FW_LEFT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));
//    pwmEnableChannel(&PWMD8, FW_RIGHT, PWM_PERCENTAGE_TO_WIDTH(&PWMD8, fw_duty_cycle * 500 + 500));

//    LOG("FW %f", fw_duty_cycle);

}

void GimbalIF::send_fw_currents() {

    CANTxFrame txmsg;

    // Fill the header
    txmsg.IDE = CAN_IDE_STD;
    txmsg.SID = 0x1FF;
    txmsg.RTR = CAN_RTR_DATA;
    txmsg.DLC = 0x08;

    // Fill the current of fw_left
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[YAW], GIMBAL_INTERFACE_MAX_CURRENT);
#endif

    if (feedback[FW_LEFT].type != RM6623) {
        txmsg.data8[0] = (uint8_t) (target_current[FW_LEFT] >> 8); // upper byte
        txmsg.data8[1] = (uint8_t) target_current[FW_LEFT];        // lower byte
    } else {
        /**
         * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
         *       direction. In order to unified coordinate system, minus sign is applied here.
         */
        txmsg.data8[0] = (uint8_t) (-target_current[FW_LEFT] >> 8); // upper byte
        txmsg.data8[1] = (uint8_t) -target_current[FW_LEFT];        // lower byte
    }

    // Fill the current of fw_right
#if GIMBAL_INTERFACE_ENABLE_CLIP
    ABS_CROP(target_current[PITCH], GIMBAL_INTERFACE_MAX_CURRENT);
#endif
    if (feedback[FW_RIGHT].type != RM6623) {
        txmsg.data8[2] = (uint8_t) (target_current[FW_RIGHT] >> 8); // upper byte
        txmsg.data8[3] = (uint8_t) target_current[FW_RIGHT];        // lower byte
    } else {
        /**
         * @note Viewing from the top of 6623, angle use CCW as positive direction, while current use CW as positive
         *       direction. In order to unified coordinate system, minus sign is applied here.
         */
        txmsg.data8[2] = (uint8_t) (-target_current[FW_RIGHT] >> 8); // upper byte
        txmsg.data8[3] = (uint8_t) -target_current[FW_RIGHT];        // lower byte
    }

    can1_->send_msg(&txmsg);

}

void GimbalIF::process_gimbal_can1_feedback(CANRxFrame const *rxmsg) {

    /**
     * Function logic description:
     *  1. First, get the absolute angle value from the motor, compared with the last absolute angle value, get the
     *     angle movement
     *  2. Add the angle movement with the relative angle, get the new relative angle, modify the relative angle and
     *     the base round value
     */

    /// Get new absolute angle value of motor
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) return;

    motor_feedback_t* fb = &feedback[(motor_id_t) (rxmsg->SID - 0x201)];

    // To check whether the feedback frame belongs to gimbal. (For example, feedback ID is 1-4, but it is from can2 while gimbal motor is can1)
    if(fb->canChannel!=fb->can_channel_1) return;

    /// Calculate the angle movement in raw data
    // KEY IDEA: add the change of angle to actual angle
    // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
    int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

    // Store new_actual_angle_raw for calculation of angle_movement next time
    fb->last_angle_raw = new_actual_angle_raw;

    /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point
    if (angle_movement < -4096) angle_movement += 8192;
    if (angle_movement > 4096) angle_movement -= 8192;

    switch (fb->type) {

        case RM6623:  // RM6623 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);
            
            break;

        case M2006:  // M2006 deceleration ratio = 36,

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.001220703f;  // * 360 / 8192 / 36

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.166666667f;  // 360 / 60 / 36

            fb->actual_current = 0;  // no current feedback available

            break;

        case M3508:  // M3508 deceleration ratio = 3591/187

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.002288436f; // 360 / 8192 / (3591/187)

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity =
                    ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.312447786f;  // 360 / 60 / (3591/187)

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);

            break;

        case GM6020:  // GM6020 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            // rpm -> degree/s
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 6.0f;  // 360 / 60

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);
            
            break;

        case GM3510:  // GM3510 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = 0;  // no current feedback available

        default:
            break;
    }

    /// Normalize the angle to [-180, 180]
    // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
    if (fb->actual_angle >= 180.0f) {
        fb->actual_angle -= 360.0f;
        fb->round_count++;
    }
    if (fb->actual_angle < -180.0f) {
        fb->actual_angle += 360.0f;
        fb->round_count--;
    }

    fb->last_update_time = SYSTIME;

}

void GimbalIF::process_gimbal_can2_feedback(CANRxFrame const *rxmsg) {

    /**
     * Function logic description:
     *  1. First, get the absolute angle value from the motor, compared with the last absolute angle value, get the
     *     angle movement
     *  2. Add the angle movement with the relative angle, get the new relative angle, modify the relative angle and
     *     the base round value
     */

    /// Get new absolute angle value of motor
    uint16_t new_actual_angle_raw = (rxmsg->data8[0] << 8 | rxmsg->data8[1]);

    // Check whether this new raw angle is valid
    if (new_actual_angle_raw > 8191) return;

    motor_feedback_t* fb = &feedback[(motor_id_t) (rxmsg->SID - 0x201)];

    // To check whether the feedback frame belongs to gimbal. (For example, feedback ID is 1-4, but it is from can1 while gimbal motor is can2)
    if(fb->canChannel!=fb->can_channel_2) return;

    /// Calculate the angle movement in raw data
    // KEY IDEA: add the change of angle to actual angle
    // We assume that the absolute value of the angle movement is smaller than 180 degrees (4096 of raw data)
    int angle_movement = (int) new_actual_angle_raw - (int) fb->last_angle_raw;

    // Store new_actual_angle_raw for calculation of angle_movement next time
    fb->last_angle_raw = new_actual_angle_raw;

    /// If angle_movement is too extreme between two samples, we grant that it's caused by moving over the 0(8192) point
    if (angle_movement < -4096) angle_movement += 8192;
    if (angle_movement > 4096) angle_movement -= 8192;

    switch (fb->type) {

        case RM6623:  // RM6623 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = (int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3]);

            break;

        case M2006:  // M2006 deceleration ratio = 36,

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.001220703f;  // * 360 / 8192 / 36

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.166666667f;  // 360 / 60 / 36

            fb->actual_current = 0;  // no current feedback available

            break;

        case M3508:  // M3508 deceleration ratio = 3591/187

            // raw -> degree with deceleration ratio
            fb->actual_angle += angle_movement * 0.002288436f; // 360 / 8192 / (3591/187)

            // rpm -> degree/s with deceleration ratio
            fb->actual_velocity =
                    ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 0.312447786f;  // 360 / 60 / (3591/187)

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);

            break;

        case GM6020:  // GM6020 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            // rpm -> degree/s
            fb->actual_velocity = ((int16_t) (rxmsg->data8[2] << 8 | rxmsg->data8[3])) * 6.0f;  // 360 / 60

            fb->actual_current = (int16_t) (rxmsg->data8[4] << 8 | rxmsg->data8[5]);

            break;

        case GM3510:  // GM3510 deceleration ratio = 1

            // raw -> degree
            fb->actual_angle += angle_movement * 0.043945312f;  // * 360 / 8192

            fb->actual_velocity = 0;  // no velocity feedback available

            fb->actual_current = 0;  // no current feedback available

        default:
            break;
    }

    /// Normalize the angle to [-180, 180]
    // If the actual_angle is greater than 180(-180) then it turns a round in CCW(CW) direction
    if (fb->actual_angle >= 180.0f) {
        fb->actual_angle -= 360.0f;
        fb->round_count++;
    }
    if (fb->actual_angle < -180.0f) {
        fb->actual_angle += 360.0f;
        fb->round_count--;
    }

    fb->last_update_time = SYSTIME;

}

void GimbalIF::motor_feedback_t::init(motor_type_t type_, motor_id_t id_) {
    type = type_;
    id = id_;
}

void GimbalIF::motor_feedback_t::reset_front_angle() {
    actual_angle = 0;
    round_count = 0;
}

float GimbalIF::motor_feedback_t::accumulated_angle() {
    return actual_angle + round_count * 360.0f;
}

void GimbalIF::motor_feedback_t::set_can_channel(can_channel_t can_channel_) {
    canChannel = can_channel_;
}

/** @} */
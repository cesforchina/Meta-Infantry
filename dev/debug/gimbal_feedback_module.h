//
// Created by liuzikai on 2018-12-30.
//

#ifndef META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H
#define META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H

#include "ch.hpp"
#include "hal.h"
#include "serial_shell.h"
#include "gimbal_interface.h"
#include "mpu6500.h"

/**
 * This class holds a thread to send gimbal data through shell
 * Time interval: 200ms
 * Feedback format: see gimbal_feedback_module.cpp
 */
class GimbalFeedbackModule : private chibios_rt::BaseStaticThread<512> {

public:

    /**
     * @brief wrapper function to start the thread
     * @param prio
     */
    void start_thread(tprio_t prio) {
        start(prio);
    }

    /**
     * @brief initialize with pointers to target value variables
     * @note when testing gimbal interface, target angle and velocity can be some non-changed variables
     * @param p_yaw_target_angle
     * @param p_yaw_target_velocity
     * @param p_yaw_target_current
     * @param p_pitch_target_angle
     * @param p_pitch_target_velocity
     * @param p_pitch_target_current
     */
    GimbalFeedbackModule(int interval,
                         float *p_yaw_target_angle,
                         float *p_yaw_target_velocity,
                         int *p_yaw_target_current,
                         float *p_pitch_target_angle,
                         float *p_pitch_target_velocity,
                         int *p_pitch_target_current,
                         float *p_bullet_loader_target_angle,
                         float *p_bullet_loader_target_velocity,
                         int *p_bullet_loader_target_current) {
        feedback_interval = interval;
        ptr_yaw_target_angle = p_yaw_target_angle;
        ptr_yaw_target_velocity = p_yaw_target_velocity;
        ptr_yaw_target_current = p_yaw_target_current;
        ptr_pitch_target_angle = p_pitch_target_angle;
        ptr_pitch_target_velocity = p_pitch_target_velocity;
        ptr_pitch_target_current = p_pitch_target_current;
        ptr_bullet_loader_target_angle = p_bullet_loader_target_angle;
        ptr_bullet_loader_target_velocity = p_bullet_loader_target_velocity;
        ptr_bullet_loader_target_current = p_bullet_loader_target_current;

        enable_yaw_feedback = false;
        enable_pitch_feedback = false;
        enable_bullet_loader_feedback = false;
    }

    int feedback_interval; // ms

    bool enable_yaw_feedback;
    bool enable_pitch_feedback;
    bool enable_bullet_loader_feedback;

private:

    void main() override;

    float *ptr_yaw_target_angle;
    float *ptr_yaw_target_velocity;
    int *ptr_yaw_target_current;

    float *ptr_pitch_target_angle;
    float *ptr_pitch_target_velocity;
    int *ptr_pitch_target_current;

    float *ptr_bullet_loader_target_angle;
    float *ptr_bullet_loader_target_velocity;
    int *ptr_bullet_loader_target_current;
};


#endif //META_INFANTRY_GIMBAL_FEEDBACK_MODULE_H

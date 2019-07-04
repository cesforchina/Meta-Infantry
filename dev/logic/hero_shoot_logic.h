//
// Created by 钱晨 on 2019-07-03.
//

#ifndef META_INFANTRY_HERO_SHOOT_LOGIC_H
#define META_INFANTRY_HERO_SHOOT_LOGIC_H

/**
 * @file    hero_shoot_logic.h
 * @brief   Logic-level module to control shooter.
 *
 * @addtogroup shoot
 * @{
 */

#include "ch.hpp"
#include "shell.h"
#include "shoot_scheduler.h"

class HeroShootLG {
public:

    static void init(float loader_angle_per_bullet_, float plate_angle_per_bullet_, tprio_t stuck_detector_thread_prio, tprio_t automatic_thread_prio);

    static void shoot();
    /**
 * Set friction wheels duty cycle in LIMITED_SHOOTING_MODE or REVERSE_TURNING_MODE
 * @param duty_cycle  Friction wheels duty cycle, from 0 to 1.0
 */
    static void set_friction_wheels(float duty_cycle);

    /**
     * Get friction wheels duty cycle
     * @return Friction wheels duty cycle, from 0 to 1.0
     */
    static float get_friction_wheels_duty_cycle();

    enum loader_state_t{
        LOADING,
        STOP,
        STUCK
    };

    static loader_state_t loaderState;
    static loader_state_t plateState;

private:

    static int load_bullet_count;

    static int loader_angle_per_bullet;
    static int plate_angle_per_bullet;

    static bool loaded_bullet[4];

    class StuckDetectorThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    class AutomateThread : public chibios_rt::BaseStaticThread<512> {
        void main() final;
    };

    static constexpr int LOADER_STUCK_THRESHOLD_CURRENT = 1500;
    static constexpr int LOADER_STUCK_THRESHOLD_VELOCITY = 2;

    static constexpr int PLATE_STUCK_THRESHOLD_CURRENT = 3000;
    static constexpr int PLATE_STUCK_THRESHOLD_VELOCITY = 2;

    static StuckDetectorThread stuckDetector;
    static AutomateThread automation;

};


#endif //META_INFANTRY_HERO_SHOOT_LOGIC_H
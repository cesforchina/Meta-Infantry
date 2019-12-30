//
// Created by ... on YYYY/MM/DD.
//

/**
 * This file contain ... Unit Test.
 */

#include "ch.hpp"
#include "hal.h"

#include "led.h"
#include "buzzer_scheduler.h"
#include "debug/shell/shell.h"
#include "interface/gimbal_interface.h"
#include "pid_controller.hpp"

// Other headers here

using namespace chibios_rt;

CANInterface can1(&CAND1);
CANInterface can2(&CAND2);

#define SHOOT_PID_BULLET_LOADER_A2V_KP 8.5f
#define SHOOT_PID_BULLET_LOADER_A2V_KI 0.0f
#define SHOOT_PID_BULLET_LOADER_A2V_KD 0.18f
#define SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT 720.0f
#define SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT 720.0f
#define SHOOT_PID_BULLET_LOADER_A2V_PARAMS \
    {SHOOT_PID_BULLET_LOADER_A2V_KP, SHOOT_PID_BULLET_LOADER_A2V_KI, SHOOT_PID_BULLET_LOADER_A2V_KD, \
    SHOOT_PID_BULLET_LOADER_A2V_I_LIMIT, SHOOT_PID_BULLET_LOADER_A2V_OUT_LIMIT}

#define SHOOT_PID_BULLET_LOADER_V2I_KP 18.0f
#define SHOOT_PID_BULLET_LOADER_V2I_KI 0.1f
#define SHOOT_PID_BULLET_LOADER_V2I_KD 0.0f
#define SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT 3000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT 5000.0f
#define SHOOT_PID_BULLET_LOADER_V2I_PARAMS \
    {SHOOT_PID_BULLET_LOADER_V2I_KP, SHOOT_PID_BULLET_LOADER_V2I_KI, SHOOT_PID_BULLET_LOADER_V2I_KD, \
    SHOOT_PID_BULLET_LOADER_V2I_I_LIMIT, SHOOT_PID_BULLET_LOADER_V2I_OUT_LIMIT}


// Thread to ...
class motor_controller_thread : public BaseStaticThread <512> {
public:
    PIDController m1_velocity_to_current;
    PIDController m1_angle_to_velocity;

    PIDController m2_velocity_to_current;
    PIDController m2_angle_to_velocity;

    float target_angle;
private:
    float m1_target_velocity;
    int m1_target_current;

    float m2_target_velocity;
    int m2_target_current;
    void main() final {
        setName("motor_controller");
        while (!shouldTerminate()) {
            m1_target_velocity = m1_angle_to_velocity.calc(GimbalIF::feedback[2].accumulated_angle(), target_angle);
            m1_target_current = (int) m1_velocity_to_current.calc(GimbalIF::feedback[2].actual_velocity, m1_target_velocity);
            GimbalIF::target_current[2] = m1_target_current;

            m2_target_velocity = m2_angle_to_velocity.calc(GimbalIF::feedback[1].accumulated_angle(), -target_angle);
            m2_target_current = (int) m2_velocity_to_current.calc(GimbalIF::feedback[1].actual_velocity, m2_target_velocity);
            GimbalIF::target_current[1] = m2_target_velocity;

            GimbalIF::send_gimbal_currents();
            sleep(TIME_MS2I(5));
        }
    }
} motorControllerThread;

/**
 * @brief set enabled state of yaw and pitch motor
 * @param chp
 * @param argc
 * @param argv
 */
static void cmd_set_angle(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 1) {
        shellUsage(chp, "set_target_angle angle[0-180]");
        return;
    }
    GimbalIF::feedback[2].reset_front_angle();
    motorControllerThread.target_angle = Shell::atof(argv[0]);
}

static void cmd_play_sound(BaseSequentialStream *chp, int argc, char *argv[]) {
    (void) argv;
    if (argc != 0) {
        shellUsage(chp, "play_sound");
        return;
    }
    BuzzerSKD::play_sound(BuzzerSKD::sound_kong_fu_FC);
}


// Shell commands to ...
ShellCommand templateShellCommands[] = {
        {"set_target_angle", cmd_set_angle},
        {"play_sound", cmd_play_sound},
        {nullptr,    nullptr}
};

int main(void) {
    halInit();
    System::init();


    // Start ChibiOS shell at high priority, so even if a thread stucks, we still have access to shell.
    Shell::start(HIGHPRIO);
    Shell::addCommands(templateShellCommands);

    can1.start(HIGHPRIO);
    can2.start(HIGHPRIO-1);

    GimbalIF::init(&can1, &can2, 0, 0,
            GimbalIF::motor_feedback_t::none, GimbalIF::motor_feedback_t::can_channel_1, GimbalIF::motor_feedback_t::none,
            GimbalIF::NONE_MOTOR, GimbalIF::M2006, GimbalIF::M2006, GimbalIF::NONE_MOTOR);

    motorControllerThread.m1_velocity_to_current.change_parameters(SHOOT_PID_BULLET_LOADER_V2I_PARAMS);
    motorControllerThread.m1_angle_to_velocity.change_parameters(SHOOT_PID_BULLET_LOADER_A2V_PARAMS);

    motorControllerThread.m2_velocity_to_current.change_parameters(SHOOT_PID_BULLET_LOADER_V2I_PARAMS);
    motorControllerThread.m2_angle_to_velocity.change_parameters(SHOOT_PID_BULLET_LOADER_A2V_PARAMS);

    motorControllerThread.start(NORMALPRIO + 1);
    motorControllerThread.target_angle;

    BuzzerSKD::init(LOWPRIO);
    BuzzerSKD::play_sound(BuzzerSKD::sound_kong_fu_FC);
#if CH_CFG_NO_IDLE_THREAD // see chconf.h for what this #define means
    // ChibiOS idle thread has been disabled, main() should implement infinite loop
    while (true) {}
#else
    // When main() quits, the main thread will somehow enter an infinite loop, so we set the priority to lowest
    // before quitting, to let other threads run normally
    BaseThread::setPriority(1);
#endif
    return 0;
}

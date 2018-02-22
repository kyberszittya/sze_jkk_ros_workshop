#include "ev3_controller.hpp"

void EV3Controller::init()
{
    //drive_servo_ac.init();
    simple_drive_ac.init();
    simple_reverse_ac.init();
    grip_simple_ac.init();
    driveuntildist_ac.init();
}
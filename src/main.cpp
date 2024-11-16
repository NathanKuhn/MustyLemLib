#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/odom.hpp" // IWYU pragma: keep

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    lemlib::init(); // initialize lemlib
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {}

/**
 * Runs in driver control
 */
void opcontrol() {
    pros::lcd::print(0, "Pose: (%f, %f, %f)", lemlib::getPose().x, lemlib::getPose().y, lemlib::getPose().theta);
}

#include "main.h"
#include "lemlib/api.hpp"

intakeState inState = intakeState::STOP;
alliance color = alliance::RED;

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup left({-5, -6, 7}, pros::MotorGearset::blue);
pros::MotorGroup right({-1, 2, 3}, pros::MotorGearset::blue);

pros::Motor intake(-11);
pros::Motor ws(18);

pros::Optical optical(21);//update port when run

pros::Rotation wsr(19);

pros::ADIDigitalOut clampPistons('A');
pros::ADIDigitalOut descore('B');

pros::Rotation h(12);
pros::Rotation v(13);
pros::Imu imu(9);

pros::Distance distanceSensor(17);

lemlib::TrackingWheel htw(&h, lemlib::Omniwheel::NEW_275, -3.687);
lemlib::TrackingWheel vtw(&v, 2, -0.465);

lemlib::Drivetrain drivetrain(&left, &right, 11.5, lemlib::Omniwheel::NEW_325, 450, 2);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              54, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3.4, // proportional gain (kP)
                                              0.01, // integral gain (kI)
                                              21, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// lemlib::OdomSensors sensors(&vtw, // vertical tracking wheel 1, set to null
//                             nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
//                             &htw, // horizontal tracking wheel 1
//                             nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
//                             &imu // inertial sensor
// );
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(10, // joystick deadband out of 127
                                     15, // minimum output where drivetrain will move out of 127
                                     1.03 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(10, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.03 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors,
                        &throttle_curve,
                        &steer_curve // odometry sensors
);
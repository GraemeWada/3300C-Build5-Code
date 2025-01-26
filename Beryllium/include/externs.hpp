#include "lemlib/api.hpp"

extern pros::Controller controller;

extern pros::MotorGroup left;
extern pros::MotorGroup right;

extern pros::Motor intake;
extern pros::Motor ws;

extern pros::Rotation wsr;

extern pros::ADIDigitalOut clampPistons;
extern pros::ADIDigitalOut descore;

extern pros::Rotation h;
extern pros::Rotation v;
extern pros::Imu imu;

extern pros::Distance distanceSensor;

extern lemlib::Drivetrain drivetrain;

extern lemlib::OdomSensors sensors;

// input curve for throttle input during driver control
extern lemlib::ExpoDriveCurve throttle_curve;

// input curve for steer input during driver control
extern lemlib::ExpoDriveCurve steer_curve;

extern lemlib::Chassis chassis;
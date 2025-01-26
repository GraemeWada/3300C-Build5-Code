#include "lemlib/api.hpp"
#include "main.h"
#include "skills.hpp"

int Intake(bool forward = true){
    if(forward){
        intake.move_voltage(12000);
        return 1;
    } else {
        intake.move_voltage(-12000);
        return -1;
    }
}

int Stop(){
    intake.move_voltage(0);
    return 0;
}

float CLE(float desiredAngle, float settleError = 50){
    static float error = 0;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    if(fabs(error) <= settleError){
        error = 0;
    }
    return error;
}

void skills(){
    //stall detection
    int intakeState = 0;
    bool detectStalls = true;

    //lift PID loop
    //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
    int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        while (true) {
            if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
                pros::delay(300);
                if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
                    intake.move_voltage(intakeState * -1 * 12000);
                    pros::delay(200);
                    intake.move_voltage(intakeState * 12000);
                }
               
            }
            
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(CLE(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(CLE(2400)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(CLE(15500)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(CLE(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(CLE(5000)));
            }
            pros::delay(25);
        }
    });

    chassis.setPose(-60.5, 0, 90);

    //Part one

    //score alliance stake
    intakeState = Intake();
    pros::delay(300);
    intakeState = Intake(false);
    pros::delay(100);
    Stop();
    //grab goal
    chassis.moveToPoint(-48, -24, 2000, {.forwards = false});
    chassis.waitUntilDone();
    clampPistons.set_value(true);
    pros::delay(300);
    intakeState = Intake();
    //grab rings 1 and 2
    chassis.moveToPoint(-24, -24, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(24, -48, 2000);
    chassis.waitUntilDone();
    pros::delay(300);
    //load ring 3
    chassis.moveToPoint(48, -60, 2000);
    pros::delay(500);
    liftState = 1; liftpid.reset(); detectStalls = false;
    chassis.waitUntilDone();
    pros::delay(800);
    intakeState = Stop();
    pros::delay(100);
    //double wall stake
    chassis.moveToPoint(0, -42, 2000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    liftState = 4; liftpid.reset(); detectStalls = true;
    pros::delay(100);
    intakeState = Intake();
    chassis.moveToPoint(0, -70, 1000, {.maxSpeed = 80});
    pros::delay(300);
    intakeState = Stop();
    chassis.waitUntil(20);
    liftState = 3; liftpid.reset();
    pros::delay(500);
}

void skills2(){
    //stall detection
    int intakeState = 0;
    bool detectStalls = true;

    //lift PID loop
    //IMPORTANT: RESET LIFTPID WHEN CHANGING LIFTSTATE!!!
    int liftState = 0;
    ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();
    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    pros::Task lift_task([&]() {
        while (true) {
            if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
                pros::delay(300);
                if(intakeState != 0 && fabs(intake.get_actual_velocity()) < 50 && detectStalls){
                    intake.move_voltage(intakeState * -1 * 12000);
                    pros::delay(200);
                    intake.move_voltage(intakeState * 12000);
                }
               
            }
            
            switch(liftState){
                case 0:
                ws.move_voltage(-liftpid.update(CLE(700, 10)));
                break;
                case 1:
                ws.move_voltage(-liftpid.update(CLE(2400)));
                break;
                case 2:
                ws.move_voltage(-liftpid.update(CLE(18000)));
                break;
                case 3:
                ws.move_voltage(-liftpid.update(CLE(21000)));
                break;
                case 4:
                ws.move_voltage(-liftpid.update(CLE(5000)));
            }
            pros::delay(25);
        }
    });

    chassis.setPose(-60.5, 0, 90);

    //Part one

    //score alliance stake
    intakeState = Intake();
    pros::delay(300);
    intakeState = Intake(false);
    pros::delay(100);
    intakeState = Stop();
    //grab goal
    chassis.moveToPoint(-48, -24, 1500, {.forwards = false});
    chassis.waitUntilDone();
    // pros::delay(100);    
    clampPistons.set_value(true);
    pros::delay(300);
    intakeState = Intake();
    //ring 1
    chassis.moveToPoint(-24,-24, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 2
    chassis.moveToPoint(24, -48, 1500);
    chassis.waitUntilDone();
    //Wall Stake Ring
    chassis.moveToPoint(0, -57, 1500, {.maxSpeed = 90});
    pros::delay(800);
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 1000);
    chassis.moveToPoint(0, -75, 5000, {.maxSpeed = 60});
    pros::delay(500);
    intakeState = Stop();
    liftState = 2; detectStalls = true; liftpid.reset();
    pros::delay(500);
    chassis.cancelMotion();
    chassis.moveToPoint(0, -48, 1000, {.forwards = false});
    chassis.waitUntil(10);
    liftState = 0; liftpid.reset();

    //ring 3
    intakeState = Intake();
    chassis.moveToPoint(-24, -48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 4, 5
    chassis.moveToPoint(-60, -48, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    //ring 6
    chassis.moveToPose(-24, -55, 90, 2500);
    chassis.waitUntilDone();
    pros::delay(200);
    //corner
    chassis.moveToPoint(-65, -65, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intakeState = Intake(false);
    clampPistons.set_value(false);
    pros::delay(500);
    intakeState = Stop();  

    //part 2

    //goal
    chassis.moveToPoint(-48, 0, 2000);
    chassis.waitUntilDone();
    chassis.turnToHeading(180, 800);
    chassis.moveToPoint(-48, 24, 1500, {.forwards = false});
    chassis.waitUntilDone();
    // pros::delay(100);
    clampPistons.set_value(true);
    pros::delay(300);
    intakeState = Intake();
    //ring 1
    chassis.moveToPoint(-24,24, 5000, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 2
    chassis.moveToPoint(-7, 36, 5000, {.minSpeed = 70, .earlyExitRange = 1});
    chassis.waitUntilDone();
    chassis.moveToPoint(27, 48, 5000);
    chassis.waitUntilDone();
    //Wall Stake Ring
    chassis.moveToPoint(0, 53, 5000, {.maxSpeed = 90});
    pros::delay(800);
    liftState = 1; detectStalls = false; liftpid.reset();
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 5000);
    chassis.moveToPoint(0, 75, 5000, {.maxSpeed = 60});
    pros::delay(500);
    intakeState = Stop();
    liftState = 2; detectStalls = true; liftpid.reset();
    pros::delay(500);
    chassis.cancelMotion();
    chassis.moveToPoint(0, 48, 1000, {.forwards = false});
    chassis.waitUntil(10);
    liftState = 0; liftpid.reset();
    //ring 3
    intakeState = Intake();
    chassis.moveToPoint(-24, 48, 1500, {.minSpeed = 70, .earlyExitRange = 2});
    chassis.waitUntilDone();
    //ring 4, 5
    chassis.moveToPoint(-60, 48, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    //ring 6
    chassis.moveToPose(-24, 60, 90, 2500);
    chassis.waitUntilDone();
    pros::delay(200);
    //corner
    chassis.moveToPoint(-65, 65, 1500, {.forwards = false});
    chassis.waitUntilDone();
    intakeState = Intake(false);
    clampPistons.set_value(false);
    pros::delay(500);
    intakeState = Stop();
}

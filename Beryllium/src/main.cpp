#include "main.h"
#include "lemlib/api.hpp"

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void computePossibleLocation(){
    double distance = distanceSensor.get(); //get distance to wall
    distance /= 25.4; //convert to inches

    double originalHeading = chassis.getPose().theta;
    double heading = chassis.getPose().theta + 90; //heading of distsensor
    if(heading >= 360){
        heading -= 360;
    }
    heading *= (3.1415926535/180); // to radians

    double xDist = distance * cos(heading) * -1; //shift for each point 
    double yDist = distance * sin(heading) * -1;
    
    double leftX = -70+xDist;//x and y values for the resulting square
    double rightX = 70+xDist;
    double topY = 70+yDist;
    double bottomY = -70+yDist;

    double currentX = chassis.getPose().x;
    double currentY = chassis.getPose().y;

    double distances[4] = {fabs(currentY - topY), fabs(currentY - bottomY), fabs(currentX - leftX), fabs(currentX - rightX)}; //TBLR
    pros::lcd::set_text(4, std::to_string(distances[0]));
    pros::lcd::set_text(5, std::to_string(distances[1]));
    pros::lcd::set_text(6, std::to_string(distances[2]));
    pros::lcd::set_text(7, std::to_string(distances[3]));
    double minDist = 0;
    int wall = 0;
    for (int i = 0; i < 4; i++){
        if(distances[i] < minDist){
            minDist = distances[i];
            wall = i;
        }
    }
    double xCoordinate;
    double yCoordinate;
    switch(wall){
        case 0: //top
            xCoordinate = currentX;
            yCoordinate = topY;
            break;
        case 1: //bottom
            xCoordinate = currentX;
            yCoordinate = bottomY;
            break;
        case 2: //left
            xCoordinate = leftX;
            yCoordinate = currentY;
            break;
        case 3: //right
            xCoordinate = rightX;
            yCoordinate = currentY;
            break;
    }
    if(fabs(xCoordinate) <= 70 && fabs(yCoordinate <= 70)){
        chassis.setPose(xCoordinate, yCoordinate, originalHeading);
        pros::lcd::set_text(3, std::to_string(wall));
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    // chassis.setPose(0, -48, 90);
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // computePossibleLocation();
            // delay to save resources
            pros::delay(20);
        }
    });
    pros::Task intakeWithSort([&](){
        int degsToReverse = 170;
        int inRotationStart = 0;
        static int redMin = 9;
        static int redMax = 18;
        static int blueMin = 190;
        static int blueMax = 250;
        bool hasSeenRejection = false;
        optical.set_led_pwm(100);

        while(true){
            if(inState == intakeState::IN){
                intake.move_voltage(12000);
                //colour sortign

                if(color == alliance::RED){
                    if(optical.get_hue()>blueMin&& optical.get_hue()<blueMax && !hasSeenRejection ){
                        intake.tare_position();
                        inRotationStart = intake.get_position();
                        hasSeenRejection = true;

                        pros::lcd::print(4, "Proximty %ld", optical.get_proximity()); // heading



                    }
                    if(hasSeenRejection && optical.get_proximity()<=210){
                        intake.move_voltage(-12000);
                        pros::delay(90);
                        hasSeenRejection = false;
                    }
                }
                else if(color == alliance::BLUE){
                    
                    if(optical.get_hue()>redMin&& optical.get_hue()<redMax && !hasSeenRejection ){
                        intake.tare_position();
                        inRotationStart = intake.get_position();
                        hasSeenRejection = true;

                        pros::lcd::print(4, "seen red"); // heading



                    }
                    if(hasSeenRejection && optical.get_proximity()<=210){
                        intake.move_voltage(-12000);
                        pros::delay(90);
                        hasSeenRejection = false;
                    }
                }





            } else if(inState == intakeState::OUT){
                intake.move_voltage(-12000);
            } else {
                intake.move_voltage(0);
                intake.brake();
                intake.tare_position();
                
            }
            pros::delay(5);
        }

    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    clampPistons.set_value(false);

    static int redMin = 9;
    static int redMax = 18;
    static int blueMin = 190;
    static int blueMax = 250;
    while(true){
        //check colour by looking at preload ring
        if(optical.get_hue()>redMin&& optical.get_hue()<redMax){
            color = alliance::RED;
        }
        else if(optical.get_hue()>blueMin&& optical.get_hue()<blueMax){
            color = alliance::BLUE;
        }
        else{
            color = alliance::RED;
        }
        pros::delay(20);
    }
   
    
    
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    static int redMin = 9;
    static int redMax = 18;
    static int blueMin = 190;
    static int blueMax = 250;

    //check colour by looking at preload ring
    if(optical.get_hue()>redMin&& optical.get_hue()<redMax){
        color = alliance::RED;
    }
    else if(optical.get_hue()>blueMin&& optical.get_hue()<blueMax){
        color = alliance::BLUE;
    }
    else{
        color = alliance::RED;
    }
    pros::delay(20);
    
    rushBlue();
}


float computeLiftError(float desiredAngle, float settleError = 50){
    static float error = 0;
    float angle = (wsr.get_angle() >= 35000) ? wsr.get_angle() - 36000 : wsr.get_angle();
    error = desiredAngle - angle;
    if(fabs(error) <= settleError){
        error = 0;
    }
    return error;
}
//copied from lemlib
float driveCurve(float input, float deadband, float minOutput, float curve) {
    // return 0 if input is within deadzone
    if (fabs(input) <= deadband) return 0;
    // g is the output of g(x) as defined in the Desmos graph
    const float g = fabs(input) - deadband;
    // g127 is the output of g(127) as defined in the Desmos graph
    const float g127 = 127 - deadband;
    // i is the output of i(x) as defined in the Desmos graph
    const float i = pow(curve, g - 127) * g * lemlib::sgn(input);
    // i127 is the output of i(127) as defined in the Desmos graph
    const float i127 = pow(curve, g127 - 127) * g127;
    return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * lemlib::sgn(input);
}
void antiTipDrive(int leftIN, int rightIN, double changeRate, int maxChange, float deadband, float minOutput, float Rcurve, float Lcurve) {
    static int prevLeftPower = 0;
    static int prevRightPower = 0;
    static float prevRightIN = 0; 
    static bool pulseActive = false; 

    float leftC = driveCurve(leftIN, deadband, minOutput, Lcurve);
    float rightC = driveCurve(rightIN, deadband, minOutput, Rcurve);
    leftC = (rightC == 0) ? leftC : leftC * 0.60;

    float leftPower = (leftC + rightC) * 12000 / 127;
    float rightPower = (rightC - leftC) * 12000 / 127;

    // Detect quick joystick release
    float releaseSpeed = fabs(prevRightIN - rightIN); 
    bool isCentered = fabs(rightIN) <= deadband+10;//tune 10

    if (!isCentered) pulseActive = false; 

    if (isCentered && !pulseActive) { // tune 50
        // Apply reverse pulse proportional to previous power
        leftPower = -prevLeftPower * 0.3; // tune 30%
        rightPower = -prevRightPower * 0.3;
        pulseActive = true; 
        pros::delay(75); // tune 75
    }

    int leftChange = leftPower - prevLeftPower;
    int rightChange = rightPower - prevRightPower;

    if (abs(leftChange) >= maxChange) {
        leftPower = prevLeftPower + ((leftChange > 0) ? changeRate : -changeRate);
    }

    if (abs(rightChange) >= maxChange) {
        rightPower = prevRightPower + ((rightChange > 0) ? changeRate : -changeRate);
    }

    // Update motors
    left.move_voltage(leftPower);
    right.move_voltage(rightPower);

    // Save previous values
    prevLeftPower = leftPower;
    prevRightPower = rightPower;
    prevRightIN = rightIN;
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	// loop forever
	bool clamp = false;
	bool descoring = false;
    int i = 0;
	ws.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    wsr.reset();

    lemlib::PID liftpid(3, 0, 3);
    liftpid.reset();
    bool manuealLift = false;
    int liftLastUsedTime = 0;
    int startTime=0;
    static int maxTime = 300;

    while (true) {
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if(controller.get_digital(DIGITAL_R1)){
    	    //intake.move_voltage(12000);
            inState = intakeState::IN;
        } else if (controller.get_digital(DIGITAL_R2)){
            //intake.move_voltage(-12000);
            inState = intakeState::OUT;
        } else {
            inState = intakeState::STOP;
		}
        if(controller.get_digital(DIGITAL_DOWN)&&controller.get_digital(DIGITAL_B) &&!manuealLift){
            if(controller.get_digital_new_press(DIGITAL_DOWN)&&controller.get_digital_new_press(DIGITAL_B)){
                startTime = pros::millis();
            }
            if(pros::millis() - startTime >= maxTime){
                manuealLift = true;
                liftLastUsedTime = pros::millis();
                controller.rumble(".");
            }

        }

		if(controller.get_digital_new_press(DIGITAL_L1)){
			clamp = !clamp;
			clampPistons.set_value(clamp);
		}
		if(controller.get_digital_new_press(DIGITAL_Y)){
			descoring = !descoring;
			descore.set_value(descoring);
		}
        if(controller.get_digital_new_press(DIGITAL_UP)){
            if(color == alliance::RED){
                color = color = alliance::BLUE;
            }

            else if(color == alliance::BLUE){
                color = color = alliance::RED;
            }

        }

        if(controller.get_digital_new_press(DIGITAL_L2)){
            liftpid.reset();
            i++;
            if(i == 5){
                i = 2;
            } else if( i > 2 ){
                i = 0;
            }
        }

        if(controller.get_digital_new_press(DIGITAL_DOWN)){
            i = 3;
        }
        if(controller.get_digital_new_press(DIGITAL_RIGHT)){
            i = 4;
        }
        if(!manuealLift){
            switch(i){
                    case 0:
                    ws.move_voltage(-liftpid.update(computeLiftError(700, 10)));
                    break;
                    case 1:
                    ws.move_voltage(-liftpid.update(computeLiftError(2400)));
                    break;
                    case 2:
                    ws.move_voltage(-liftpid.update(computeLiftError(15500)));
                    break;
                    case 3:
                    ws.move_voltage(-liftpid.update(computeLiftError(21000)));
                    break;
                    case 4:
                    ws.move_voltage(-liftpid.update(computeLiftError(5000)));
                }
        }
        else{
            if(pros::millis()-liftLastUsedTime>=5000){
                manuealLift = false;
                i=0;
                controller.rumble("-");
            }
            if(controller.get_digital(DIGITAL_DOWN)){
                ws.move_voltage(-12000);
                liftLastUsedTime = pros::millis();
            }
            else if(controller.get_digital(DIGITAL_B)){
                ws.move_voltage(12000);
                liftLastUsedTime = pros::millis();
            }
            else{
                   ws.move_voltage(0);
            }

            

        }

        // move the robot
        antiTipDrive(rightX, leftY, 1000, 3000, 15, 20, 1.05, 1.09);
        // chassis.arcade(leftY, rightX);

        // delay to save resources
        pros::delay(25);
    }
}
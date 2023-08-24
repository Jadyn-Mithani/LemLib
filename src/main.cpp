#include "main.h"
#include "lemlib/api.hpp"
#include <iostream>


pros::Controller master(pros::E_CONTROLLER_MASTER);
//Utility Motors

pros::Motor catapult(19);
pros::Motor Intake (9);

// drivetrain motors
pros::Motor lf(-3); 
pros::Motor lt(14);
pros::Motor lb(-13);
pros::Motor rf(7); 
pros::Motor rt(-6);
pros::Motor rb(5);

//Sensors
pros::Rotation CataRot(20);
pros::IMU IMU1(1);

//Drivetrain motor groups
pros::MotorGroup leftmotors({lf, lt, lb});
pros::MotorGroup rightmotors({rf, rt, rb});

//Pneumatics
pros::ADIDigitalOut wings ('H');
pros::ADIDigitalOut  Intakepneu ('G', true);

 
lemlib::Drivetrain_t drivetrain {
    &leftmotors, // left drivetrain motors
    &rightmotors, // right drivetrain motors
    11, // track width
    3.25, // wheel diameter
    360 // wheel rpm
};
 
// left tracking wheel encoder
// inertial sensor
//pros::Imu inertial_sensor(2); // port 2
 
// odometry struct
lemlib::OdomSensors_t sensors {
    nullptr, // vertical tracking wheel 1
    nullptr, // vertical tracking wheel 2
    nullptr, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2
    &IMU1 // inertial sensor
};
 
// forward/backward PID
lemlib::ChassisController_t lateralController {
	8,
	30,
	1,
	100,
	3,
	500,
	200
};
 //kp 8 kd 80
// turning PID
lemlib::ChassisController_t angularController {
	11,
	109.75,
	1,
	100,
	3,
	500,
	4
};
 
 
// create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
bool firecata = false;
bool block = false;
bool CataSettled;
bool blockertoggle = false;
float pos;
float error;

void IntakeForward(){
    Intake.move(127);
}

void IntakeBackward(){
    Intake.move(-127);
}

void IntakeStop(){
    Intake.move(0);
}

lemlib::FAPID Catapid(0, 0, 13, 0, 30, "CataPID");
void CataFire(){
    while(true){
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && CataSettled == true){
        firecata = true;
        pros::delay(70);
        firecata = false;
        pros::delay(250);
        Catapid.reset();
        CataSettled = false;
    }
    pros::delay(10);
    }
}
bool bumpfire = false;
const float maintargetval = 94;
const float mainFFVval = 35;
// kp 8 kd 12 FFV 73
void CataPID() {
    Catapid.setExit(2, 0.5, 1000, 200, -1);
    float target = maintargetval;
    float FFV = mainFFVval;
    float finalVoltage;
    while (true) {

        if (block == true){
            target = 70;
        } 
        else if (blockertoggle == true){
            target = 0;
            if (pos < 3){
            FFV = 0;
            } else {
            FFV = mainFFVval;
        }
        } else {
            target = maintargetval;
            FFV = mainFFVval;
        }

        pos = (CataRot.get_angle() / 100.0); 

        if (pos > 350) {pos = 0;}

        float output = FFV + Catapid.update(target, pos);
        error = target - pos;

        pros::lcd::print(0, "CataPos: %f", (pos));
        pros::lcd::print(1, "Error: %f", (error));
        pros::lcd::print(2, "Power: %f", finalVoltage);
        pros::lcd::print(3, "CataTemp: %f", catapult.get_temperature());

        if (block){
            master.print(1, 1, "CMode: %s", "IntakeHold");
        } else if (blockertoggle) {
            master.print(1, 1, "CMode: %s", "Blocker");
        } else {
            master.print(1, 1, "CMode %s", "Normal");
        }

        if (CataSettled == true){
            pros::lcd::print(4, "Settled: %s", "true");
        } else {
             pros::lcd::print(4, "Settled: %s", "false");
        }

        if (firecata == true || bumpfire == true){
            finalVoltage = 127;
        }
        else if (int(fabs(error)) && CataSettled == false){
            finalVoltage = output;
        }
        else {
            finalVoltage = 0;
        }

        if (Catapid.settled() == true){
            CataSettled = true;
        }

        if (finalVoltage < 0) {
            finalVoltage = 0;
        }

        catapult.move(finalVoltage);
        
        master.clear();
        pros::delay(10);
    }

}

void screen() {
    // loop forever
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        // pros::lcd::print(5, "x: %f", pose.x); // print the x position
        // pros::lcd::print(6, "y: %f", pose.y); // print the y position
        // pros::lcd::print(7, "heading: %f", pose.theta); // print the heading

        pros::delay(10);
    }
}

// void DetectFunction() {
//     while (true){

//         if(pointIndex == ){

//         }



//     }
// }

void initialize() {
    catapult.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	chassis.calibrate();
	chassis.setPose(-12, -60, 270);
    pros::Task screenTask(screen);
    pros::lcd::initialize();
    // calibrate sensors
    pros::Task CataControl(CataPID);
    pros::Task CataFireControl(CataFire);
    pros::Task DetectFunction(DetectFunction);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

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
    chassis.follow("bruh.txt", 100, 10)
    IntakeIN();
    chassis.moveTo(0, -60, 10000);

	//chassis.moveTo(0, -25, 10000);
    // chassis.turnTo(100, 0, 10000);
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

    //Set Motor Brake Modes
    leftmotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);
    rightmotors.set_brake_modes(pros::E_MOTOR_BRAKE_BRAKE);

    //Toggle Variables
    bool toggleBrake = false; // Initialize the toggle variable to false
    bool toggleIntake = false; // Initialize the toggle variable to false
    bool intakepos = false; // False is in, true is out
    bool togglewings = true;
    bool override = false;
    //Expo Drive Curve tuning
    double t = 10.0; // Linear Stick
    double tt = 15.0; // Angular Stick
    //Saturation Tuning
    double SatValue = 0.1;
    //Deadzone tuning
    const int deadzone = 3;
    //Intake Control
    float speed = 127;

    while (true) {

    //Driver Control 
    int power = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    //Apply Deadzone
    if (abs(power) <= deadzone) {
      power = 0;
    }
    if (abs(turn) <= deadzone) {
      turn = 0;
    }

    double CurvedPower = std::exp(((std::abs(power)-127)*t)/1270) * power;
    double CurvedTurn = std::exp(((std::abs(turn)-127)*tt)/1270) * turn;

    double RPower = CurvedPower * (1 - (std::abs(turn) / 127.0) * SatValue);

    float left = (RPower + CurvedTurn);
    float right = (RPower - CurvedTurn);

    leftmotors.move(left);
    rightmotors.move(right);

    if (abs(power) + abs(turn) == 0) {
        leftmotors.brake();
        rightmotors.brake();
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1) && fabs(error) < 10 && int(catapult.get_actual_velocity()) == 0 ) {
        override = true;
    }  

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){
        toggleIntake = !toggleIntake;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
        block = !block;
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)){
        toggleIntake = false;
    }

    if (block == true){
        speed = 60;
    }else{
        speed = 127;
    }

    if(override == true || (error < 0 || (CataSettled == true && (fabs(error)) < 5 ))){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        Intake.move(-127);
    }
    else if (toggleIntake == true){
        Intake.move(speed);
    }
    else {
        if(toggleIntake == true){
            Intake.move(-127);
        }
        else{
            Intake.move(0);
        }
    }
    }
    else {
        Intake.move(0);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        togglewings = !togglewings;
    }

    if (togglewings == true){
        wings.set_value(false);
    } else {
        wings.set_value(true);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
        intakepos = !intakepos;
    }

    if (intakepos == true){
        Intakepneu.set_value(true);
    } else {
        Intakepneu.set_value(false);
    }

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
        blockertoggle = !blockertoggle;
    }
    
    // if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
    //     bumpfire = !bumpfire;
    // }  


pros::delay(10);

}
}
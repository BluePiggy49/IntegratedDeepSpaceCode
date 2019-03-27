#include "Robot.h"
#include <iostream>
#include "ctre/Phoenix.h"
#include "frc/WPILib.h"
#include <frc/Joystick.h>
#include <twist.h>
#include <PID.h>
#include <AHRS.h>
#include <iostream>
#include <socket.h>

#include <constants.h>
#include <drivebase.h> 
#include <intake_wheels.h>
#include <intake_clamp.h>
#include <intake_pivot.h>
#include <elevator.h>
#include <climber.h>
#include <diagnostic.h>

using namespace frc;
//using namespace std::chrono;

Joystick *joy0;
Joystick *joy1;

TalonSRX *talon_left_enc;
TalonSRX *talon_left_noenc;
TalonSRX *talon_right_enc; 
TalonSRX *talon_right_noenc;
TalonSRX *climber_talon_wheel;
TalonSRX *climber_talon_arm;
TalonSRX *claw_pivot_talon_enc;
TalonSRX *talon_wheels;
Servo *lock_servo;

DigitalInput * mag_switch_climber;
DigitalInput * mag_switch_claw;
DigitalInput *mag_switch_elevator;
TalonSRX *talon_clamp;
TalonSRX *talon_elevator_enc;

Drivebase *drivebase;
Intake_wheels *intake_wheels;
Climber *climber;
Intake_clamp *intake_clamp;
Intake_pivot *intake_pivot;
PID *pid;
Elevator *elevator;
Solenoid *light;
AHRS *navx;
Socket *client;
Twist *twist;

int mode = 1;
bool pig = 0;
bool pigpig = 0;
bool dog = 0;
bool dogdog = 0;
int loop = 0;

void Robot::RobotInit() {
	   joy0 = new Joystick(0);
	   joy1 = new Joystick(1);

	   talon_left_noenc = new TalonSRX(1);
	   talon_left_enc = new TalonSRX(2);
	   talon_right_noenc = new TalonSRX(3);
	   talon_right_enc = new TalonSRX(4);
	   claw_pivot_talon_enc = new TalonSRX(9);
	   climber_talon_arm = new TalonSRX(6);
	   climber_talon_wheel = new TalonSRX(7);
	   talon_elevator_enc = new TalonSRX(8);
	   talon_wheels = new TalonSRX(5);
	talon_clamp = new TalonSRX(10);

	talon_left_noenc->Set(ControlMode::Follower, 2);
	talon_right_noenc->Set(ControlMode::Follower, 4);
	   
	lock_servo = new Servo(1);

	mag_switch_climber = new DigitalInput(2);
	mag_switch_elevator = new DigitalInput(4);
	mag_switch_claw = new DigitalInput (1);

	   
	   
	elevator = new Elevator(talon_elevator_enc, mag_switch_elevator, joy1);
	pid = new PID (talon_right_enc, talon_left_enc, claw_pivot_talon_enc, talon_elevator_enc);
	intake_pivot = new Intake_pivot(claw_pivot_talon_enc, joy1, mag_switch_claw);
	intake_clamp = new Intake_clamp(joy1, talon_clamp);
	drivebase = new Drivebase (joy0, talon_left_enc, talon_left_noenc, talon_right_enc, talon_right_noenc);
	climber = new Climber(joy0, mag_switch_climber, climber_talon_arm,climber_talon_wheel, lock_servo);
	intake_wheels= new Intake_wheels(joy1, talon_wheels);

	CameraServer::GetInstance()->StartAutomaticCapture();

	climber_talon_arm->ConfigPeakOutputForward(.75, 10);
	climber_talon_arm->ConfigPeakOutputReverse(-.75, 10);
	climber_talon_wheel->ConfigPeakOutputForward(1, 10);
	climber_talon_wheel->ConfigPeakOutputReverse(-1, 10);
	talon_elevator_enc->ConfigPeakOutputForward(.25, 10);
	talon_elevator_enc->ConfigPeakOutputReverse(-.75, 10);
	talon_wheels->ConfigPeakOutputForward(1, 10);
	talon_wheels->ConfigPeakOutputReverse(-1, 10);
	talon_clamp->ConfigPeakOutputForward(1, 10);
	talon_clamp->ConfigPeakOutputReverse(-1, 10);

	light = new Solenoid(1);

	navx = new AHRS(SPI::Port::kMXP);

	pid->PID_claw_elevator();
	pid->PID_drivebase30();

	client = new Socket();
	twist = new Twist(talon_left_enc, talon_right_enc, pid, navx, client, joy0);
	std::cout<<"Full Teleop v2"<<std::endl;


}

void Robot::RobotPeriodic() {}

void Robot::AutonomousInit() {
	talon_left_enc->ConfigPeakOutputForward(1, 10);
	talon_left_enc->ConfigPeakOutputReverse(-1, 10);
	talon_right_enc->ConfigPeakOutputForward(1, 10);
	talon_right_enc->ConfigPeakOutputReverse(-1, 10);
	claw_pivot_talon_enc->SetSelectedSensorPosition(0,0,10);
	talon_elevator_enc->SetSelectedSensorPosition(0,0,10);
}



void Robot::AutonomousPeriodic() {
	light->Set(1);
	// climber->run_climber(.75);  
	//intake_wheels->update();
	// intake_clamp->update();
	claw_pivot_talon_enc->Set(ControlMode::PercentOutput, joy1->GetRawAxis(5));
	//intake_pivot->run_intake_pivot(0, -175, -650, -1800, -1000);
	elevator->run_elevator(-9500,-22750, -27500 );
	drivebase->update();
}

void Robot::TeleopInit() {
	talon_left_enc->ConfigPeakOutputForward(.75, 10);
	talon_left_enc->ConfigPeakOutputReverse(-.75, 10);
	talon_right_enc->ConfigPeakOutputForward(.75, 10);
	talon_right_enc->ConfigPeakOutputReverse(-.75, 10);
}

void Robot::TeleopPeriodic() {
	light->Set(1);
	//climber->run_climber(.75);   
	//intake_wheels->update();
	// intake_clamp->update();
	claw_pivot_talon_enc->Set(ControlMode::PercentOutput, joy1->GetRawAxis(5));
	//intake_pivot->run_intake_pivot(0, -175, -650, -1350, -1000);
	elevator->run_elevator(-9500,-22750, -27500 );

	bool button = joy0->GetRawButton(1);
	if (button == 1 && pigpig == 0 && pig == 0)
	{
		pigpig = 1;
		std::cout<<"I'm Here in Teleop"<<std::endl;
	}
	if (pigpig == 1 && joy0->GetRawButton(1) == 0)
	{
		pig = 1;
		pigpig = 0;
		twist->mode = 1;
		std::cout<<"Now I'm here in teleop"<<std::endl;
	}
	if (pig == 1 && pigpig == 0)
	{
		//twist->auto_align();
		std::cout<<client->update()<<std::endl;
		//twist->mode++;
		std::cout<<"I'm aligning now"<<std::endl;
		bool button_2 = joy0->GetRawButton(2);
		if (/*twist->mode == 7 || */button_2 == 1)
		{
			pig = 0;
			pigpig = 0;
			std::cout<<"Reset"<<std::endl;
			twist->mode = 0;
		}
	}
	if (button == 0 && pigpig == 0 && pig == 0)
	{
		drivebase->update();
	}
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <cmath>
#include <map>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Lib830.h"

using namespace Lib830;

std::map<int, std::string> sounds {
//	{1, "engine"},
	{2, "roar"},
	{3, "growl"},
	{4, "sheep"},
	{5, "fart"},
//	{6, "laser"},
//	{7, "elevator"},
//	{8, "cat"},
};

enum GearState {low = false, high = true};

class PIDSpeedControllerGroup : public SpeedControllerGroup {
private:
	PIDController pid;
	Encoder &encoder;
public:
	template<typename... Args>
    PIDSpeedControllerGroup(Encoder &encoder, Args&&... args) 
       :SpeedControllerGroup(std::forward<Args>(args)...),
	    pid(0.05, 0, 0, encoder, *this),
		encoder(encoder)
	{
		encoder.SetPIDSourceType(PIDSourceType::kRate);
		pid.SetPIDSourceType(PIDSourceType::kRate);
		pid.SetInputRange(-100, 100);
		pid.SetOutputRange(-0.5,0.5);
		pid.Enable();
	}

	void Set(double value) override {
		pid.SetSetpoint(value);
	}
	void PIDWrite(double value) override {
		SpeedControllerGroup::Set(value);
	}
};

class Robot : public frc::IterativeRobot {
public:
	static const int PWM_R1 = 6, PWM_R2 = 7, PWM_R3 = 8,
					 PWM_L1 = 3, PWM_L2 = 4, PWM_L3 = 5,
					 PWM_WING_FLAP = 9;

	static const int ENCODER_RIGHT_1 = 0;
	static const int ENCODER_RIGHT_2 = 1;
	static const int ENCODER_LEFT_1 = 2;
	static const int ENCODER_LEFT_2 = 3;

	static const int PCM_GEAR_SHIFT = 1, PCM_WING_OPEN = 0;

	Talon L1{PWM_L1};
	Talon L2{PWM_L2};
	Talon L3{PWM_L3};
	Talon R1{PWM_R1};
	Talon R2{PWM_R2};
	Talon R3{PWM_R3};

	Victor wingFlap{PWM_WING_FLAP};

	Solenoid wingOpen{PCM_WING_OPEN};
	Solenoid gearShift{PCM_GEAR_SHIFT};

	Toggle wingState{false};

	const int BUBBLE_MACHINE_RELAY = 0; //placeholder value

	Relay bubbleBoi {BUBBLE_MACHINE_RELAY, Relay::kForwardOnly};

	Encoder left_encoder{ENCODER_LEFT_1, ENCODER_LEFT_2};
	Encoder right_encoder{ENCODER_RIGHT_1, ENCODER_RIGHT_2};
	// PIDSpeedControllerGroup Left{left_encoder, L1, L2, L3};
	// PIDSpeedControllerGroup Right{right_encoder, R1, R2, R3};
	SpeedControllerGroup Left{L1, L2, L3};
	SpeedControllerGroup Right{R1, R2, R3};

	DifferentialDrive drive {Left, Right};

	XboxController pilot {0};


	std::map<std::string, DigitalOutput*> sound_outputs;
	std::map<int /*button ID*/, SendableChooser<DigitalOutput*>> sound_choosers;


	static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
	static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

	void RobotInit() override {

		for (auto kv : sounds) {
			sound_outputs[kv.second] = new DigitalOutput(kv.first);

			for (int button : {GamepadF310::BUTTON_X, GamepadF310::BUTTON_Y}) {
				sound_choosers[button].AddObject(kv.second, sound_outputs[kv.second]);
			}
		}



		SmartDashboard::PutData("Sound X", &sound_choosers[GamepadF310::BUTTON_X]);
		SmartDashboard::PutData("Sound Y", &sound_choosers[GamepadF310::BUTTON_Y]);

		setSound(0);
		gearShift.Set(false);
		wingOpen.Set(false);
	}

	void setSound(DigitalOutput *out) {
		for (auto kv : sound_outputs) {
			// 1 = off
			kv.second -> Set(kv.second != out);
		}
	}

	void AutonomousInit() override {
		
	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		
	}

	double prev_speed = 0;
	GearState gear = low; 
	void TeleopPeriodic() {

		if (pilot.GetBButton()){
			bubbleBoi.Set(Relay::kForward);
		}
		else{
			bubbleBoi.Set(Relay::kOff);
		}

		double speed = accel(prev_speed, -pilot.GetY(LEFT) * 0.5, 50);
		prev_speed = speed;

		drive.CurvatureDrive(-speed,pilot.GetX(RIGHT) * -0.5, std::abs(speed) < 0.05);

		setSound(0);

		for (auto &kv : sound_choosers) {
			if (pilot.GetRawButton(kv.first)) {
				setSound(kv.second.GetSelected());
			}
		}

		int POV = pilot.GetPOV();
		if ((POV <= 45 && POV >= 0) || POV >= 315){
			gear = high;
		}
		if (POV <= 225 && POV >= 135){
			gear = low;			
		}
		gearShift.Set(gear);

		wingOpen.Set(wingState.toggle(pilot.GetAButton()));
		wingFlap.Set(deadzone(pilot.GetTriggerAxis(RIGHT)) * 0.2);

		SmartDashboard::PutBoolean("Wings Extended: ", wingState);
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	float deadzone(float a) {
		if (fabs(a) < 0.05){
			return 0;
		}
		return a;
	}
};

START_ROBOT_CLASS(Robot)

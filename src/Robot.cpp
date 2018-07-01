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

class Robot : public frc::IterativeRobot {
public:
	static const int PWM_R1 = 0, PWM_R2 = 1, PWM_R3 = 2,
					 PWM_L1 = 3, PWM_L2 = 4, PWM_L3 = 5;
	VictorSP L1{PWM_L1};
	VictorSP L2{PWM_L2};
	VictorSP L3{PWM_L3};
	VictorSP R1{PWM_R1};
	VictorSP R2{PWM_R2};
	VictorSP R3{PWM_R3};

	const int Bubble_Machine_Relay = 0; //placeholder value

	Relay bubbleBoi {Bubble_Machine_Relay, Relay::kForwardOnly};

	SpeedControllerGroup Left{L1,L2,L3};
	SpeedControllerGroup Right{R1,R2,R3};

	DifferentialDrive drive {Left, Right};

	XboxController pilot {0};


	std::map<std::string, DigitalOutput*> sound_outputs;
	std::map<int /*button ID*/, SendableChooser<DigitalOutput*>> sound_choosers;


	static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
	static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

	void RobotInit() {

		for (auto kv : sounds) {
			sound_outputs[kv.second] = new DigitalOutput(kv.first);

			for (int button : {GamepadF310::BUTTON_A, GamepadF310::BUTTON_B}) {
				sound_choosers[button].AddObject(kv.second, sound_outputs[kv.second]);
			}
		}



		SmartDashboard::PutData("sound A", &sound_choosers[GamepadF310::BUTTON_A]);
		SmartDashboard::PutData("sound B", &sound_choosers[GamepadF310::BUTTON_B]);
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

	void TeleopPeriodic() {

		if (pilot.GetBButton()){
			bubbleBoi.Set(Relay::kForward);
		}
		else{
			bubbleBoi.Set(Relay::kOff);
		}

		double speed = pilot.GetY(LEFT);
		drive.CurvatureDrive(speed,pilot.GetX(RIGHT),fabs(speed) < 0.05);

		setSound(0);

		for (auto &kv : sound_choosers) {
			if (pilot.GetRawButton(kv.first)) {
				setSound(kv.second.GetSelected());
			}
		}
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
};

START_ROBOT_CLASS(Robot)

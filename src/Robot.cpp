/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* comment your name if you can see this -peter, Adithya cool */
#include <iostream>
#include <string>
#include <cmath>
#include <map>

#include <frc/drive/DifferentialDrive.h>
#include <frc/TimedRobot.h>
#include <frc/LiveWindow/LiveWindow.h>
#include <frc/SmartDashboard/SendableChooser.h>
#include <frc/SmartDashboard/SmartDashboard.h>
#include <frc/SpeedControllerGroup.h>
#include <frc/Victor.h>
#include <frc/Talon.h>
#include <frc/Solenoid.h>
#include <frc/Relay.h>
#include <frc/XBoxController.h>
#include "Lib830.h"

using namespace frc;
using namespace Lib830;

std::map<int, std::string> sounds {
//	{1, "engine"},
	{4, "roar"},
	{5, "growl"},
	{6, "sheep"},
	{7, "fart"},
//	{6, "laser"},
//	{7, "elevator"},
//	{8, "cat"},
};

enum GearState {low = false, high = true};

/*class PIDSpeedControllerGroup : public SpeedControllerGroup {
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
};*/

class Robot : public frc::TimedRobot {
public:
	// Motor PWM Pins
	static const int PWM_R1 = 6, PWM_R2 = 7, PWM_R3 = 8,
	PWM_L1 = 3, PWM_L2 = 4, PWM_L3 = 5,
	PWM_WING_FLAP = 9;

	// Motor Encoder Pins
	static const int ENCODER_RIGHT_1 = 0, ENCODER_RIGHT_2 = 1,
	ENCODER_LEFT_1 = 2, ENCODER_LEFT_2 = 3;

	// soloniod pins
	static const int PCM_GEAR_SHIFT = 1, PCM_WING_OPEN = 0;

	// dead zones				
	static constexpr double CONTROLLER_DEADZONE = 0.1;
	static constexpr double WING_DEADZONE = 0.05;		

	// Drivetrain
	Talon L1{PWM_L1};
	Talon L2{PWM_L2};
	Talon L3{PWM_L3};
	Talon R1{PWM_R1};
	Talon R2{PWM_R2};
	Talon R3{PWM_R3};
	SpeedControllerGroup Left{L1, L2, L3};
	SpeedControllerGroup Right{R1, R2, R3};
	DifferentialDrive drive {Left, Right};
	Solenoid gearShift{PCM_GEAR_SHIFT};
	double prev_speed = 0;
	GearState gear = low;
        
		// Wings
	Victor wingFlap{PWM_WING_FLAP};
	Toggle wingState{false};
	Solenoid wingOpen{PCM_WING_OPEN};
	
	// Bubbble Machine
	const int BUBBLE_MACHINE_RELAY = 0; //placeholder value
	Relay bubbleBoi {BUBBLE_MACHINE_RELAY, Relay::kForwardOnly};

	// Encoder left_encoder{ENCODER_LEFT_1, ENCODER_LEFT_2};
	// Encoder right_encoder{ENCODER_RIGHT_1, ENCODER_RIGHT_2};
	// PIDSpeedControllerGroup Left{left_encoder, L1, L2, L3};
	// PIDSpeedControllerGroup Right{right_encoder, R1, R2, R3};


	XboxController pilot {0};


	std::map<std::string, DigitalOutput*> sound_outputs;
	std::map<int /*button ID*/, SendableChooser<DigitalOutput*>> sound_choosers;

	static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
	static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

	void RobotInit() override {
		for (auto kv : sounds) {
			sound_outputs[kv.second] = new DigitalOutput(kv.first);

			for (int button : {GamepadF310::BUTTON_X, GamepadF310::BUTTON_Y}) {
				sound_choosers[button].AddOption(kv.second, sound_outputs[kv.second]);
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

	void AutonomousInit() override {}

	void AutonomousPeriodic() {}

	void TeleopInit() {}



	void TeleopPeriodic() {
		// Bubble Machine
		if (pilot.GetBButton()){
			bubbleBoi.Set(Relay::kForward);
		}
		else{
			bubbleBoi.Set(Relay::kOff);
		}
		// Drivetrain
		double speed = accel(prev_speed, -pilot.GetY(LEFT), 75);
		prev_speed = speed;

        drive.CurvatureDrive(-speed, pilot.GetX(RIGHT) * -0.5, std::abs(speed) < CONTROLLER_DEADZONE);

		// Sounds
        setSound(0);

		for (auto &kv : sound_choosers) {
			if (pilot.GetRawButton(kv.first)) {
				setSound(kv.second.GetSelected());
			}
		}
		// Gearshifter
		int POV = pilot.GetPOV();
		if ((POV <= 45 && POV >= 0) || POV >= 315){
			gear = high;
		}
		if (POV <= 225 && POV >= 135){
			gear = low;			
		}
		gearShift.Set(gear);

		// Wings
		wingOpen.Set(wingState.toggle(pilot.GetAButton()));
		wingFlap.Set(deadzone(pilot.GetTriggerAxis(RIGHT), WING_DEADZONE) * 0.2);

		SmartDashboard::PutBoolean("Wings Extended: ", wingState);
	}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	float deadzone(float a, float deadzoneValue) {
		if (fabs(a) < deadzoneValue){
			return 0;
		}
		return a;
	}
};

int main() { 
	return frc::StartRobot<Robot>(); 
}

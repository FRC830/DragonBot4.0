/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <string>
#include <cmath>

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Lib830.h"

class Robot : public frc::IterativeRobot {
public:
	static const int PWM_R1 = 0, PWM_R2 = 1,
					 PWM_L1 = 2, PWM_L2 = 3;
	VictorSP L1{PWM_L1};
	VictorSP L2{PWM_L2};
	VictorSP R1{PWM_R1};
	VictorSP R2{PWM_R2};

	SpeedControllerGroup Left{L1,L2};
	SpeedControllerGroup Right{R1, R2};

	DifferentialDrive drive {Left, Right};

	XboxController pilot {0};

	static const GenericHID::JoystickHand LEFT = GenericHID::kLeftHand;
	static const GenericHID::JoystickHand RIGHT = GenericHID::kRightHand;

	void RobotInit() {

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		double speed = pilot.GetY(LEFT);
		drive.CurvatureDrive(speed,pilot.GetX(RIGHT),fabs(speed) < 0.05);
	}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
};

START_ROBOT_CLASS(Robot)

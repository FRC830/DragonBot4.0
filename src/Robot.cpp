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
	DifferentialDrive drive {
		SpeedControllerGroup {VictorSP {PWM_L1}, VictorSP {PWM_L2}},
		SpeedControllerGroup {VictorSP {PWM_R1}, VictorSP {PWM_R2}}
	};

	XboxController pilot {0};
	void RobotInit() {

	}

	void AutonomousInit() override {

	}

	void AutonomousPeriodic() {

	}

	void TeleopInit() {
		if (Numeric::abs(pilot.GetY(GenericHID::kLeftHand)) < )
		drive.CurvatureDrive(pilot.GetY(GenericHID::kLeftHand),
				pilot.GetX(GenericHID::kRightHand),
				false);
	}

	void TeleopPeriodic() {}

	void TestPeriodic() {}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
};

START_ROBOT_CLASS(Robot)

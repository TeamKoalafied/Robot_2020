//==============================================================================
// Intake.cpp
//==============================================================================

#include "Intake.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Intake::Intake()  {
}

Intake::~Intake() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Intake::Setup() {
    std::cout << "Intake::Setup()\n";
    intake_master_speed_controller = new TalonSRX(RobotConfiguration::kIntakeTalonId);

    TalonSRXConfiguration intake_configuration;

    intake_configuration.nominalOutputReverse = -0.0f;
    intake_configuration.nominalOutputForward = 0.0f;

    intake_configuration.peakOutputReverse = -1.0f;
    intake_configuration.peakOutputForward = +1.0f;
    
    intake_configuration.continuousCurrentLimit = RobotConfiguration::kShooterMotorContinuousCurrentLimit;
    intake_configuration.peakCurrentLimit = RobotConfiguration::kShooterMotorPeakCurrentLimit;
    intake_configuration.peakCurrentDuration = RobotConfiguration::kShooterMotorPeakCurrentDurationMs;

    intake_configuration.forwardSoftLimitEnable = false;
    intake_configuration.reverseSoftLimitEnable = false;

    int error = intake_master_speed_controller->ConfigAllSettings(intake_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }
    
    intake_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    intake_master_speed_controller->SetSensorPhase(true); // Not reversed
    intake_master_speed_controller->EnableCurrentLimit(true);
	intake_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic()
{      
    frc::SmartDashboard::PutNumber("Intake Current", intake_master_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Intake Output", intake_master_speed_controller->GetMotorOutputPercent());         
}

//==============================================================================
// Operations

void Intake::ManualDriveIntake() {

}

void Intake::AutoDriveDashboard() {

}

void Intake::TestDriveIntake(frc::Joystick* joystick) {
     double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);

    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;

    intake_master_speed_controller->Set(ControlMode::PercentOutput, joystick_value);
}

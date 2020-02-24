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
#include <frc/Solenoid.h>


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
    m_intake_speed_controller = new TalonSRX(RobotConfiguration::kIntakeTalonId);

    m_intake_solenoid = new frc::Solenoid(2);

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

    int error = m_intake_speed_controller->ConfigAllSettings(intake_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the intake Talon failed with code:  " << error << "\n";
    }
    
    m_intake_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_intake_speed_controller->SetSensorPhase(true); // Not reversed
    m_intake_speed_controller->EnableCurrentLimit(true);
	m_intake_speed_controller->SetNeutralMode(NeutralMode::Coast);
}

void Intake::Shutdown() {
    std::cout << "Intake::Shutdown()\n";
}

void Intake::Periodic() {      
    frc::SmartDashboard::PutNumber("Intake Current", m_intake_speed_controller->GetOutputCurrent());        
    frc::SmartDashboard::PutNumber("Intake Output", m_intake_speed_controller->GetMotorOutputPercent());         
}

//==============================================================================
// Operations


void Intake::Extend() {
    m_intake_solenoid->Set(true);
}

void Intake::Retract() {
    m_intake_solenoid->Set(false);
}

void Intake::Run() {
    ManualDriveIntake(1.0);
}

void Intake::RunReverse() {
    ManualDriveIntake(-1.0);
}

void Intake::Stop() {
    ManualDriveIntake(0.0);
}

void Intake::ManualDriveIntake(double percentage_output) {
    m_intake_speed_controller->Set(ControlMode::PercentOutput, percentage_output);
}

void Intake::TestDriveIntake(frc::Joystick* joystick) {
    // Do tune driving of the intake roller. Using the right Y for the drive and trigger
    // close loop with the left trigger button.
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);

    const double MAX_RPM = 200.0; // TODO need to work this out properly
    KoalafiedUtilities::TuneDriveTalonSRX(m_intake_speed_controller, "Intake", joystick_value, MAX_RPM, close_loop);
}

//==============================================================================
// Indexer.cpp
//==============================================================================

#include "Indexer.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Indexer::Indexer()  {
}

Indexer::~Indexer() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Indexer::Setup() {
    std::cout << "Indexer::Setup()\n";
    
    // Create and configure the indexer Talon
    m_indexer_master_speed_controller = new TalonSRX(RobotConfiguration::kIndexerTalonId);

    TalonSRXConfiguration indexer_configuration;
    indexer_configuration.continuousCurrentLimit = RobotConfiguration::kShooterMotorContinuousCurrentLimit;
    indexer_configuration.peakCurrentLimit = RobotConfiguration::kShooterMotorPeakCurrentLimit;
    indexer_configuration.peakCurrentDuration = RobotConfiguration::kShooterMotorPeakCurrentDurationMs;

    int error = m_indexer_master_speed_controller->ConfigAllSettings(indexer_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the indexer Talon failed with code:  " << error << "\n";
    }
    
    m_indexer_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_indexer_master_speed_controller->SetSensorPhase(true); // Not reversed
    m_indexer_master_speed_controller->EnableCurrentLimit(true);
	m_indexer_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
}

void Indexer::Shutdown() {
    std::cout << "Indexer::Shutdown()\n";
}

void Indexer::Periodic(bool show_dashboard)
{
    if (show_dashboard) {
        frc::SmartDashboard::PutNumber("Indexer Current", m_indexer_master_speed_controller->GetOutputCurrent());        
        frc::SmartDashboard::PutNumber("Indexer Output", m_indexer_master_speed_controller->GetMotorOutputPercent());         
    }
}

//==============================================================================
// Operations

void Indexer::ManualDriveIndexer(double percentage_speed) {
    m_indexer_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

void Indexer::AutoDriveDashboard(bool feed_desire) {
    if (feed_desire){
        m_indexer_master_speed_controller->Set(ControlMode::PercentOutput, -1);
    } else {
        m_indexer_master_speed_controller->Set(ControlMode::PercentOutput, 0);
    }
}

void Indexer::TestDriveIndexer(frc::Joystick* joystick) {
    // Do tune driving of the indexer. Using the right Y for the drive and trigger
    // close loop with the left trigger button.
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;
    bool close_loop = joystick->GetRawButton(RobotConfiguration::kJoystickLTrigButton);

    const double MAX_RPM = 200.0; // TODO need to work this out properly
    KoalafiedUtilities::TuneDriveTalonSRX(m_indexer_master_speed_controller, "Indexer", joystick_value, MAX_RPM, close_loop);
}

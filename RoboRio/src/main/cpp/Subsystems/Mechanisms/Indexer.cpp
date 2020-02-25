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
    
    // Create controllers for each of the 4 drive talons
    indexer_master_speed_controller = new TalonSRX(RobotConfiguration::kIndexerTalonId);

    TalonSRXConfiguration indexer_configuration;

    indexer_configuration.nominalOutputReverse = -0.0f;
    indexer_configuration.nominalOutputForward = 0.0f;

    indexer_configuration.peakOutputReverse = -1.0f;
    indexer_configuration.peakOutputForward = +1.0f;
    
    indexer_configuration.continuousCurrentLimit = RobotConfiguration::kShooterMotorContinuousCurrentLimit;
    indexer_configuration.peakCurrentLimit = RobotConfiguration::kShooterMotorPeakCurrentLimit;
    indexer_configuration.peakCurrentDuration = RobotConfiguration::kShooterMotorPeakCurrentDurationMs;

    indexer_configuration.forwardSoftLimitEnable = false;
    indexer_configuration.reverseSoftLimitEnable = false;

    int error = indexer_master_speed_controller->ConfigAllSettings(indexer_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the indexer Talon failed with code:  " << error << "\n";
    }
    
    indexer_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    indexer_master_speed_controller->SetSensorPhase(true); // Not reversed
    indexer_master_speed_controller->EnableCurrentLimit(true);
	indexer_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
    
    // Log whether the encoders are connected
    //    printf("Left magnetic encode present: %s ",
    //           (m_left_master_speed_controller->IsSensorPresent(TalonSRX::CtreMagEncoder_Absolute) ? "true" : "false"));
    //    printf("Right magnetic encode present: %s ",
    //           (m_right_master_speed_controller->IsSensorPresent(TalonSRX::CtreMagEncoder_Absolute) ? "true" : "false"));

    // Set the encoders to be the feedback devices for closed loop control on the master motors
    // TSSRM Section 7 (page 43)

    

    
    // Set the peak and nominal voltage outputs for the master motors. This is for closed loop only.
    // The peak outputs are the maximum, but the nominal (same in both directions) is tuned to be
    // about the minimum value that will overcome the drive train friction.
    // TSSRM Section 10.5 (page 66)

    // Set the ramp rate for open and close loop modes. TODO Is this necessary for slaves?
    // TSSRM Section 6 (page 41)
    // indexer_master_speed_controller->ConfigOpenloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // indexer_master_speed_controller->ConfigClosedloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // m_shooter_slave_speed_controller->ConfigOpenloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // m_shooter_slave_speed_controller->ConfigClosedloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);

    // Voltage compensation TODO Is this required?
    // TSSRM Section 9.2 (page 60)
    //  virtual ctre::phoenix::ErrorCode ConfigVoltageCompSaturation(double voltage, int timeoutMs);
    //	virtual ctre::phoenix::ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);
    //	virtual void EnableVoltageCompensation(bool enable);

    // Set the continuous and peak current limits, for all motors. The is for all control modes (i.e. open/closed loop)
    // TSSRM Section 9.3 (page 62)
    
}

void Indexer::Shutdown() {
    std::cout << "Indexer::Shutdown()\n";
}

void Indexer::Periodic(bool show_dashboard)
{
    if (show_dashboard) {
        frc::SmartDashboard::PutNumber("Indexer Current", indexer_master_speed_controller->GetOutputCurrent());        
        frc::SmartDashboard::PutNumber("Indexer Output", indexer_master_speed_controller->GetMotorOutputPercent());         
    }
}

//==============================================================================
// Operations

void Indexer::ManualDriveIndexer(double percentage_speed) {
    indexer_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

void Indexer::AutoDriveDashboard(bool feed_desire) {
    if (feed_desire){
        indexer_master_speed_controller->Set(ControlMode::PercentOutput, -1);
    } else {
        indexer_master_speed_controller->Set(ControlMode::PercentOutput, 0);
    }
}

void Indexer::TestDriveIndexer(frc::Joystick* joystick) {
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);

    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;

    indexer_master_speed_controller->Set(ControlMode::PercentOutput, joystick_value);
}

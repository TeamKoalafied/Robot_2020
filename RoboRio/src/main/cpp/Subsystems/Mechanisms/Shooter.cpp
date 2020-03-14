//==============================================================================
// Shooter.cpp
//==============================================================================

#include "Shooter.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;

//const double Shooter::kShooterDegreesPerEncoder = 360.0 / (4096.0 * RC::kShooterRetractRatio);


//==============================================================================
// Construction

Shooter::Shooter()  {
}

Shooter::~Shooter() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Shooter::Setup() {
    std::cout << "Shooter::Setup()\n";
    
    m_shooter_master_speed_controller = new TalonFX(RobotConfiguration::kShooterMasterTalonId);
    m_shooter_slave_speed_controller = new TalonFX(RobotConfiguration::kShooterSlaveTalonId);

    m_shooter_slave_speed_controller->Set(ControlMode::Follower, RobotConfiguration::kShooterMasterTalonId);
    m_shooter_slave_speed_controller->SetInverted(true);

    TalonFXConfiguration shooter_configuration;

    shooter_configuration.nominalOutputReverse = -0.0f;
    shooter_configuration.nominalOutputForward = 0.0f;

    shooter_configuration.peakOutputReverse = -1.0f;
    shooter_configuration.peakOutputForward = +1.0f;

    shooter_configuration.closedloopRamp = 0.5;
    shooter_configuration.openloopRamp = 0.5;

    shooter_configuration.forwardSoftLimitEnable = false;
    shooter_configuration.reverseSoftLimitEnable = false;

    
    shooter_configuration.supplyCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration (true, 
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentLimit,
        RobotConfiguration::kShooterMotorPeakCurrentDurationMs);

    shooter_configuration.slot0.kF = 0.048;
    shooter_configuration.slot0.kP= 0.25;
    
    // shooter_configuration.statorCurrLimit = ctre::phoenix::motorcontrol::SupplyCurrentLimitConfiguration ();
        
    int error = m_shooter_master_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the shooter Talon failed with code:  " << error << "\n";
    }

    int error2 = m_shooter_slave_speed_controller->ConfigAllSettings(shooter_configuration, RC::kTalonTimeoutMs);
    if (error2 != 0) {
        std::cout << "Configuration of the shooter slave Talon failed with code:  " << error2 << "\n";
    }
    // Comment out PID settings so they don't override the Phoenix tuner.
    // m_shooter_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
	m_shooter_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
    m_shooter_slave_speed_controller->SetNeutralMode(NeutralMode::Coast);

    m_shooter_master_speed_controller->SetSensorPhase(false);
}


void Shooter::Shutdown() {
    std::cout << "Shooter::Shutdown()\n";
}

void Shooter::Periodic() {
    frc::SmartDashboard::PutNumber("Shooter Current", m_shooter_master_speed_controller->GetOutputCurrent());            
    frc::SmartDashboard::PutNumber("Shooter Output", m_shooter_master_speed_controller->GetMotorOutputPercent());              

    double shooter_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double shooter_speed_rpm =  KoalafiedUtilities::TalonFXVelocityNativeToRpm(shooter_speed_native) * RC::kShooterMotorGearRatio;

    frc::SmartDashboard::PutNumber("Shooter Speed RPM", shooter_speed_rpm);
}


//==============================================================================
// Operations

void Shooter::ManualDriveShooter(double percentage_speed) {
    m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

void Shooter::AutoDriveDashboard(double dRPM) {
    // std::cout << "in auto drive dash" << std::endl;
    m_shooter_master_speed_controller->Set(ControlMode::Velocity, (dRPM * 2048.0 / (60.0 * 10.0)));

    double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
    frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);
}

bool Shooter::ShooterAtSpeed(double dRPM) {
    double shooter_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
    double shooter_speed_rpm =  KoalafiedUtilities::TalonFXVelocityNativeToRpm(shooter_speed_native);

    double diffpercent = fabs(dRPM-shooter_speed_rpm) * 100 /dRPM;
    return (diffpercent < 5);
}

void Shooter::TestDriveShooter(frc::Joystick* joystick) {  
    double dRPM = (frc::SmartDashboard::GetNumber("dRPM", 4000.0)) * -1;

    if (joystick->GetRawAxis(RobotConfiguration::kJoystickRightTriggerAxis) > 0.0) {
        // Run in close loop and report the error margin

        // Do closed loop velocity control and set a desired speed from
        // our movement value.
        m_shooter_master_speed_controller->Set(ControlMode::Velocity, (dRPM * 2048.0/ (60.0 *10.0)));


        // Get the close loop error and convert to RPM
        double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
        frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);    
    } else {

        double shooter_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(shooter_drive) < RC::kJoystickDeadzone) shooter_drive = 0.0;

        m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, shooter_drive);
        
        // Run in open loo
    // if (joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis)){
    //     m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, joystick->GetRawAxis(RobotConfiguration::kJoystickLeftYAxis));
    // } else {p and calculate a value for F

        // $5 for free - afghan32

        // Calculate the motor output voltage as a fraction
        double motor_output = m_shooter_master_speed_controller->GetMotorOutputVoltage()/
                                m_shooter_master_speed_controller->GetBusVoltage();

        // Get the speed in RPM and convert to the native units of encode counts
        // per 100ms time period (see TSSRM page 88).
        double speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
        // double speed_rpm = speed_native * 60.0 *10.0 / 2048.0;

        // Calculate a feed forward gain (F) for this speed
        double F;
        if (speed_native == 0) {
            F = 0;
        } else
        {
            F = motor_output * 1023.0/speed_native;
        }
        
        // output F to the networktables
        frc::SmartDashboard::PutNumber("Shooter F", F);
        // Output the values if required
        // sstd::cout << "shooter F: " << F << std::endl;
    // }
    }
}

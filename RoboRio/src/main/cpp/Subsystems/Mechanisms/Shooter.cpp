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

    TalonFXConfiguration shooter_configuration;

    shooter_configuration.nominalOutputReverse = -0.0f;
    shooter_configuration.nominalOutputForward = 0.0f;

    shooter_configuration.peakOutputReverse = -1.0f;
    shooter_configuration.peakOutputForward = +1.0f;

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
    // Comment out PID settings so they don't override the Phoenix tuner.
    // m_shooter_master_speed_controller->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Absolute, RC::kTalonPidIdx, RC::kTalonTimeoutMs);
    m_shooter_master_speed_controller->SetSensorPhase(true); // Not reversed
	m_shooter_master_speed_controller->SetNeutralMode(NeutralMode::Coast);
    m_shooter_master_speed_controller->SetSensorPhase(false);

    // Create controllers for each of the 4 drive talons

    // Setup the slave Talons to follow the masters
  
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
    // m_shooter_master_speed_controller->ConfigOpenloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // m_shooter_master_speed_controller->ConfigClosedloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // m_shooter_slave_speed_controller->ConfigOpenloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);
    // m_shooter_slave_speed_controller->ConfigClosedloopRamp(RobotConfiguration::kDriveMotorRampRateS, kTalonTimeoutMs);

    // Voltage compensation TODO Is this required?
    // TSSRM Section 9.2 (page 60)
//  virtual ctre::phoenix::ErrorCode ConfigVoltageCompSaturation(double voltage, int timeoutMs);
//	virtual ctre::phoenix::ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples, int timeoutMs);
//	virtual void EnableVoltageCompensation(bool enable);

    // Set the continuous and peak current limits, for all motors. The is for all control modes (i.e. open/closed loop)
    // TSSRM Section 9.3 (page 62)




    // Set the PID controller parameters for the closed loop control of the master
    // motors. Use the profile slot for running the robot.
    // m_left_master_speed_controller->SelectProfileSlot(kRunProfileSlotIdx, kPidDefaultIdx);
    // m_left_master_speed_controller->Config_kF(kRunProfileSlotIdx, RobotConfiguration::kDriveBasePidF, kTalonTimeoutMs);
    // m_left_master_speed_controller->Config_kP(kRunProfileSlotIdx, RobotConfiguration::kDriveBasePidP, kTalonTimeoutMs);
    // m_left_master_speed_controller->Config_kI(kRunProfileSlotIdx, RobotConfiguration::kDriveBasePidI, kTalonTimeoutMs);
    // m_left_master_speed_controller->Config_kD(kRunProfileSlotIdx, RobotConfiguration::kDriveBasePidD, kTalonTimeoutMs);

//    m_left_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_13_Base_PIDF0_, 20, kTalonTimeoutMs);
//    m_left_master_speed_controller->SetStatusFramePeriod(StatusFrame::Status_10_MotionMagic_, 20, kTalonTimeoutMs);
 

}

void Shooter::Shutdown() {
    std::cout << "Shooter::Shutdown()\n";
}

void Shooter::Periodic(bool show_dashboard)
{
    if (show_dashboard) {
        frc::SmartDashboard::PutNumber("Shooter Current", m_shooter_master_speed_controller->GetOutputCurrent());            
        frc::SmartDashboard::PutNumber("Shooter Output", m_shooter_master_speed_controller->GetMotorOutputPercent());              

        double shooter_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
        double shooter_speed_rpm =  shooter_speed_native * 60.0 *10.0 / 2048.0;

        frc::SmartDashboard::PutNumber("Shooter Speed RPM", shooter_speed_rpm);
    }
}


//==============================================================================
// Operations

void Shooter::ManualDriveShooter(double percentage_speed) {
    m_shooter_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

void Shooter::AutoDriveDashboard(double dRPM) {
    std::cout << "in auto drive dash" << std::endl;
    m_shooter_master_speed_controller->Set(ControlMode::Velocity, (dRPM * 2048.0 / (60.0 * 10.0)));

    double closed_loop_error_native = m_shooter_master_speed_controller->GetClosedLoopError(RC::kTalonPidIdx);
    frc::SmartDashboard::PutNumber("Shooter Closed Loop Error", closed_loop_error_native);
}

bool Shooter::ShooterAtSpeed(double dRPM) {
   double adjustedRPM = (m_shooter_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx) * 60.0 * 10.0 / 2048.0) * -0.925;
    if (adjustedRPM < dRPM){
        return true;
    } else {
        return false;
    }
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

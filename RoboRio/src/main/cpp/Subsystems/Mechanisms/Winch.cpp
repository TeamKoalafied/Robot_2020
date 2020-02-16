//==============================================================================
// Winch.cpp
//==============================================================================

#include "Winch.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"
#include "../../Subsystems/DriveBase.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>


namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Winch::Winch()  {
}

Winch::~Winch() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Winch::Setup() {
    std::cout << "Winch::Setup()\n";
    
    // Create controllers for each of the 4 drive talons
    m_winch_master_speed_controller = new TalonSRX(RC::kWinchTalonId);

    TalonSRXConfiguration winch_configuration;

    // Current limit
    winch_configuration.continuousCurrentLimit = RC::kWinchMotorContinuousCurrentLimit;
    winch_configuration.peakCurrentLimit = RC::kWinchMotorPeakCurrentLimit;
    winch_configuration.peakCurrentDuration = RC::kWinchMotorPeakCurrentDurationMs;

    // Nominal and peak outputs
    // TODO These are defaults. Winch drive is very asynmetric so these should probably be
    // set to something non-standard.
    winch_configuration.nominalOutputForward = 0.0;
    winch_configuration.nominalOutputReverse = 0.0;
    winch_configuration.peakOutputForward = 1.0;
    winch_configuration.peakOutputReverse = -1.0;

    // Ramp rates
    winch_configuration.openloopRamp = 0.1;
    winch_configuration.closedloopRamp = 0.1;

    // Voltage compensation

    // Feedback sensor
    winch_configuration.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Absolute;

    // Soft limits
	double winch_circumference_inch = RC::kWinchDiameterInch * M_PI;
	double max_extension_revolutions = RC::kWinchMaximumExtensionInch/winch_circumference_inch;
    winch_configuration.forwardSoftLimitThreshold = 0;
    winch_configuration.reverseSoftLimitThreshold = -max_extension_revolutions * RC::kCtreEnocderCounts;
    //winch_configuration.forwardSoftLimitEnable = false;
    //winch_configuration.reverseSoftLimitEnable = false;
    std::cout << "reverseSoftLimitThreshold = " << winch_configuration.reverseSoftLimitThreshold << "\n";

    winch_configuration.forwardSoftLimitEnable = true;
    winch_configuration.reverseSoftLimitEnable = true;

    // Limit switches


    int error = m_winch_master_speed_controller->ConfigAllSettings(winch_configuration, RC::kTalonTimeoutMs);
    if (error != 0) {
        std::cout << "Configuration of the winch Talon failed with code:  " << error << "\n";
    }


    // Perform non-configuration setup

    m_winch_master_speed_controller->SetSensorPhase(false); // Not reversed
    m_winch_master_speed_controller->EnableCurrentLimit(true);
	m_winch_master_speed_controller->SetNeutralMode(NeutralMode::Brake);
    m_winch_master_speed_controller->SetSelectedSensorPosition(0,RC::kTalonPidIdx);
    
}

void Winch::Shutdown() {
    std::cout << "Winch::Shutdown()\n";
}

void Winch::Periodic(bool show_dashboard)
{
    if (show_dashboard) {
        frc::SmartDashboard::PutNumber("Winch Current", m_winch_master_speed_controller->GetOutputCurrent());        
        frc::SmartDashboard::PutNumber("Winch Output", m_winch_master_speed_controller->GetMotorOutputPercent());         
        double winch_speed_native = m_winch_master_speed_controller->GetSelectedSensorVelocity(RC::kTalonPidIdx);
        double winch_speed_rpm =   KoalafiedUtilities::TalonSRXCtreVelocityNativeToRpm(winch_speed_native);
        frc::SmartDashboard::PutNumber("Winch Speed RPM", winch_speed_rpm);
        frc::SmartDashboard::PutNumber("Winch Position Inch", GetWinchPositionInch());
        frc::SmartDashboard::PutNumber("Winch Encoder Position",m_winch_master_speed_controller->GetSelectedSensorPosition());

		Faults talon_faults;
		m_winch_master_speed_controller->GetFaults(talon_faults);
		// frc::SmartDashboard::PutBoolean("Arm RLimit", arm_talon_faults.ReverseLimitSwitch);
		//bool reverse_limit = m_arm_speed_controller->GetSensorCollection().IsRevLimitSwitchClosed();
		frc::SmartDashboard::PutBoolean("Winch FLimit", talon_faults.ForwardSoftLimit);
		frc::SmartDashboard::PutBoolean("Winch RLimit", talon_faults.ReverseSoftLimit);



        DriveBase& drive_base = DriveBase::GetInstance();
        int16_t acceleration_xyz[3];
        drive_base.GetPigeonIMU()->GetBiasedAccelerometer(acceleration_xyz);
        frc::SmartDashboard::PutNumber("Accelerometer X", acceleration_xyz[0]);
        frc::SmartDashboard::PutNumber("Accelerometer Y", acceleration_xyz[1]);
        frc::SmartDashboard::PutNumber("Accelerometer Z", acceleration_xyz[2]);
    }
}

//==============================================================================
// Operations

void Winch::ManualDriveWinch(double percentage_speed) {
    m_winch_master_speed_controller->Set(ControlMode::PercentOutput, percentage_speed);
}

double Winch::GetWinchPositionInch() {
    int encoder_count = m_winch_master_speed_controller->GetSelectedSensorPosition(RC::kTalonPidIdx);
	double revolutions = (double)encoder_count / (double)RC::kCtreEnocderCounts;
	double winch_circumference_inch = RC::kWinchDiameterInch * M_PI;
	return winch_circumference_inch * revolutions;
}

void Winch::TestDriveWinch(frc::Joystick* joystick) {
    double joystick_value = joystick->GetRawAxis(RC::kJoystickRightYAxis);

    if (fabs(joystick_value) < RC::kJoystickDeadzone) joystick_value = 0.0;

    joystick_value *= 0.5;

    m_winch_master_speed_controller->Set(ControlMode::PercentOutput, joystick_value);
}

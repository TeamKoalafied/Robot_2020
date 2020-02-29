//==============================================================================
// Kicker.cpp
//==============================================================================

#include "Kicker.h"

#include "../../RobotConfiguration.h"
#include "../../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <ctre/Phoenix.h>
#include <frc/DoubleSolenoid.h>


namespace RC = RobotConfiguration;




//==============================================================================
// Construction

Kicker::Kicker()  {
}

Kicker::~Kicker() {
    Shutdown();
}


//==============================================================================
// Mechanism Lifetime - Setup, Shutdown and Periodic

void Kicker::Setup() {
    std::cout << "Kicker::Setup()\n";

    m_kicker_double_solenoid = new frc::DoubleSolenoid(0,1);
}

void Kicker::Shutdown() {
    std::cout << "Kicker::Shutdown()\n";
}

void Kicker::Periodic() {      

}

//==============================================================================
// Operations

void Kicker::SetStop() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kForward);

//    m_kicker_double_solenoid->Get();
}

void Kicker::SetShoot() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
}

void Kicker::SetOff() {
    m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
}

void Kicker::KickBall() {

}

void Kicker::TestDriveKicker(frc::Joystick* joystick) {
    if (joystick->GetRawButton(RC::kJoystickAButton)) {
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    } else if (joystick->GetRawButton(RC::kJoystickBButton)) {
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    }
}

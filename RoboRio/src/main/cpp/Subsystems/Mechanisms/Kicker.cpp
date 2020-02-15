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

    frc::DoubleSolenoid m_kicker_double_solenoid {17, 18};
}

void Kicker::Shutdown() {
    std::cout << "Kicker::Shutdown()\n";
}

void Kicker::Periodic(bool periodicDashboard)
{      

}

//==============================================================================
// Operations

void Kicker::ManualDriveKicker() {

}

void Kicker::AutoDriveDashboard() {

}

void Kicker::TestDriveKicker(frc::Joystick* joystick) {
    if(joystick->GetRawButton(RC::kJoystickAButton)){
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kForward);
    } else if (joystick->GetRawButton(RC::kJoystickBButton)){
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kReverse);
    } else {
        m_kicker_double_solenoid->Set(frc::DoubleSolenoid::Value::kOff);
    }
}

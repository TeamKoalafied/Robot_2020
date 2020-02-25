//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "Mechanisms/Shooter.h"
#include "Mechanisms/Indexer.h"
#include "Mechanisms/Winch.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Kicker.h"
#include "../RobotConfiguration.h"
#include "../KoalafiedUtilities.h"

#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
#include <math.h>

namespace RC = RobotConfiguration;


//==============================================================================
// Construction

Manipulator::Manipulator() :
    TSingleton<Manipulator>(this),
    JoystickSubsystem("Manipulator", RC::kJoystickPortOperator) {

    m_shooter = new Shooter;
    m_indexer = new Indexer;
    m_winch = new Winch;
    m_intake = new Intake;
    m_kicker = new Kicker;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    // frc::SmartDashboard::PutNumber("Shooter Current", m_shooter_master_speed_controller->GetOutputCurrent());        
    // frc::SmartDashboard::PutNumber("Shooter Slave Current", m_shooter_slave_speed_controller->GetOutputCurrent());        
    // frc::SmartDashboard::PutNumber("Shooter Output", m_shooter_master_speed_controller->GetMotorOutputPercent());        
    // frc::SmartDashboard::PutNumber("Shooter Slave Output", m_shooter_slave_speed_controller->GetMotorOutputPercent());        

    // double shooter_speed_native = m_shooter_master_speed_controller->GetSelectedSensorVelocity(kPidDefaultIdx);
    // double shooter_speed_rpm =  shooter_speed_native * 60.0 *10.0 / 4096.0;

    // frc::SmartDashboard::PutNumber("Shooter Speed RPM", shooter_speed_rpm);        


    m_shooter->Periodic(true);
    m_indexer->Periodic(true);
    // m_winch->Periodic(true);
    m_intake->Periodic();
    m_kicker->Periodic(true);
}

//==========================================================================
// Joystick Operation (from JoystickSubsystem)

void Manipulator::JoystickControlStarted() {
    JoystickSubsystem::JoystickControlStarted();
}

void Manipulator::DoJoystickControl() {
    frc::Joystick* joystick = GetJoystick();

    // IMPORTANT: Only one of thes following lines should ever be uncommented at a time
    DoManualJoystickControl(joystick);
    // m_shooter->TestDriveShooter(joystick);
    // m_indexer->TestDriveIndexer(joystick);
    // m_winch->TestDriveWinch(joystick);
    // m_intake->TestDriveIntake(joystick);
    // m_kicker->TestDriveKicker(joystick);
}

void Manipulator::JoystickControlStopped() {
    JoystickSubsystem::JoystickControlStopped();
}


//==============================================================================
// Setup and Shutdown

void Manipulator::Setup() {
    std::cout << "Manipulator::Setup()\n";
    frc::SmartDashboard::PutNumber("dRPM", 4000.0);
    // Setup all the mechanisms
    m_shooter->Setup();
    m_indexer->Setup();
    m_winch->Setup();
    m_intake->Setup();
    m_kicker->Setup();
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // Shutdown all the mechanisms
    m_shooter->Shutdown();
    m_indexer->Shutdown();
    m_winch->Shutdown();
    m_intake->Shutdown();
    m_kicker->Shutdown();
}

//==========================================================================
// Mechanism Access

//==========================================================================
// Joystick Control

void Manipulator::DoManualJoystickControl(frc::Joystick* joystick)
{
    double dRPM = (frc::SmartDashboard::GetNumber("dRPM", 4000.0)) * -1;

    // Run indexer and intake together
    if (joystick->GetRawButton(RC::kJoystickAButton)) {
        m_indexer->ManualDriveIndexer(0.5);
        m_intake->Run();
    } else {
        m_intake->Stop();
        m_indexer->ManualDriveIndexer(0);
    }

    // Shoot, then kick
    if (joystick->GetRawButton(RC::kJoystickBButton)) {
        m_shooter->AutoDriveDashboard(dRPM);
        if (frc::SmartDashboard::GetNumber("Shooter Speed RPM", 0)*1.075 > dRPM){
            m_kicker->SetShoot();
        }
    } else {
        m_shooter->ManualDriveShooter(0);
    }

    if (joystick->GetRawButton(RC::kJoystickYButton)) {
        m_kicker->SetStop();
    }
    if (joystick->GetRawButton(RC::kJoystickXButton)) {
        m_kicker->SetShoot();
    }

    // if (joystick->GetRawButton(RC::kJoystickXButton)) {
    //     m_shooter->AutoDriveDashboard(dRPM);
    //     std::cout << "shooting in manipulator\n";
    //     // if (m_shooter->ShooterAtSpeed(dRPM)) {
    //     //     // m_indexer->AutoDriveDashboard(true);
    //     // } else {
    //     //     // m_indexer->AutoDriveDashboard(false);
    //     // }
    // } else {
    //     // Set the motor to approx 1000-1100RPM (-0.2/6000)
    //     m_shooter->ManualDriveShooter(0.0);
    //     // m_indexer->AutoDriveDashboard(false);
    // }

    // double indexer_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
    // double intake_drive = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    // if (fabs(indexer_drive) < RC::kJoystickDeadzone) indexer_drive = 0.0;
    // if (fabs(intake_drive) < RC::kJoystickDeadzone) intake_drive = 0.0;

    // m_indexer->ManualDriveIndexer(indexer_drive);
    // m_intake->ManualDriveIntake(intake_drive);

    

    // if (joystick->GetRawButton(RC::kJoystickRTrigButton)){
    //     m_intake->OperateSolenoid(true);
    // } else if (joystick->GetRawButton(RC::kJoystickLTrigButton)){
    //     m_intake->OperateSolenoid(false);
    // }
    // // double shooter_drive = joystick->GetRawAxis(RC::kJoystickLeftYAxi>s);
    // // // std::cout << "Shooter Drive" << shooter_drive << "\n";
    // // if (fabs(shooter_drive) < RC::kJoystickDeadzone) shooter_drive = 0.0;

    // double indexer_drive = joystick->GetRawAxis(RC::kJoystickRightYAxis);
    // //std::cout << "Shooter Drive" << shooter_drive << "\n";
    // if (fabs(indexer_drive) < RC::kJoystickDeadzone) indexer_drive = 0.0;
    
}
    
    

    // if button pressed, then auto drive shooter, in the if statement, then if shooter is at speed, drive indexer

        // Calculate the motor output voltage as a fraction
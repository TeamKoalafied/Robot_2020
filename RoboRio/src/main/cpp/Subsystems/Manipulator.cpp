//==============================================================================
// Manipulator.cpp
//==============================================================================

#include "Manipulator.h"

#include "Mechanisms/Shooter.h"
#include "Mechanisms/Indexer.h"
#include "Mechanisms/Winch.h"
#include "Mechanisms/Intake.h"
#include "Mechanisms/Kicker.h"

#include "Mechanisms/DistanceSensor.h"

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

    m_distanceSensor = new DistanceSensor;

    m_state = State::Idle;
}

Manipulator::~Manipulator() {
    Shutdown();
}


//==============================================================================
// frc::Subsystem Function Overrides

void Manipulator::Periodic() {
    m_shooter->Periodic();
    m_indexer->Periodic();
    m_winch->Periodic();
    m_intake->Periodic();
    m_kicker->Periodic();
    m_distanceSensor->Periodic(true);
    switch (m_state) {
        case State::Intaking: UpdateIntakingState(); break;
        case State::Shooting: UpdateShootingState(); break;
        case State::Climbing: UpdateClimbingState(); break;
        case State::Idle: break;
    }    
    
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

    m_distanceSensor->Setup();
}

void Manipulator::Shutdown() {
    std::cout << "Manipulator::Shutdown()\n";

    // Shutdown all the mechanisms
    m_shooter->Shutdown();
    m_indexer->Shutdown();
    m_winch->Shutdown();
    m_intake->Shutdown();
    m_kicker->Shutdown();

    m_distanceSensor->Shutdown();
}

//==========================================================================
// Mechanism Access

//==========================================================================
// Joystick Control

void Manipulator::DoManualJoystickControl(frc::Joystick* joystick) {


    bool shoot_button = joystick->GetRawButton(RC::kJoystickYButton);
    bool intake_button = joystick->GetRawButton(RC::kJoystickAButton);

    // Calculate the new state based on the buttons pressed. This will get more complex
    // with climbing is added.
    State new_state = State::Idle;
    if (intake_button) {
        new_state = State::Intaking;
    }
    else if (shoot_button) {
        new_state = State::Shooting;
    }

    // Update the state if required
    if (new_state != m_state) {
        ChangeState(new_state);
    }

    // If in the idle state allow special override controls
    if (m_state == State::Idle) {
        double dRPM = (frc::SmartDashboard::GetNumber("dRPM", 4000.0)) * -1;

        // Run indexer and intake together
        if (joystick->GetPOV(0) == RC::kJoystickPovLeft) {
            m_indexer->ManualDriveIndexer(0.5);
            m_intake->Run();
        } else {
            m_intake->Stop();
            m_indexer->ManualDriveIndexer(0);
        }

        // Shoot, then kick
        if (joystick->GetRawButton(RC::kJoystickBButton)) {
            m_shooter->AutoDriveDashboard(dRPM);
            if (frc::SmartDashboard::GetNumber("Shooter Speed RPM", 0)*1.075 < dRPM){
                m_kicker->SetShoot();
            }
        } else {
            m_shooter->ManualDriveShooter(0);
        }

        double rightYAxisJoystickValue = joystick->GetRawAxis(RC::kJoystickRightYAxis);
        if (fabs(rightYAxisJoystickValue) < RC::kJoystickDeadzone) rightYAxisJoystickValue = 0.0;
        m_shooter->ManualDriveShooter(rightYAxisJoystickValue);
      
        double leftYAxisJoystickValue = joystick->GetRawAxis(RC::kJoystickLeftYAxis);
        if (fabs(leftYAxisJoystickValue) < RC::kJoystickDeadzone) leftYAxisJoystickValue = 0.0;
        m_indexer->VelocityDriveIndexer(leftYAxisJoystickValue * 0.4);

        if (joystick->GetPOV(0) == RC::kJoystickPovRight) {
            m_kicker->SetStop();
        }
        if (joystick->GetRawButton(RC::kJoystickXButton)) {
            m_kicker->SetShoot();
        }
    }
}


//==========================================================================
// State Management

void Manipulator::ChangeState(State new_state) {
    // If the state is not changing do nothing
    if (new_state == m_state) return;

    // Leave the current state
    switch (m_state) {
        case State::Intaking: LeaveIntakingState(); break;
        case State::Shooting: LeaveShootingState(); break;
        case State::Climbing: LeaveClimbingState(); break;
        case State::Idle: break;
    }

    // Record the new state as active
    m_state = new_state;


    // Enter the new state
    switch (m_state) {
        case State::Intaking: EnterIntakingState(); break;
        case State::Shooting: EnterShootingState(); break;
        case State::Climbing: EnterClimbingState(); break;
        case State::Idle: break;
    }

    switch (m_state) {
        case State::Intaking: frc::SmartDashboard::PutString("m_state", "Intaking"); break;
        case State::Shooting: frc::SmartDashboard::PutString("m_state", "Shooting"); break;
        case State::Climbing: frc::SmartDashboard::PutString("m_state", "Climbing"); break;
        case State::Idle: frc::SmartDashboard::PutString("m_state", "Idle"); break;
    }
}

//==========================================================================
// Intaking State

void Manipulator::EnterIntakingState() {
    m_intake->Extend();
    m_intake->Run();
}

void Manipulator::LeaveIntakingState() {
    m_intake->Retract();
    m_intake->Stop();
}

void Manipulator::UpdateIntakingState() {
    const double kBallDistance = 7;
    // If we sense the ball run the indexer for a short time
    if (m_distanceSensor->GetDistance() < kBallDistance) {
        m_indexer->VelocityDriveIndexer(0.1);

        // m_indexer->ManualDriveIndexer(0.5);
    }  else {
        m_indexer->ManualDriveIndexer(0);
    }
}


//==========================================================================
// Shooting State

void Manipulator::EnterShootingState() {

    // TODO at the start of the game we should start assuming there is a ball in the kicker and
    // maybe at other times too.

    m_shooting_state = ShootingState::DrivingBallsUp;
    m_indexer->VelocityDriveIndexer(0.4);
    m_shoot_timer.Start();
    m_shoot_timer.Reset();
}

void Manipulator::LeaveShootingState() {
    m_shooter->ManualDriveShooter(0);
    m_kicker->SetStop();
}

void Manipulator::UpdateShootingState() {

    // If we need to turn to the target do that

    // Calculate the rmp required for the current distance to target
    double required_rmp = (frc::SmartDashboard::GetNumber("dRPM", 4000.0)) * -1;
    m_shooter->AutoDriveDashboard(required_rmp);


    switch (m_shooting_state) {
        case ShootingState::BallInKicker:
            // If we are on target, up to speed and there is a ball in the kicker then kick it!
            if (m_shooter->ShooterAtSpeed(required_rmp)) {
                m_kicker->SetShoot();
                m_shooting_state = ShootingState::KickingBall;
                m_shoot_timer.Reset();
            }
            break;
        case ShootingState::DrivingBallsUp:
            // If the indexer current is high then the balls have reach the end so drive backwards
            // slightly to give a gap.
            if (m_indexer->HasHighCurrent()) {
                m_indexer->ManualDriveIndexer(-0.5);
                m_shooting_state = ShootingState::SettlingBallsBack;
                m_shoot_timer.Reset();
            }
            // If we drive balls up for 1s and there is no high current jump straight to
            // trying to shoot (probably we are empty)
            if (m_shoot_timer.Get() > 1.0) {
                m_indexer->ManualDriveIndexer(0.0);
                m_shooting_state = ShootingState::BallInKicker;
            }
            break;
        case ShootingState::SettlingBallsBack:
            // After 100ms of driving back we are ready to shoot
            if (m_shoot_timer.Get() > 0.1) {
                m_indexer->ManualDriveIndexer(0.0);
                m_shooting_state = ShootingState::BallInKicker;
            }
            break;
        case ShootingState::KickingBall:
            // After 200ms return the kicker and start moving the next ball up
            if (m_shoot_timer.Get() > 0.2) {
                m_kicker->SetStop();

                m_shooting_state = ShootingState::DrivingBallsUp;
                // m_indexer->ManualDriveIndexer(0.5);
                m_indexer->VelocityDriveIndexer(0.5);
                m_shoot_timer.Reset();
            }
            break;
    }



 
}


//==========================================================================
// Climbing State

void Manipulator::EnterClimbingState() {

}

void Manipulator::LeaveClimbingState() {

}

void Manipulator::UpdateClimbingState() {

}


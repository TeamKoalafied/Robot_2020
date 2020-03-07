//==============================================================================
// Manipulator.h
//==============================================================================

#ifndef Manipulator_H
#define Manipulator_H

#include "../TSingleton.h"
#include "JoystickSubsystem.h"
#include <ctre/Phoenix.h>
#include <frc/Timer.h>

class Shooter;
class Indexer;
class Winch;
class Intake;
class Kicker;

class DistanceSensor;

// The Manipulator subsystem controls the ???.
class Manipulator : public TSingleton<Manipulator>, public JoystickSubsystem {
public:
    //==========================================================================
    // Construction

    // Constructor
    Manipulator();

    // Destructor
    virtual ~Manipulator();


    //==========================================================================
    // frc::Subsystem Function Overrides
    virtual void Periodic() override;
    //==========================================================================

    //==========================================================================
    // Joystick Operation (from JoystickSubsystem)
    virtual void JoystickControlStarted() override;
    virtual void DoJoystickControl() override;
    virtual void JoystickControlStopped() override;
    //==========================================================================


    //==========================================================================
    // Setup and Shutdown

    // Setup the pneumatics for operation
    void Setup();

    // Shutdown the pneumatics
    void Shutdown();


    //==========================================================================
    // Mechanism Access


private:
    //==========================================================================
    // Private Nested Types

    // Overall operational state ot the manipulator
    enum State {
        Idle,
        Intaking,
        Shooting,
        Climbing
    };

    // State of the current shooting operation
    enum ShootingState {
        BallInKicker,
        DrivingBallsUp,
        SettlingBallsBack,
        KickingBall
    };


    //==========================================================================
    // Joystick Control

    // Do manual control of the manipulator with the joystick.
    // 
    // joystick - joystick to use
    void DoManualJoystickControl(frc::Joystick* joystick);

    //==========================================================================
    // State Management

    void ChangeState(State new_state);

    //==========================================================================
    // Intaking State

    void EnterIntakingState();
    void LeaveIntakingState();
    void UpdateIntakingState();

    //==========================================================================
    // Shooting State

    void EnterShootingState();
    void LeaveShootingState();
    void UpdateShootingState();

    //==========================================================================
    // Climbing State

    void EnterClimbingState();
    void LeaveClimbingState();
    void UpdateClimbingState();


    //==========================================================================
    // Member Variables

    Intake* m_intake;           // Intake mechanism
    Indexer* m_indexer;         // Indexer mechanism
    Kicker* m_kicker;           // Kicker mechanism
    Shooter* m_shooter;         // Shooter mechanism
    Winch* m_winch;             // Winch mechanism
    DistanceSensor* m_distanceSensor;

    State m_state;                  // Current state of the mechanism

    // Shooting State
    frc::Timer m_shoot_timer;       // Timer used to time events during shooting
    ShootingState m_shooting_state; // State in the shooting state machine

    static const double kIndexerDriveUpVelocity;      // Relative velocity for driving the balls up the indexer when shooting
    static const double kIndexerDriveBackVelocity;    // Relative velocity for driving the balls back down the indexer when shooting
    static constexpr double kTestVelocityFactor = 0.5;      // Ratio to slow down movement by when testing
    
};

#endif  // Manipulator_H

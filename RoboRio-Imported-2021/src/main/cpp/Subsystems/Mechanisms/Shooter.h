//==============================================================================
// Shooter.h
//==============================================================================

#ifndef Shooter_H
#define Shooter_H

#include <ctre/Phoenix.h>
#include "../../RobotConfiguration.h"
#include <frc/Timer.h>
namespace frc { 
    class Joystick;
}

// The Shooter mechanism is part of the Manipulator subsystem. It controls the
// position (in, out, vertical) and roller roation of the roller intake for
// grabbing the ball.
class Shooter  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Shooter();

    // Destructor
    virtual ~Shooter();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Shooter for operation
    void Setup();

    // Shutdown the Shooter
    void Shutdown();

    // Perform periodic updates for the Shooter
//    double shooter_speed_rpm;
    void Periodic();



    //==========================================================================
    // Operations

    // Manually drive the intake at a given percentage of motor output. The intake will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveShooter(double percentage_output);

    void AutoDriveDashboard(double dRPM);

    bool ShooterAtSpeed(double dRPM);

    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveShooter(frc::Joystick* joystick);
    int getRPM();
private:

    //==========================================================================
    // Member Variables

    TalonFX* m_shooter_master_speed_controller;
    TalonFX* m_shooter_slave_speed_controller;
};

#endif  // Shooter_H
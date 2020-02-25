//==============================================================================
// Kicker.h
//==============================================================================

#ifndef Kicker_H
#define Kicker_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include <frc/DoubleSolenoid.h>
#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Kicker mechanism is part of the Manipulator subsystem. It controls the
// 
// 
class Kicker  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Kicker();

    // Destructor
    virtual ~Kicker();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Kicker for operation
    void Setup();

    // Shutdown the Kicker
    void Shutdown();

    // Perform periodic updates for the Kicker

    void Periodic();
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool periodicDashboard);


    //==========================================================================
    // Operations

    // Set the kicker to the stop position (i.e. blocking the end of the indexer)
    void SetStop();

    // Set the kicker to the shoot position (i.e. pushing the ball into the shooter)
    void SetShoot();

    // Set the kicker off (i.e. not driving the cylinder in either direction)
    void SetOff();

    // Kick the ball into the shooter.
    void KickBall();
    
    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveKicker(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    // Solenoid that controls the kicker position
    frc::DoubleSolenoid* m_kicker_double_solenoid;

    // Timer used to actuate the kicker for the correct length of time to shoot the ball
    frc::Timer m_kick_timer;
};

#endif  // Kicker_H

//==============================================================================
// Kicker.h
//==============================================================================

#ifndef Kicker_H
#define Kicker_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
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
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic();


    //==========================================================================
    // Operations

    // Manually drive the intake at a given percentage of motor output. The intake will not
    // drive past its end limits.
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveKicker();

    void AutoDriveDashboard();
    
    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveKicker(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

};

#endif  // Kicker_H
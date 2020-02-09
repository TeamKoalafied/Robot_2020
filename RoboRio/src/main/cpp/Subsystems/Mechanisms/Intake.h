//==============================================================================
// Intake.h
//==============================================================================

#ifndef Intake_H
#define Intake_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Intake mechanism is part of the Manipulator subsystem. It controls the
// 
// 
class Intake  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Intake();

    // Destructor
    virtual ~Intake();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Intake for operation
    void Setup();

    // Shutdown the Intake
    void Shutdown();

    // Perform periodic updates for the Intake
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
    void ManualDriveIntake();

    void AutoDriveDashboard();
    
    // Perform testing of the intake using the joystick. This function is only for testing the
    // pivot and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveIntake(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

      TalonSRX* intake_master_speed_controller;
};

#endif  // Intake_H

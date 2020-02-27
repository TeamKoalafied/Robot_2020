//==============================================================================
// Indexer.h
//==============================================================================

#ifndef Indexer_H
#define Indexer_H

#include <ctre/Phoenix.h>
#include <frc/Timer.h>
#include "../../RobotConfiguration.h"

namespace frc { 
    class Joystick;
}

// The Indexer mechanism is part of the Manipulator subsystem. It controls the
// 
// 
class Indexer  {
public:
    //==========================================================================
    // Construction

    // Constructor
    Indexer();

    // Destructor
    virtual ~Indexer();


    //==========================================================================
    // Mechanism Lifetime - Setup, Shutdown and Periodic

    // Setup the Indexer for operation
    void Setup();

    // Shutdown the Indexer
    void Shutdown();

    // Perform periodic updates for the Indexer
    //
    // show_dashboard - whether to show debugging information on the dashboard
    void Periodic(bool show_dashboard);


    //==========================================================================
    // Operations

    // Manually drive the indexer at a given percentage of motor output
    //
    // percentage_output - Percentage output to drive at. Positive is rotate to the
    //      front and negative is to the back.
    void ManualDriveIndexer(double percentage_output);

    void AutoDriveDashboard(bool feed_desire);
    
    // Perform testing of the indexer using the joystick. This function is only for testing the
    // indexer and may use any controls of the joystick in an ad hoc fashion.
    //
    // joystick - Joystick to use
    void TestDriveIndexer(frc::Joystick* joystick);

private:

    //==========================================================================
    // Member Variables

    TalonSRX* m_indexer_master_speed_controller;   // Motor controller for the indexer
};

#endif  // Indexer_H

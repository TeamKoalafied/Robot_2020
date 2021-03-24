//==============================================================================
// AutonomousCommand.h
//==============================================================================

#ifndef AutonomousCommand_H
#define AutonomousCommand_H

namespace frc { class Command; }

// Namespace with functions for setting up the command runs the autonomous period for the robot.
namespace AutonomousCommand {

    //==========================================================================
	// Dashboard Setup

	// Set up controls on the dashboard for choosing autonomous parameters
	void SetupDashboard();

    // Create the autonomous command to run the currently selected strategy
    //
    // Returns the command. Ownership is taken by the caller.
    frc::Command* CreateAutonomousCommand();
}
#endif  // AutonomousCommand_H

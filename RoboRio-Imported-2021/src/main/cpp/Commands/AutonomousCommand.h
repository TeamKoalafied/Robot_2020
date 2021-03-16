//==============================================================================
// AutonomousCommand.h
//==============================================================================

#ifndef AutonomousCommand_H
#define AutonomousCommand_H

#include <frc/commands/CommandGroup.h>
#include <frc/smartdashboard/SendableChooser.h>

class PathFollower;
class RobotPath;


// This command runs the autonomous period for the robot.
class AutonomousCommand : public frc::CommandGroup {
public:
	// The strategy for autonomous mode
	enum class Strategy {
		kAutoNavSlalom,			// AutoNav Challenge Slalom path
		kAutoNavBarrelRace,		// AutoNav Challenge Barrel Race path
		kAutoNavBounce,			// AutoNav Challenge Bounce path
		kGalacticSearchARed,	// Galactic Search Challenge Path A Red Markers
		kGalacticSearchABlue,	// Galactic Search Challenge Path A Blue Markers
		kGalacticSearchBRed,	// Galactic Search Challenge Path B Red Markers
		kGalacticSearchBBlue	// Galactic Search Challenge Path B Blue Markers
	};

	
    //==========================================================================
    // Construction

    // Constructor
	AutonomousCommand();

	//==========================================================================
	// Command Setup

	// Setup the command using information from the dashboard and the game data/
	// This should be called repeatedly (beginning in AutonomouseInit()) until it
	// returns true and then the this command can be started.
	//
	// Returns true if the game data was available and the command setup was complete
	bool SetupCommand();

    //==========================================================================
	// Dashboard Setup

	// Set up controls on the dashboard for choosing autonomous parameters
	static void SetupDashboard();

	// Echo the settings to the dashboard so we can check that they are working
	static void EchoSettingsToDashboard();

    // Create the autonomous command to run the currently selected strategy
    //
    // Returns the command. Ownership is taken by the caller.
    static frc::Command* CreateAutonomousCommand();

private:

    static PathFollower* CreatePathPointsFollower(RobotPath* robot_path);


    //==========================================================================
    // Member Variables

	Strategy m_strategy;		// The strategy for autonomous mode

	static frc::SendableChooser<Strategy> ms_strategy_chooser;
};

#endif  // AutonomousCommand_H

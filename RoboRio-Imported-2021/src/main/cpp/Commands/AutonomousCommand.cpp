//==============================================================================
// AutonomousCommand.cpp
//==============================================================================

#include "AutonomousCommand.h"

#include "ChallengePaths.h"
#include "DrivePathFollower.h"
#include "MechanismController2020.h"
#include "PathFollower/PathPointsFollower.h"
#include "../Subsystems/DriveBase.h"

#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>

//==============================================================================
// This is an anonymous namespace, which means that all the stuff in it is only
// available in this file
namespace {
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

// Smart dashboard chooser for the autonomouse strategy
frc::SendableChooser<Strategy> ms_strategy_chooser;

// Build a single structure to keep all the mappings in one place to reduce risk of errors
struct AutoModeMapping {
	std::string label;
	Strategy strategy;
};

// Define the mapping between the labels on the dashboard and the position and strategy
const AutoModeMapping kAutoModes[] = {
		{ "AutoNav Slalom",         Strategy::kAutoNavSlalom },
		{ "AutoNav Barrel Race",    Strategy::kAutoNavBarrelRace },
		{ "AutoNav Bounce",         Strategy::kAutoNavBounce },
		{ "Galactic Search A Red",  Strategy::kGalacticSearchARed },
		{ "Galactic Search A Blue", Strategy::kGalacticSearchABlue },
		{ "Galactic Search B Red",  Strategy::kGalacticSearchBRed },
		{ "Galactic Search B Blue", Strategy::kGalacticSearchBBlue },
};

// Compute the number of entries so we can create the string array for populating the default dashboard
const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);

}


//==========================================================================
// Dashboard Setup

void AutonomousCommand::SetupDashboard() {
	// Setup the chooser for determining the strategy for the autonomous period
   	for (int i = 0; i < kAutoModeCount; i++) {
    	ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
	ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
	frc::SmartDashboard::PutData("Autonomous Strategy", &ms_strategy_chooser);
}

frc::Command* AutonomousCommand::CreateAutonomousCommand()
{
	// Get the maximum velocity and acceleration from the dashboard
	double max_velocity = frc::SmartDashboard::GetNumber("AutoMaxV", 0.5);
	if (max_velocity < 0.1) max_velocity = 0.1;
	if (max_velocity > 3.0) max_velocity = 3.0;
	double max_acceleration = frc::SmartDashboard::GetNumber("AutoMaxA", 0.25);
	if (max_acceleration < 0.1) max_acceleration = 0.1;
	if (max_acceleration > 3.0) max_acceleration = 3.0;

	// Get the strategy from the dashboard and create the appropriate path
	Strategy strategy = ms_strategy_chooser.GetSelected();
	RobotPath* robot_path = NULL;
    switch (strategy) {
        default:
        case Strategy::kAutoNavSlalom:
            robot_path = ChallengePaths::CreateSlalomPath(max_velocity, max_acceleration);
            break;
        case Strategy::kAutoNavBarrelRace:
            robot_path = ChallengePaths::CreateBarrelRacingPathRightAngles(max_velocity, max_acceleration);
            break;
        case Strategy::kAutoNavBounce:
            robot_path = ChallengePaths::CreateBouncePath(max_velocity, max_acceleration);
            break;
        case Strategy::kGalacticSearchARed:
            robot_path = ChallengePaths::CreateGalaticSearchPathARed(max_velocity, max_acceleration);
            break;
        case Strategy::kGalacticSearchABlue:
            robot_path = ChallengePaths::CreateGalaticSearchPathABlue(max_velocity, max_acceleration);
            break;
        case Strategy::kGalacticSearchBRed:
            robot_path = ChallengePaths::CreateGalaticSearchPathBRed(max_velocity, max_acceleration);
            break;
        case Strategy::kGalacticSearchBBlue:
            robot_path = ChallengePaths::CreateGalaticSearchPathBBlue(max_velocity, max_acceleration);
            break;

    }

    // Create the path follower and drive command from the path
    PathFollower* path_follower = DrivePathFollower::CreatePurePursuitFollower(robot_path, max_velocity, max_acceleration);
	return new DrivePathFollower(path_follower);
}


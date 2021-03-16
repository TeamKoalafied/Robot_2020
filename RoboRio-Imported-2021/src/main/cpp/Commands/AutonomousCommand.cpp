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
#include <iostream>



//==============================================================================
// Static Data Members

frc::SendableChooser<AutonomousCommand::Strategy> AutonomousCommand::ms_strategy_chooser;

//==========================================================================
// Default dashboard data

// Build a single structure to keep all the mappings in one place to reduce risk of errors
struct AutoModeMapping {
	std::string label;
	AutonomousCommand::Strategy strategy;
};

// Define the mapping between the labels on the dashboard and the position and strategy
const AutoModeMapping kAutoModes[] = {
		{ "AutoNav Slalom",         AutonomousCommand::Strategy::kAutoNavSlalom },
		{ "AutoNav Barrel Race",    AutonomousCommand::Strategy::kAutoNavBarrelRace },
		{ "AutoNav Bounce",         AutonomousCommand::Strategy::kAutoNavBounce },
		{ "Galactic Search A Red",  AutonomousCommand::Strategy::kGalacticSearchARed },
		{ "Galactic Search A Blue", AutonomousCommand::Strategy::kGalacticSearchABlue },
		{ "Galactic Search B Red",  AutonomousCommand::Strategy::kGalacticSearchBRed },
		{ "Galactic Search B Blue", AutonomousCommand::Strategy::kGalacticSearchBBlue },
};

// Compute the number of entries so we can create the string array for populating the default dashboard
static const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);


//==============================================================================
// Construction

AutonomousCommand::AutonomousCommand() :
    frc::CommandGroup("AutonomousCommand") {

	// Initialise to known values. These are all overwritten in SetupCommand(), but
	// initialising in the constructor is good practice and gets rid of warnings.
	// m_strategy = Strategy::kDriveForward;
	// m_position = Position::kLeft;
	// m_position_inch = 0.0;
	// m_game_data = "";
	// m_our_switch_side = Side::kLeft;
	// m_our_scale_side = Side::kLeft;
}


//==========================================================================
// Command Setup

bool AutonomousCommand::SetupCommand() {
	// // Determine the strategy, robot position and the switch and scale sides
	// int i;
	// std::string result;

	// // Get the strategy and the robot position from the dashboard.
	// m_strategy = ms_stratgey_chooser.GetSelected();

	// // Attempt to get the autonomous strategy from the default dashboard . If nothing is
	// // selected, then the returned string will not match and nothing will change.

	// // Update network tables (not sure if this is needed, but it looked relevant)
	// frc::SmartDashboard::UpdateValues();
	// // Read the Auto Selector from the default dashboard
	// result = frc::SmartDashboard::GetString("Auto Selector", "");
	// // Search to see if one of the strings was selected
	// for (i = 0; i < kAutoModeCount; i++) {
	// 	if (kAutoModes[i].label == result) {
	// 		m_position = kAutoModes[i].position;
	// 		m_strategy = kAutoModes[i].strategy;
	// 		std::cout << "Default dashboard mode \"" << kAutoModes[i].label << "\" selected\n";
	// 	}
	// }

	// switch(m_position) {
	// case Position::kLeft:   m_position_inch = kSideWallToAllianceInch + kRobotWidthInch/2; break;
	// case Position::kMiddle: m_position_inch = kFieldWidthInch/2 - 12 + kRobotWidthInch/2; break;
	// case Position::kRight:  m_position_inch = kFieldWidthInch - (kSideWallToAllianceInch + kRobotWidthInch/2); break;
	// }

	// // Get the game (switch/scale colours) from the driver station.
	// m_game_data = frc::DriverStation::GetInstance().GetGameSpecificMessage();

	// // If the game data is not ready return false
	// if (m_game_data.size() < 2) {
	// 	std::cout << "AutonomousCommand::SetupCommand() failed. Game data was not ready.\n";
	// 	return false;
	// }

	// // Log information about it input state that will determine the autonomous command
	// std::cout << "==================================================================\n";
	// std::cout << "AutonomousCommand Initialising\n";
	// std::cout << "Strategy: ";
	// switch (m_strategy) {
	// 	case Strategy::kDriveForward: std::cout << "Drive Forward\n"; break;
	// 	case Strategy::kLoadSwitch:   std::cout << "Load Switch\n";   break;
	// 	case Strategy::kPathfinderSwitch:   std::cout << "Load Switch Pathfinder\n";   break;
	// 	case Strategy::kLoadScale: std::cout << "Load Scale\n"; break;
	// 	case Strategy::kPathSwitch: std::cout << "Load Switch Simple Path\n"; break;
	// 	case Strategy::kPathScale: std::cout << "Load Scale Simple Path\n"; break;
	// 	case Strategy::kTestDriveAndReturn: std::cout << "TEST - Drive and Return\n"; break;
	// }

	// std::cout << "Position: " << m_position_inch << "inches from the left of the alliance wall\n";
	// std::cout << "Game Data: " << m_game_data << "\n";

	// // Parse the game data and determine the which side of the switch and scale belong to us
	// if (m_game_data.size() >= 2) {
	// 	m_our_switch_side = m_game_data[0] == 'L' ? Side::kLeft : Side::kRight;
	// 	m_our_scale_side = m_game_data[1] == 'L' ? Side::kLeft : Side::kRight;
	// } else {
	// 	std::cout << "ERROR: Game data not set. Assuming LLL.\n";
	// 	m_our_switch_side = Side::kLeft;
	// 	m_our_scale_side = Side::kLeft;
	// }


	// frc::DriverStation::Alliance alliance = frc::DriverStation::GetInstance().GetAlliance();
	// std::cout << "Alliance: ";
	// switch (alliance) {
	// 	case frc::DriverStation::kRed:     std::cout << "Red\n"; break;
	// 	case frc::DriverStation::kBlue:    std::cout << "Blue\n"; break;
	// 	case frc::DriverStation::kInvalid: std::cout << "Invalid\n"; break;
	// }

	// // Reset the Pigeon heading so that we can make turns from a starting angle of 0
	// DriveBase::GetInstance().ClearState();

	// // Initialise this command based on the strategy
	// switch (m_strategy) {
	// 	case Strategy::kDriveForward: InitialiseDriveForward(); break;
	// 	case Strategy::kLoadSwitch:   InitialiseLoadSwitch();   break;
	// 	case Strategy::kPathfinderSwitch:   break; // InitialiseLoadSwitchPathfinder();   
	// 	case Strategy::kLoadScale: InitialiseLoadScale(); break;
	// 	case Strategy::kTestDriveAndReturn: InitialiseTestDriveAndReturn(); break;
	// 	case Strategy::kPathSwitch: InitialiseLoadSwitchPath(); break;
	// 	case Strategy::kPathScale: InitialiseLoadScalePath(); break;
	// }

	// Setup was successful
	return true;
}


//==========================================================================
// Dashboard Setup

void AutonomousCommand::SetupDashboard() { // static
	// Setup the chooser for determining the strategy for the autonomous period
   	for (int i = 0; i < kAutoModeCount; i++) {
    	ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
	ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
	frc::SmartDashboard::PutData("Autonomous Strategy", &ms_strategy_chooser);
}

void AutonomousCommand::EchoSettingsToDashboard() { // static
	// std::string message = "";
	// std::string result;

	// switch (m_stratgey_chooser.GetSelected()) {
	// 	case Strategy::kDriveForward:       message += "Drive Forward"; break;
	// 	case Strategy::kLoadSwitch:   		message += "Load Switch";   break;
	// 	case Strategy::kPathfinderSwitch:	message += "Load Switch Pathfinder";   break;
	// 	case Strategy::kLoadScale:          message += "Load Scale"; break;
	// 	case Strategy::kTestDriveAndReturn: message += "TEST - Drive and Return"; break;
	// 	case Strategy::kPathSwitch:         message += "Load Switch Simple Path";   break;
	// 	case Strategy::kPathScale:          message += "Load Scale Simple Path";   break;
	// }


	// switch (m_position_chooser.GetSelected()) {
	// 	case Position::kLeft:   message += " - Left"; break;
	// 	case Position::kMiddle: message += " - Middle"; break;
	// 	case Position::kRight:  message += " - Right";break;
	// }

	// result = frc::SmartDashboard::GetString("Auto Selector", "");
	// if ("Select Autonomous ..." != result)
	// 	message += " - Mode overridden to: \"" + result + "\"";

	// frc::SmartDashboard::PutString("Auto Settings", message);

	// // Echo the default dashboard selection back to the default dashboard as a sanity check
	// frc::SmartDashboard::PutString("DB/String 0", result);
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

	// If the POV is pressed start one of the test paths
	RobotPath* robot_path = NULL;
	// Get the strategy and the robot position from the dashboard.
	Strategy strategy = ms_strategy_chooser.GetSelected();

    switch (strategy) {
        default:
        case Strategy::kAutoNavSlalom:
            robot_path = ChallengePaths::CreateSlalomPath(max_velocity, max_acceleration);

    }

    PathFollower* path_follower = CreatePathPointsFollower(robot_path);
    // PathFollower* path_follower = CreatePathfinderFollower(robot_path);

	return new DrivePathFollower(path_follower);
}

PathFollower* AutonomousCommand::CreatePathPointsFollower(RobotPath* robot_path) {
	double p_gain = frc::SmartDashboard::GetNumber("AutoP", 1.0);
	if (p_gain < 0.5) p_gain = 0.5;
	if (p_gain > 3.0) p_gain = 3.0;
	double i_gain = frc::SmartDashboard::GetNumber("AutoI", 0.0);
	if (i_gain < 0.0) i_gain = 0.0;
	if (i_gain > 3.0) i_gain = 3.0;
	double d_gain = frc::SmartDashboard::GetNumber("AutoD", 0.0);
	if (d_gain < 0.0) d_gain = 0.0;
	if (d_gain > 3.0) d_gain = 3.0;


	DriveBase& drive_base = DriveBase::GetInstance();
	PathPointsFollower* path_follower = new PathPointsFollower(robot_path, &drive_base, new MechanismController2020, true);
	path_follower->GetFollowerParameters().m_kp = p_gain;
	path_follower->GetFollowerParameters().m_ki = i_gain;
	path_follower->GetFollowerParameters().m_kd = d_gain;
	path_follower->GetFollowerParameters().m_kv = 1.0/4.28;
	path_follower->GetFollowerParameters().m_kv_offset = 0.104; // 0.05719;
	path_follower->GetFollowerParameters().m_ka = 1.0/18.29;
	path_follower->GetFollowerParameters().m_wheelbase_width_m = 0.64;
	path_follower->GetFollowerParameters().m_period_s = 0.02;
	return path_follower;
}


//==========================================================================
// Strategy Initialisation

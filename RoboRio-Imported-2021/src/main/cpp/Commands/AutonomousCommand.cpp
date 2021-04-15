//==============================================================================
// AutonomousCommand.cpp
//==============================================================================

#include "AutonomousCommand.h"

#include "ChallengePaths.h"

#include "DrivePathFollower.h"
//#include "MechanismController2020.h"
//#include "PathFollower/PathPointsFollower.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

//#include "PathFollower/PathfinderFollower.h"
//#include "PathFollower/PathPointsFollower.h"

//#include "../Subsystems/DriveBase.h"


#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <iostream>
#include <sstream>

//==============================================================================
// This is an anonymous namespace, which means that all the stuff in it is only
// available in this file
namespace {
// The strategy for autonomous mode
enum class Strategy {
    kShoot,			        // Shoot our initial 3 balls and do nothing else
    kShootAndMoveBackward,  // Shoot our initial 3 balls and move backward out of the way (towards target)
    kShootAndTrench,        // Shoot our initial 3 balls then get 3 balls from the trench and shoot them
    kShootAndShield,        // Shoot our initial 3 balls then get 3 balls from under the shield generator and shoot them
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
        { "Shoot",          Strategy::kShoot },
        { "Shoot Backward", Strategy::kShootAndMoveBackward },
        { "Shoot Trench",   Strategy::kShootAndTrench },
        { "Shoot Shield",   Strategy::kShootAndShield },
};

// Compute the number of entries so we can create the string array for populating the default dashboard
const int kAutoModeCount = sizeof(kAutoModes) / sizeof(AutoModeMapping);

}


//==========================================================================
// Dashboard Setup

void AutonomousCommand::SetupAutonomousDashboard() {
    // Setup the chooser for determining the strategy for the autonomous period
    for (int i = 0; i < kAutoModeCount; i++) {
        ms_strategy_chooser.AddOption(kAutoModes[i].label, kAutoModes[i].strategy);
    } 
    ms_strategy_chooser.SetDefaultOption(kAutoModes[0].label, kAutoModes[0].strategy);
    frc::SmartDashboard::PutData("Autonomous Strategy", &ms_strategy_chooser);

    // Setup the inputs for initial delay and offset. Both default to 0.
    frc::SmartDashboard::PutNumber("Autonomous Delay Sec", 0.0);
    frc::SmartDashboard::PutNumber("Autonomous Trench Offset Inch", 0.0);
}

void AutonomousCommand::UpdateDashboard() {
    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);
    double trench_offset_inch = frc::SmartDashboard::GetNumber("Autonomous Trench Offset Inch", 0.0);

    // Format a description of the settings back to the dashboard. This allow us to check that
    // the dashboard is working and has go the right settings
    std::string strategy_label = "<Error";
    for (int i = 0; i < kAutoModeCount; i++) {
        if (kAutoModes[i].strategy == strategy) {
            strategy_label = kAutoModes[i].label;
        }
    }
	std::ostringstream strategy_description;
  	strategy_description << strategy_label << " Delay: " << delay_s << " Offset: " << trench_offset_inch;
    frc::SmartDashboard::PutString("Autonomous Setting", strategy_description.str());
}


frc::Command* AutonomousCommand::CreateAutonomousCommand() {

    // kShoot,			        // Shoot our initial 3 balls and do nothing else
    // kShootAndMoveBackward,  // Shoot our initial 3 balls and move backward out of the way (towards target)
    // kShootAndTrench,        // Shoot our initial 3 balls then get 3 balls from the trench and shoot them
    // kShootAndShield,        // Shoot our initial 3 balls then get 3 balls from under the shield generator and shoot them

    // Get the strategy, delay and offset from the dashboard
    Strategy strategy = ms_strategy_chooser.GetSelected();
    double delay_s = frc::SmartDashboard::GetNumber("Autonomous Delay Sec", 0.0);
    double trench_offset_inch = frc::SmartDashboard::GetNumber("Autonomous Trench Offset Inch", 0.0);

    // TODO Subtract the shooter wheel ramp up time from the delay

    // Create the appropriate path for the strategy
    RobotPath* robot_path = NULL;
    switch (strategy) {
        default:
        case Strategy::kShoot:
            robot_path = CreateShootPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndMoveBackward:
            robot_path = CreateShootAndMoveBackwardPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndTrench:
            robot_path = CreateShootAndTrenchPath(delay_s, trench_offset_inch);
            break;
        case Strategy::kShootAndShield:
            robot_path = CreateShootAndShieldPath(delay_s, trench_offset_inch);
            break;
    }

    // Get the maximum velocity and acceleration from the dashboard
    double max_velocity = frc::SmartDashboard::GetNumber("AutoMaxV", 0.5);
    if (max_velocity < 0.1) max_velocity = 0.1;
    if (max_velocity > 3.0) max_velocity = 3.0;
    double max_acceleration = frc::SmartDashboard::GetNumber("AutoMaxA", 0.25);
    if (max_acceleration < 0.1) max_acceleration = 0.1;
    if (max_acceleration > 3.0) max_acceleration = 3.0;

    // Create the path follower and drive command from the path
    PathFollower* path_follower = DrivePathFollower::CreatePurePursuitFollower(robot_path, max_velocity, max_acceleration);
    return new DrivePathFollower(path_follower);
}

//==========================================================================
// Path Creation

RobotPath* AutonomousCommand::CreateShootPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "Shoot";

    // All the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // Return the robot path
    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndMoveBackwardPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveBackwards";

    // All the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    const double INCH = 0.0254;
    const double FOOT = 12*INCH;

    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(-2*FOOT, 0.0);
    path.m_point3.Set(-6*FOOT, 0.0);
    path.m_point4.Set(-8*FOOT, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndTrenchPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "Shoot";

    // All the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);


    const double INCH = 0.0254;
    const double FOOT = 12*INCH;

    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(-2*FOOT, 0.0);
    path.m_point3.Set(-6*FOOT, 0.0);
    path.m_point4.Set(-8*FOOT, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndShieldPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "Shoot";

    // All the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    const double INCH = 0.0254;
    const double FOOT = 12*INCH;

    Bezier3 path;
    path.m_point1.Set(0.0, 0.0);
    path.m_point2.Set(-2*FOOT, 0.0);
    path.m_point3.Set(-6*FOOT, 0.0);
    path.m_point4.Set(-8*FOOT, 0.0);	
    PathSegment* path_segment = new PathSegment();
    path_segment->m_name = "Straight";
    path_segment->m_path_definition.push_back(path);
    path_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(path_segment);

    return robot_path;
}

//==========================================================================
// Path Creation Helpers

void AutonomousCommand::AddDelaySegment(RobotPath* robot_path, double delay_s) {
    if (delay_s > 0.0) {
        PathSegment* delay_segment = new PathSegment();
        delay_segment->m_name = "Delay";
        delay_segment->m_reverse = false;
    	delay_segment->m_mechanism_actions.push_back(MechanismAction("None", delay_s));
        robot_path->m_path_segments.push_back(delay_segment);
    }
}

void AutonomousCommand::AddShootSegment(RobotPath* robot_path) {
    PathSegment* delay_segment = new PathSegment();
    delay_segment->m_name = "Shoot";
    delay_segment->m_reverse = false;
    delay_segment->m_mechanism_actions.push_back(MechanismAction("Shoot", 0.0));
    robot_path->m_path_segments.push_back(delay_segment);
}

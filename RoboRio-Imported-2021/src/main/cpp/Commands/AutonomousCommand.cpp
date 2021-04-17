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

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // Return the robot path
    return robot_path;
}

RobotPath* AutonomousCommand::CreateShootAndMoveBackwardPath(double delay_s, double trench_offset_inch) {
    // Create and name the robot path
    RobotPath* robot_path = new RobotPath();
    robot_path->m_name = "ShootAndMoveBackwards";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // Add a segment to move the robot backwards 8 feet. This is towards the target, which
    // is 10 feet from the start line.
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
    robot_path->m_name = "ShootAndTrench";

    // Add the initial delay, if any, and shooting of the initial 3 balls 
    AddDelaySegment(robot_path, delay_s);
    AddShootSegment(robot_path);

    // See Layout and Markings Diagram page 5 https://firstfrc.blob.core.windows.net/frc2021/PlayingField/2021LayoutMarkingDiagram.pdf

    //                  Trench Balls
    //     |             o    o    o     66.91" - Distance sideways from robot at centre of target   
    //     |             122.62"   194.62"      - Distance from initiation line
    //   > * Robot
    //     | Initiation Line
    //
    // Robot starts on the initiation line with its back bumper just over the initiation line (most
    // of the robot further way from the target).

    const double INCH = 0.0254;
	const double ROBOT_LENGTH = 37 * INCH;
    const double SLALOM_FACTOR = 0.7;

    const double BALL1_DISTANCE = 122.62 * INCH;
    const double BALL3_DISTANCE = 194.62 * INCH;
    const double TRENCH_DISTANCE = 66.91 * INCH;

    // Slalon left to the first ball, then forwards to pick up the three balls
    PathSegment* pickup_segment = new PathSegment();
    pickup_segment->m_name = "Straight";
    pickup_segment->m_reverse = false;
    robot_path->m_path_segments.push_back(pickup_segment);
    ChallengePaths::AddSlalomLeft(pickup_segment, BALL1_DISTANCE - ROBOT_LENGTH, TRENCH_DISTANCE, SLALOM_FACTOR,
                                  Point2D(0, 0), Point2D(1, 0));
    ChallengePaths::AddStraight(pickup_segment, BALL3_DISTANCE - BALL1_DISTANCE);
    pickup_segment->m_mechanism_actions.push_back(MechanismAction("StartIntaking", 0.0));

    PathSegment* return_segment = new PathSegment();
    return_segment->m_name = "Straight";
    return_segment->m_reverse = true;
    robot_path->m_path_segments.push_back(return_segment);

    // Drive back to the position of the first ball to be closer to the target
    Point2D start;
    Point2D direction;
    ChallengePaths::GetCurrent(pickup_segment, start, direction);
    direction = -direction;
    ChallengePaths::AddStraight(return_segment, BALL3_DISTANCE - BALL1_DISTANCE, start, direction);
    return_segment->m_mechanism_actions.push_back(MechanismAction("StopIntaking", 0.0));

    // Rotate to face the target and shoot the balls
    AddRotateToTargetSegment(robot_path);
    AddShootSegment(robot_path);

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

void AutonomousCommand::AddRotateToTargetSegment(RobotPath* robot_path) {
    PathSegment* find_segment = new PathSegment();
    find_segment->m_name = "RotateToTarget";
    find_segment->m_reverse = false;
    find_segment->m_drivebase_action = "RotateToTarget";
    robot_path->m_path_segments.push_back(find_segment);
}

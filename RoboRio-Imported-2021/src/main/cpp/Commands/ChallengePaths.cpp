#include "ChallengePaths.h"
#include "../RobotConfiguration.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

#include "PathFollower/PathfinderFollower.h"
#include "PathFollower/PathPointsFollower.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <sstream>

namespace RC = RobotConfiguration;


//==========================================================================
// Static Joystick Testing Control Functions

RobotPath* ChallengePaths::CreateTestPath(int pov_angle, double max_velocity, double max_acceleration)
{
	// If the POV is pressed start one of the test paths
	RobotPath* robot_path = NULL;
	switch (pov_angle) {
		case RC::kJoystickPovUp: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::Straight\n";
			robot_path = CreateStraightPath(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovLeft: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::Slalom\n";
			robot_path = CreateSlalomPath(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovDown: {
			std::cout << "Starting DrivePathFollower - ChallengePaths::SlalomRA\n";
			robot_path = CreateSlalomPathRightAngles(max_velocity, max_acceleration);
			break;
		}
		case RC::kJoystickPovRight: {
			break;
		}

	}

	// Append the max velocity and acceleration to the name so it gets logged
	// std::ostringstream full_name;
  	// full_name << robot_path->m_name << " MaxV: " << max_velocity << " MaxA: " << max_acceleration;
  	// robot_path->m_name = full_name.str();

    return robot_path;
}


//==============================================================================
// Path Creation

RobotPath* ChallengePaths::CreateStraightPath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Straight2m";

	// Straight 2m forward
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	path.m_point2.Set(0.5, 0.0);
	path.m_point3.Set(1.5, 0.0);
	path.m_point4.Set(2.0, 0.0);			
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Straight";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	path_segment->m_reverse = false;
//	path_segment->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* ChallengePaths::CreateSlalomPath(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Slalom";

	const double length = 60 * 0.0254;
	const double width = 70 * 0.0254;
	const double d_fraction = 0.25;
	const double d = d_fraction * length;

	// Straight 2m forward
	Bezier3 path1;
	path1.m_point1.Set(0.0, 0.0);
	path1.m_point2.Set(d*2, 0.0);
	path1.m_point3.Set(length - d, -width);
	path1.m_point4.Set(length,     -width);			
	Bezier3 path2;
	path2.m_point1.Set(length,       -width);
	path2.m_point2.Set(length + d,   -width);
	path2.m_point3.Set(3*length - d, -width);
	path2.m_point4.Set(3*length,     -width);
	Bezier3 path3;
	path3.m_point1.Set(3*length, 		 -width);
	path3.m_point2.Set((3*length)+d, -width);
	path3.m_point3.Set((4*length)-d, 		0.0);
	path3.m_point4.Set(4*length, 	 		0.0);

	Bezier3 path4;
	path4.m_point1.Set(4*length, 		 -0.0);
	path4.m_point2.Set((4*length)+d,    0.0);
	path4.m_point3.Set(5*length,	-width/2 + d);
	path4.m_point4.Set(5*length, 	-width/2);


	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Slalom";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path1);
	path_segment->m_path_definition.push_back(path2);
	path_segment->m_path_definition.push_back(path3);
	path_segment->m_path_definition.push_back(path4);
	path_segment->m_reverse = false;
//	path_segment->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* ChallengePaths::CreateSlalomPathRightAngles(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "SlalomRA";

	const double length = 60 * 0.0254;
	const double width = 70 * 0.0254;
	const double radius = 25 * 0.0254;

	// Create a path where the robot does the slalom in straight sections and right angle turns.
	// This might not work very well but is a useful experiment.
 	
// 	double start_length = length /2 - radius;
// 	Bezier3 path1; // Initial straight section
// 	path1.m_point1.Set(0.0, 0.0);
// 	path1.m_point2.Set(start_length * 0.25, 0.0);
// 	path1.m_point3.Set(start_length * 0.75, 0.0);
// 	path1.m_point4.Set(start_length, 0.0);			
// 	Bezier3 path2; // Right angle turn right
// 	path2.m_point1.Set(start_length, 0.0);
// 	path2.m_point2.Set(length/2, 0.0);
// 	path2.m_point3.Set(length/2, 0.0);
// 	path2.m_point4.Set(length/2,     -radius);
// 	Bezier3 path3; // Straight through the first gap
// 	path3.m_point1.Set(length/2,     -radius);
// 	path3.m_point2.Set(length/2, -width/2);
// 	path3.m_point3.Set(length/2, -width/2);
// 	path3.m_point4.Set(length/2,     -width + radius);
// 	Bezier3 path4; // Right angle turn right
// 	path4.m_point1.Set(length/2,     -width + radius);
// 	path4.m_point2.Set(length/2,     -width);
// 	path4.m_point3.Set(length/2,     -width);
// 	path4.m_point4.Set(length/2 + radius, -width);


PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Slalom";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
// 	path_segment->m_path_definition.push_back(path1);
// 	path_segment->m_path_definition.push_back(path2);
// 	path_segment->m_path_definition.push_back(path3);
// 	path_segment->m_path_definition.push_back(path4);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);

// Alternative method for creating the path
double startLength = length/2 - radius;
AddStraight(path_segment, startLength + length/2 - radius, Point2D(0, 0), Point2D(1.0, 0.0));
AddTurnRight(path_segment, radius);
AddStraight(path_segment, width - 2*radius - 0.3);
AddTurnLeft(path_segment, radius);
AddStraight(path_segment, 2*length - 0.3);
AddTurnLeft(path_segment, radius);
AddStraight(path_segment, width - 2*radius);
AddTurnRight(path_segment, radius);
AddStraight(path_segment, length/2 - 1.05*radius);
AddTurnRight(path_segment, radius);
AddStraight(path_segment, width - 2*radius - 0.3);
AddTurnRight(path_segment, radius);
AddTurnRight(path_segment, radius);
AddStraight(path_segment, width - 2*radius);
AddTurnLeft(path_segment, radius);
AddStraight(path_segment, 2*length - 0.3);
AddTurnLeft(path_segment, radius);
AddStraight(path_segment, width - 2*radius);
AddTurnRight(path_segment, radius);
AddStraight(path_segment, 0.1);

	return robot_path;
}

RobotPath* ChallengePaths::CreateGalaticSearchPath(double max_velocity, double max_acceleration)
{
	static MechanismAction START_INTAKE[] = {
			{ "StartIntaking",     MechanismAction::TimeSpecification::Start, 0.5, 0 }
	};

	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "GalaticSearch1";
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "GalaticSearch1";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_reverse = false;
	robot_path->m_path_segments.push_back(path_segment);


	const double INCH = 0.0254;
	const double radius = 30 * INCH;
	const double robot_length = 37 * INCH;


	AddStraight(path_segment, (90 - 30)*INCH + robot_length/2, Point2D(0, 0), Point2D(1.0, 0.0));
	AddSlalomRight(path_segment, (150 - 90)*INCH, 30 * INCH, 0.5);
	AddTurnLeft(path_segment, radius);
	AddStraight(path_segment, 60 * INCH);
	AddTurnRight(path_segment, radius);
	AddStraight(path_segment, (360 - 180) * INCH - radius);

	return robot_path;
}


//==============================================================================
// Path Building

void ChallengePaths::AddStraight(PathSegment* path_segment, double length)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddStraight(path_segment, length, start, direction);
}

void ChallengePaths::AddTurnLeft(PathSegment* path_segment, double radius)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddTurnLeft(path_segment, radius, start, direction);
}

void ChallengePaths::AddTurnRight(PathSegment* path_segment, double radius)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);
	AddTurnRight(path_segment, radius, start, direction);
}

void ChallengePaths::AddSlalomLeft(PathSegment* path_segment, double length, double width, double fraction)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);

	Point2D unit_direction = direction;
	unit_direction.Normalize();

    Point2D fraction_vector = fraction * unit_direction;
    Point2D end = start + unit_direction * length + TurnLeft(unit_direction) * width;

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + fraction_vector;
	path.m_point3 = end - fraction_vector;
	path.m_point4 = end;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddSlalomRight(PathSegment* path_segment, double length, double width, double fraction)
{
	Point2D start;
	Point2D direction;
	GetCurrent(path_segment, start, direction);

	Point2D unit_direction = direction;
	unit_direction.Normalize();

    Point2D fraction_vector = fraction * unit_direction;
    Point2D end = start + unit_direction * length + TurnRight(unit_direction) * width;

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + fraction_vector;
	path.m_point3 = end - fraction_vector;
	path.m_point4 = end;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::GetCurrent(PathSegment* path_segment, Point2D& position, Point2D& direction)
{
	const Bezier3& path = path_segment->m_path_definition.back();
	position = path.m_point4;
	direction = path.m_point4 - path.m_point3; 
}

void ChallengePaths::AddStraight(PathSegment* path_segment, double length, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * (0.25 * length);
	path.m_point3 = start + unit_direction * (0.75 * length);
	path.m_point4 = start + unit_direction * length;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddTurnLeft(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * radius;
	path.m_point3 = start + unit_direction * radius;
	path.m_point4 = start + unit_direction * radius + TurnLeft(unit_direction) * radius;

	path_segment->m_path_definition.push_back(path);
}

void ChallengePaths::AddTurnRight(PathSegment* path_segment, double radius, const Point2D& start, const Point2D& direction)
{
	Point2D unit_direction = direction;
	unit_direction.Normalize();

	Bezier3 path;
	path.m_point1 = start;
	path.m_point2 = start + unit_direction * radius;
	path.m_point3 = start + unit_direction * radius;
	path.m_point4 = start + unit_direction * radius + TurnRight(unit_direction) * radius;

	path_segment->m_path_definition.push_back(path);
}


//==============================================================================
// Utility Helpers

Point2D ChallengePaths::TurnLeft(const Point2D& direction)
{
	return Point2D(-direction.y, direction.x);
}

Point2D ChallengePaths::TurnRight(const Point2D& direction)
{
	return Point2D(direction.y, -direction.x);
}

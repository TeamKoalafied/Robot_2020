//==============================================================================
// TestPaths.cpp
//==============================================================================

#include "TestPaths.h"

#include "ChallengePaths.h"

#include "RobotPath/Bezier3.h"
#include "RobotPath/MotionProfile.h"
#include "RobotPath/Point2D.h"
#include "RobotPath/PathSegment.h"
#include "RobotPath/RobotPath.h"

#include "../RobotConfiguration.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>

namespace RC = RobotConfiguration;

//==========================================================================
// Static Joystick Testing Control Functions

RobotPath* TestPaths::CreateTestPath(int pov_angle, double max_velocity, double max_acceleration)
{
	switch (pov_angle) {
		case RC::kJoystickPovUp: {
			std::cout << "Starting TestPaths - Straight\n";
			return CreateStraightPath(max_velocity, max_acceleration, 1.0);
		}
		case RC::kJoystickPovLeft: {
			std::cout << "Starting TestPaths - Cube1\n";
			return CreateCube1Path(max_velocity, max_acceleration);
		}
		case RC::kJoystickPovDown: {
			std::cout << "Starting TestPaths - Cube2\n";
			return CreateCube2Path(max_velocity, max_acceleration);
		}
		case RC::kJoystickPovRight: {
			std::cout << "Starting TestPaths - From Dashboard\n";
			return CreateVisionPathFromDashBoard(max_velocity, max_acceleration);
		}
	}
    return NULL;
}


//==========================================================================
// Static Joystick Testing Implementation Functions

RobotPath* TestPaths::CreateVisionPathFromDashBoard(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();

	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	double targetOffsetAngle_Horizontal = table->GetNumber("tx",0.0);
	double targetOffsetAngle_Vertical = table->GetNumber("ty",0.0);
	// double targetArea = table->GetNumber("ta",0.0);
	//double targetSkew = table->GetNumber("ts",0.0);

	// Get the parameters from the SmartDashboard
    double vision_x = 0;
    double vision_y = 0;
    double vision_heading_degrees = 0.0;

	double kCameraHeight = 0.63; //0.26 when lowered
	double kTargetHeight = 2.17;
	double kCameraAngle = 24.55;
	double kWantedDistance = 2;
	
	// calculate distance
	double distanceFromTarget = (kTargetHeight - kCameraHeight) / 
								(tan((kCameraAngle + targetOffsetAngle_Vertical)*(M_PI/180)));
	vision_x = (distanceFromTarget) - kWantedDistance;
	vision_y = (distanceFromTarget * (tan(targetOffsetAngle_Horizontal * (M_PI/180)))) * -1;
	std::cout << "Vertical offset: " << targetOffsetAngle_Vertical << std::endl;
	std::cout << "Horizontal offset: " << targetOffsetAngle_Horizontal <<std::endl << std::endl;
	std::cout << "Distance: " << distanceFromTarget << std::endl << std::endl;
	std::cout << "Vision X: " << vision_x << std::endl;
	std::cout << "VIsion Y: " << vision_y << std::endl;
	// vision_heading_degrees = targetOffsetAngle_Horizontal;
	// vision_x=0;
	// vision_y=0;
	// Limit the values to a sensible range (could change, but prevents nasty behaviour while testing)
	if (vision_x < 0.0) vision_x = 0.0;
	// if (vision_x > 5.0) vision_x = 5.0;
	// if (vision_y < -3.0) vision_y = -3.0;
	
	// if (vision_y > 3.0)  vision_y =  3.0;

	// Start the path at the origin. This is arbitrary, so the origin is just simplest.
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	// Set the end of the path and calculate the straight line distance
	path.m_point4.Set(vision_x, vision_y);

	double distance = path.m_point4.Length();

	// The initial heading is 0 degrees so the second Bezier point is directly on the x axis.
	// We chose the distance to the second Bezier point be half the distance. This is can be changed
	// but seems to produce reasonable results
	path.m_point2.Set(distance / 2.0, 0.0);
	// The third Bezier point is calculated using the desired final heading.
	path.m_point3 = path.m_point4 - (distance / 2.0)*Point2D::UnitVectorDegrees(vision_heading_degrees);
	// Setup a path segment using the calculated Bezier
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Vision";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	robot_path->m_path_segments.push_back(path_segment);
	return robot_path;
}

static MechanismAction GRAB_CUBE[] = {
		{ "DropArm",    MechanismAction::TimeSpecification::End, -2.5, 0 },
		{ "OpenClaw",   MechanismAction::TimeSpecification::End, -2.0, 0 },
		{ "RollerGrab", MechanismAction::TimeSpecification::End, -1.0, 0 },
		{ "CloseClaw",  MechanismAction::TimeSpecification::End, -0.5, 0 },
		{ "LiftArm",    MechanismAction::TimeSpecification::End,  0.0, 0 },
};
static MechanismAction PLACE_CUBE[] = {
		{ "DropArm",     MechanismAction::TimeSpecification::End, -1.5, 0 },
		{ "OpenClaw",    MechanismAction::TimeSpecification::End,  0.0, 0 },
		{ "RollerPlace", MechanismAction::TimeSpecification::End,  0.0, 0 },
};

static MechanismAction RESET[] = {
		{ "CloseClaw",   MechanismAction::TimeSpecification::Start, 1.0, 0 },
		{ "LiftArm",     MechanismAction::TimeSpecification::Start, 1.0, 0 },
};	

RobotPath* TestPaths::CreateStraightPath(double max_velocity, double max_acceleration, double distance) {
	RobotPath* robot_path = new RobotPath();

	robot_path->m_name = "Straight2m";

	// Straight forward
	Bezier3 path;
	path.m_point1.Set(0.0, 0.0);
	path.m_point2.Set(distance*0.25, 0.0);
	path.m_point3.Set(distance*0.75, 0.0);
	path.m_point4.Set(distance, 0.0);			
	PathSegment* path_segment = new PathSegment();
	path_segment->m_name = "Straight";
	path_segment->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment->m_path_definition.push_back(path);
	path_segment->m_reverse = false;
	//path_segment->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment);

	return robot_path;
}

RobotPath* TestPaths::CreateCube1Path(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();
	robot_path->m_name = "Cube1";

	const double TL = 1.5;
	Bezier3 path1;
	path1.m_point1.Set( 0.0,  0.0);
	path1.m_point2.Set( 0.5,  0.0);
	path1.m_point3.Set( 2.5,  0.0);
	path1.m_point4.Set( 3.0,  0.0);
	Bezier3 path2;
	path2.m_point1.Set( 3.0,  0.0);
	path2.m_point2.Set( 3.0-TL,  0.0);
	path2.m_point3.Set( 1.5,  -1.5+TL);
	path2.m_point4.Set( 1.5, -1.5);			
	Bezier3 path3;
	path3.m_point1.Set( 1.5, -1.5);
	path3.m_point2.Set( 1.5,  -1.5+TL);
	path3.m_point3.Set( TL,  0.0);
	path3.m_point4.Set( 0.0,  0.0);


	PathSegment* path_segment1 = new PathSegment();
	path_segment1->m_name = "GrabCube";
	path_segment1->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment1->m_path_definition.push_back(path1);
	//path_segment1->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment1);

	PathSegment* path_segment2 = new PathSegment();
	path_segment2->m_name = "Reverse";
	path_segment2->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment2->m_reverse = true;
	path_segment2->m_path_definition.push_back(path2);
	robot_path->m_path_segments.push_back(path_segment2);

	PathSegment* path_segment3 = new PathSegment();
	path_segment3->m_name = "PlaceCube";
	path_segment3->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment3->m_path_definition.push_back(path3);
	//path_segment3->m_mechanism_actions.assign(PLACE_CUBE, PLACE_CUBE +  sizeof(PLACE_CUBE)/sizeof(PLACE_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment3);

	return robot_path;
}

RobotPath* TestPaths::CreateCube2Path(double max_velocity, double max_acceleration) {
	RobotPath* robot_path = new RobotPath();
	robot_path->m_name = "Cube2";

	Bezier3 path1;
	path1.m_point1.x =  0.0;
	path1.m_point1.y =  0.0;
	path1.m_point2.x =  1.5;
	path1.m_point2.y =  0.0;
	path1.m_point3.x =  1.5;
	path1.m_point3.y =  0.0;
	path1.m_point4.x =  1.5;
	path1.m_point4.y = -1.5;
	Bezier3 path2;
	path2.m_point1.x =  1.5;
	path2.m_point1.y = -1.5;
	path2.m_point2.x =  1.5;
	path2.m_point2.y =  0.0;
	path2.m_point3.x =  1.5;
	path2.m_point3.y =  1.0;
	path2.m_point4.x =  3.0;
	path2.m_point4.y =  1.0;			
	Bezier3 path3;
	path3.m_point1.x =  3.0;
	path3.m_point1.y =  1.0;
	path3.m_point2.x =  1.5;
	path3.m_point2.y =  1.0;
	path3.m_point3.x =  1.5;
	path3.m_point3.y =  0.0;
	path3.m_point4.x =  1.5;
	path3.m_point4.y = -1.5;			
	Bezier3 path4;
	path4.m_point1.x =  1.5;
	path4.m_point1.y = -1.5;
	path4.m_point2.x =  1.5;
	path4.m_point2.y =  0.0;
	path4.m_point3.x =  1.5;
	path4.m_point3.y =  0.0;
	path4.m_point4.x =  0.0;
	path4.m_point4.y =  0.0;


	PathSegment* path_segment1 = new PathSegment();
	path_segment1->m_name = "Cube2_1";
	path_segment1->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment1->m_reverse = true;
	path_segment1->m_path_definition.push_back(path1);
	path_segment1->m_mechanism_actions.assign(RESET, RESET +  sizeof(RESET)/sizeof(RESET[0]));
	robot_path->m_path_segments.push_back(path_segment1);

	PathSegment* path_segment2 = new PathSegment();
	path_segment2->m_name = "Cube2_2";
	path_segment2->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment2->m_path_definition.push_back(path2);
	path_segment2->m_mechanism_actions.assign(GRAB_CUBE, GRAB_CUBE +  sizeof(GRAB_CUBE)/sizeof(GRAB_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment2);

	PathSegment* path_segment3 = new PathSegment();
	path_segment3->m_name = "Cube2_3";
	path_segment3->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment3->m_reverse = true;
	path_segment3->m_path_definition.push_back(path3);
	robot_path->m_path_segments.push_back(path_segment3);

	PathSegment* path_segment4 = new PathSegment();
	path_segment4->m_name = "Cube2_4";
	path_segment4->m_motion_profile.Setup(max_velocity, max_acceleration);
	path_segment4->m_path_definition.push_back(path4);
	path_segment4->m_mechanism_actions.assign(PLACE_CUBE, PLACE_CUBE +  sizeof(PLACE_CUBE)/sizeof(PLACE_CUBE[0]));
	robot_path->m_path_segments.push_back(path_segment4);

	return robot_path;
}

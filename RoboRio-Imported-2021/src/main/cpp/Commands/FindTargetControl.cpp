//
// FindTargetControl.cpp
//

#include "FindTargetControl.h"


#include "../RobotConfiguration.h"
#include "../Subsystems/DriveBase.h"


#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace RC = RobotConfiguration;


FindTargetControl::FindTargetControl(DriveBase& drive_base) :
    m_drive_base(drive_base) {

}

FindTargetControl::~FindTargetControl() {

}


bool FindTargetControl::DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller) {

	// See Robot.cpp for initial settings. The defaults here are 0.0 if no connection
	float kp = frc::SmartDashboard::GetNumber("VisionKp", 0.0);						// 0.017 or higher
	float minRotation = frc::SmartDashboard::GetNumber("VisionMinRotation", 0.0);	// 0.33 on carpet, 0.25 on wood
	float maxRotation = frc::SmartDashboard::GetNumber("VisionMaxRotation", 0.0);	// 0.7



    bool shoot_button = joystick->GetRawButton(RC::kJoystickYButton);


}


void UpdateTargetHeading() {
	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("pivision");
	float tx = table->GetNumber("tx", 0.0);  // degrees (-27 to 27 for limelight1)
	bool object_found = (0.0f != table->GetNumber("tv", 0.0));  // 0.0 unless the target is detected

	double heading = -drive_base.GetPigeonHeading();  // positive tx is clockwise, so flip sign here
	float offset = 0;

}

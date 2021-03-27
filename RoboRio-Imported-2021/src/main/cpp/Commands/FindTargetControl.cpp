//
// FindTargetControl.cpp
//

#include "FindTargetControl.h"


#include "../HapticController.h"
#include "../RobotConfiguration.h"
#include "../Subsystems/DriveBase.h"


#include <frc/Joystick.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

namespace RC = RobotConfiguration;


FindTargetControl::FindTargetControl(DriveBase& drive_base) :
    m_drive_base(drive_base) {

    m_state = State::kIdle;
    m_target_heading_index = 0;
    for (int i = 0; i < HEADING_HISTORY; i++) m_target_headings[i] = kErrorHeading;
}

FindTargetControl::~FindTargetControl() {

}


bool FindTargetControl::DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller) {

    bool old_target_valid = m_target_valid;
    UpdateTargetHeading();

    if (joystick->GetRawButton(RC::kJoystickYButton)) {
        if (m_target_valid) {
            if (m_state != State::kReachedTarget) {
                if (RotateToTarget()) {
                    m_state = State::kReachedTarget;
                    haptic_controller->DoContinuousFeedback(1.0, 1.0);
                }
                else {
                    m_state != State::kRotatingToTarget;                    
                }
            }
        }
        else {
            if (m_state != State::kSignalNoTarget) {
                static double PATTERN[10] = { 1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0 };
                haptic_controller->DoFeedback(PATTERN, 10);

                m_state = State::kSignalNoTarget;
            }
        }
    }
    else {
        m_state = State::kIdle;
        if (m_target_valid && !old_target_valid) {
            haptic_controller->DoContinuousFeedback(0.5, 1.0);
        }
    }
}

void FindTargetControl::UpdateTargetHeading() {
	std::shared_ptr<NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("pivision");
	float tx = table->GetNumber("tx", 0.0);  // degrees (-27 to 27 for limelight1)
	bool object_found = (0.0f != table->GetNumber("tv", 0.0));  // 0.0 unless the target is detected

	double heading = m_drive_base.GetPigeonHeading();  // positive tx is clockwise, so flip sign here

    // Calculate the current heading of the target
    double target_heading = kErrorHeading;
    if (object_found && heading != DriveBase::kHeadingError) {
        target_heading = heading - tx;
    }

    // Record the current heading in the circular heading buffer
    m_target_headings[m_target_heading_index++] = target_heading;
    if (m_target_heading_index == HEADING_HISTORY) m_target_heading_index = 0;

    // Get the sum and count of all the valid heading in the buffer
    double total_target_heading = 0.0;
    int valid_heading_count = 0;
    for (int i = 0; i < HEADING_HISTORY; i++) {
        double heading = m_target_headings[i];
        if (heading != kErrorHeading) {
            total_target_heading += heading;
            valid_heading_count++;
        }
    }

    // If at least half the headings are valid then regard the heading as valid
    if (valid_heading_count > HEADING_HISTORY/2) {
        m_target_valid = true;
        m_target_heading = total_target_heading / valid_heading_count;
    }
    else {
        m_target_valid = false;
    }
}

bool FindTargetControl::RotateToTarget() {
	// See Robot.cpp for initial settings. The defaults here are 0.0 if no connection
	float kp = frc::SmartDashboard::GetNumber("VisionKp", 0.0);						// 0.017 or higher
	float minRotation = frc::SmartDashboard::GetNumber("VisionMinRotation", 0.0);	// 0.33 on carpet, 0.25 on wood
	float maxRotation = frc::SmartDashboard::GetNumber("VisionMaxRotation", 0.0);	// 0.7

	double current_heading = m_drive_base.GetPigeonHeading();


    double offset = m_target_heading - current_heading;
  
    while (offset < -180.0) offset += 360.0;
    while (offset > 180) offset -= 360.0;

    // Start the rotate, and give it 100ms to settle
    if ((offset < -0.5) || (offset > 0.5)) {
        // Give motors a little power even if error is small
        double rotation;
        if (offset > 0.0) {
            rotation = kp*offset + minRotation;
        } else {
            rotation = kp*offset - minRotation;
        }

        // Clip the maximum rotation for safety!
        if (rotation > maxRotation) rotation = maxRotation;
        if (rotation < -maxRotation) rotation = -maxRotation;

        //std::cout << "Vision: offset " << offset << "rotation " << rotation << std::endl; //debug
        m_drive_base.ArcadeDriveForVision(0.0, rotation);

        return false;
    } else {
        return true;
    }
}

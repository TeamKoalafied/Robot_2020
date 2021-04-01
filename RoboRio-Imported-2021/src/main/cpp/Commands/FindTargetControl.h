//
// FindTargetControl.h
//

#ifndef FindTargetControl_H
#define FindTargetControl_H


#include <frc/Joystick.h>
#include <frc/Timer.h>
class HapticController;
class DriveBase;

//
class FindTargetControl
{
public:
    //==========================================================================
    // Construction and Destruction

    // Constructor
    //
    // drive_base - Drive base to control
    FindTargetControl(DriveBase& drive_base);

    // Destructor
    ~FindTargetControl();

    //==========================================================================
    // Operation

    // Perform finding and rotating to the target under joystick control
    //
    // joystick - Joystick being used for drivebase control
    // haptic_controller - Controller for doing haptic feedback to the driver
    //
    // Returnsw hether normal driving operation should be performed in this period update.
    bool DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller);

private:
    //==========================================================================
    // Private Nested Types

    // State of finding the target
    enum class State {
        kIdle,              // Idle. Driver is not trying to rotate to target
        kSignalNoTarget,    // Driver tried to rotate but there was no target visible
        kRotatingToTarget,  // Currently rotating to the target
        kReachedTarget      // Target had been rotated to
    };


    //==========================================================================
    // Implementation

    // Update the information about the target heading
    void UpdateTargetHeading();

    // Rotate the drivebase towards the target
    //
    // Returns
    bool RotateToTarget();


    //==========================================================================
    // Constants and Member Variables

    static const int HEADING_HISTORY = 50;     // Number heading recorded to get a stable result

    DriveBase& m_drive_base;                    // Drive base being controlled
    State m_state;                              // Current state of finding the target
    //frc::Timer m_timer;
    double m_target_headings[HEADING_HISTORY];  // Circular buffer of target headings 
    int m_target_heading_index;                 // Index to insert
    double m_target_heading;
    bool m_target_valid;
    bool m_signaling_no_target;

    static constexpr double kErrorHeading = 999.0;
};

#endif

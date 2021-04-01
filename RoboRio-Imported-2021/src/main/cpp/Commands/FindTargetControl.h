//
// FindTargetControl.h
//

#ifndef FindTargetControl_H
#define FindTargetControl_H


#include <frc/Joystick.h>
#include <frc/Timer.h>
class HapticController;

class DriveBase;

class FindTargetControl
{
public:
    FindTargetControl(DriveBase& drive_base);
    ~FindTargetControl();


    bool DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller);

private:
    enum class State {
        kIdle,
        kSignalNoTarget,
        kRotatingToTarget,
        kReachedTarget
    };



    void UpdateTargetHeading();

    bool RotateToTarget();

    static const int HEADING_HISTORY = 50;     // Number heading recorded to get a stable result


    DriveBase& m_drive_base;
    State m_state;
    frc::Timer m_timer;
    double m_target_headings[HEADING_HISTORY];
    int m_target_heading_index;
    double m_target_heading;
    bool m_target_valid;
    bool m_signaling_no_target;

    static constexpr double kErrorHeading = 999.0;
};

#endif

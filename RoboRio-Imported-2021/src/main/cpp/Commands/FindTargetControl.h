//
// FindTargetControl.h
//

#include <frc/Joystick.h>
#include <frc/Timer.h>
class HapticController;

class DriveBase;

class FindTargetControl
{
public:
    FindTargetControl(Drivebase& drive_base);
    ~FindTargetControl();


    bool DoFindTargetJoystick(frc::Joystick* joystick, HapticController* haptic_controller);

private:

    void UpdateTargetHeading();

    Drivebase& m_drive_base;
    frc::Timer m_timer;
    double m_target_headings[];
    double m_target_heading;
    bool m_target_valid;
}
/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     30 Apr 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef JOYSTICK_H
#define JOYSTICK_H

////////////////////////////////////////////////////////////////////////////////

#include <plib/js.h>
#include <QString>

////////////////////////////////////////////////////////////////////////////////


class Joystick
{
public:
    Joystick();
    ~Joystick();
    void Init();
    void Read();

    QString getName();
    inline int getNumAxes() { return numAxes; }
    inline int getNumButtons() { return numButtons; }
    double getAileron();
    double getElevator();
    double getRudder();
    double getThrottle();
    double getAileronTrim();
    double getElevatorTrim();
    bool   getButtonState( int index );

    bool notWorking();

    inline void setElevatorAxis( int axis ) { elevator_axis = axis; }
    inline void setAileronAxis( int axis ) { aileron_axis = axis; }
    inline void setRudderAxis( int axis ) { rudder_axis = axis; }
    inline void setThrottleAxis( int axis ) { throttle_axis = axis; }
    inline void setAileronTrimAxis( int axis ) { aileron_trim_axis = axis; }
    inline void setElevatorTrimAxis( int axis ) { elevator_trim_axis = axis; }
    void setDeadbands( float axis0, float axis1, float axis2 );

private:

    jsJoystick* js;
    int numAxes;
    int numButtons;
    int buttons;
    float* ax;
    int* b;
    float* deadband;
    int elevator_axis;
    int aileron_axis;
    int rudder_axis;
    int throttle_axis;
    int aileron_trim_axis;
    int elevator_trim_axis;
    int MAX_AXES;
    int MAX_BUTTONS;
    int MAX_DEADBANDS;
};

#endif // JOYSTICK_H

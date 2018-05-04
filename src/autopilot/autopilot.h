/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef AUTOPILOT_H
#define AUTOPILOT_H

////////////////////////////////////////////////////////////////////////////////

#include "pid.h"

////////////////////////////////////////////////////////////////////////////////


class autopilot
{
public:
    autopilot();
    ~autopilot();
    enum vMode { VNone, AltHold, VertSpeed, Speed, VNAV, ILS };
    enum lMode { LNone, Heading, LNAV, LOC };
    void mainLoop( double dt );

    // get the new elevator command based on current pitch and altitude
    double updateVMode( double dt, double pitch, double alt );

    // get the new aileron command based on current bank and course
    double updateLMode( double dt, double bank, double crs );

    void setEngaged( bool engage );
    void setVMode( vMode mode );
    void setLMode( lMode mode );
    void setDesiredAlt( double alt );
    void setDesiredCrs( double crs );
    void setDesiredHdg( double hdg );
    void setDesiredVS( double vs );

    double getFDPitchCmd();
    double getFDRollCmd();
    vMode getVMode();
    lMode getLMode();
    bool getEngaged();

private:
    bool engaged;
    vMode VMode;
    lMode LMode;
    PID* vsPID;        // uses elevator to set pitch angle
    PID* baPID;        // uses aileron to set bank angle
    PID* vPID;         // uses pitch angle to set altitude
    PID* lPID;         // uses bank angle to set course

    double elevator, desiredPitch, pitch, desiredAlt, altitude;
    double aileron, desiredBank, bank, desiredCrs, course;
    double desiredHdg, desiredVS;
};

#endif // AUTOPILOT_H

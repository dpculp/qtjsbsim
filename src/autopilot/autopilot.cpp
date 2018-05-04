/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "autopilot.h"

////////////////////////////////////////////////////////////////////////////////


autopilot::autopilot()
{
    elevator = desiredPitch = desiredAlt = 0.0;
    aileron = desiredBank = desiredCrs = 0.0;
    desiredHdg = desiredVS = 0.0;
    VMode = VertSpeed;
    LMode = Heading;
    engaged = false;
    vsPID = new PID(0.01, 0.8, -0.4, 0.05, 0.0005, 0.05);
    baPID = new PID(0.01, 0.8, -0.8, 0.01, 0.0001, 0.001);
    vPID =  new PID(0.01, 15.0, -5.0, 0.001, 0.00001, 0.0001);
    lPID =  new PID(0.01, 25.0, -25.0, 0.5, 0.005, 0.05);
}

void autopilot::setEngaged( bool engage ) { engaged = engage; }
void autopilot::setVMode( vMode mode ) { VMode = mode; }
void autopilot::setLMode( lMode mode ) { LMode = mode; }
void autopilot::setDesiredAlt(double alt) { desiredAlt = alt; }
void autopilot::setDesiredCrs(double crs) { desiredCrs = crs; }
void autopilot::setDesiredHdg( double hdg ) { desiredHdg = hdg; }
void autopilot::setDesiredVS( double vs ) { desiredVS = vs; }

double autopilot::getFDPitchCmd() { return desiredPitch; }
double autopilot::getFDRollCmd() { return desiredBank; }

autopilot::vMode autopilot::getVMode() { return VMode; }
autopilot::lMode autopilot::getLMode() { return LMode; }

bool autopilot::getEngaged() { return engaged; }

void autopilot::mainLoop(double dt)
{

}

double autopilot::updateVMode(double dt, double pitch, double alt)
{
   switch (VMode)
   {
     case AltHold:
       desiredPitch = vPID->calculate(dt, desiredAlt, alt);
       elevator = -vsPID->calculate(dt, desiredPitch, pitch);
       break;
     case VertSpeed:
       break;
     case Speed:
       break;
     case VNAV:
       break;
     case ILS:
       break;
     default:
       elevator = 0.0;
   }

   return elevator;
}

double autopilot::updateLMode(double dt, double bank, double crs)
{
    switch (LMode)
    {
      case Heading:
        desiredBank = -lPID->calculate(dt, desiredCrs, crs);
        aileron = baPID->calculate(dt, desiredBank, bank);
        break;
      case LNAV:
        break;
      case LOC:
        break;
      default:
        aileron = 0.0;
    }

   return aileron;
}

autopilot::~autopilot()
{
    delete vsPID;
    delete baPID;
    delete vPID;
    delete lPID;
}


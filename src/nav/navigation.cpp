/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     30 Apr 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "navigation.h"

////////////////////////////////////////////////////////////////////////////////


navigation::navigation()
{
  d2r = 0.017453293;
  r2d = 57.29577951;
  ft2nm = 0.000164579;
  ILS = false;
  rwyLen = 10000.0;
}

void navigation::SetMagVar(double var) {magvar=var;}
void navigation::SetILSRunwayEndPoints(double nearLat, double nearLon,
                                       double farLat, double farLon)
   {rwyNearLat=nearLat; rwyNearLon=nearLon; rwyFarLat=farLat; rwyFarLon=farLon;}
void navigation::SetRwyLength(double len) {rwyLen=len;}
void navigation::SetILSFrequency(float freq) {ILSfreq=freq;}
void navigation::SetILSCourseMag(double course) {ILSCourse=course;}
void navigation::SetILSGS(double gs) {ILSGS=gs;}
void navigation::SetILSTDZE(double elev) {ILSTDZE=elev;}
void navigation::SetPosition(double lat, double lon, double alt)
   {pos.lat=lat; pos.lon=lon; pos.alt=alt;}
void navigation::SetTrueHeading(double hdg) {trueHdg=hdg;}
void navigation::SetILSEngaged(bool set) {ILS=set;}
void navigation::SetHDGEngaged(bool set) {HDG=set;}
void navigation::SetSelectedHdg(int val) {selectedHDG=val;}

double navigation::GetMagHdg() {return trueHdg-magvar;}
double navigation::GetCourseErrorDeg() {return courseError;}
double navigation::GetGSErrorDeg() {return GSError;}
double navigation::GetInterceptAngle() {return interceptAngle;}
double navigation::GetDistance() {return distance;}
int navigation::GetSelectedHdg() {return selectedHDG;}
double navigation::GetILSCourse() {return ILSCourse;}
QString navigation::GetIdentifier() {return QString("ILAX");}


///////////////////////////////////////////////////////////////////////////////
/// Get true bearing (deg) and distance (nm) from start to finish
///
BrgDst navigation::GetBearingDistance(GeoLoc start, GeoLoc finish) {
    BrgDst bd;
    double deltaL = (finish.lon - start.lon)*d2r;
    double y = sin(deltaL) * cos(finish.lat*d2r);
    double x = cos(start.lat*d2r) * sin(finish.lat*d2r) -
            sin(start.lat*d2r) * cos(finish.lat*d2r) * cos(deltaL);
    bd.bearing = atan2(y, x)*r2d;  // true bearing in degrees
    bd.bearing = fmod((bd.bearing + 360.0), 360);

    double R = 3440.06479482;  // Radius of Earth in nautical miles
    double p1 = start.lat*d2r;
    double p2 = finish.lat*d2r;
    double dp = (finish.lat - start.lat)*d2r;
    double dq = (finish.lon - start.lon)*d2r;
    double a = sin(dp*0.5) * sin(dp*0.5) +
               cos(p1) * cos(p2) * sin(dq*0.5) * sin(dq*0.5);
    double c = 2.0 * (atan2(sqrt(a), sqrt(1.0-a)));
    bd.distance = R*c;

    return bd;
}


///////////////////////////////////////////////////////////////////////////////
// Do calculations for the active navigational mode
//
void navigation::Update() {
    if (ILS) {
        dest.lat = rwyFarLat;
        dest.lon = rwyFarLon;
        BrgDst bd = GetBearingDistance(pos, dest);
        courseError = ILSCourse + magvar - bd.bearing;  // course error (mag, deg)
        distance = bd.distance - (rwyLen * ft2nm); // distance to approach end of runway (nm)
        double a = bd.distance - ((rwyLen - 1080.0) * ft2nm);
        double b = (pos.alt - ILSTDZE) * ft2nm;
        GSError = atan2(b, a)*r2d - ILSGS;
    }
}

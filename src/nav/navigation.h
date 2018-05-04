/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     30 Apr 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef NAVIGATION_H
#define NAVIGATION_H

////////////////////////////////////////////////////////////////////////////////

#include "math.h"
#include <QString>

////////////////////////////////////////////////////////////////////////////////


struct GeoLoc {
    double lat;  // degrees -90 to 90
    double lon;  // degrees -180 to 180
    double alt;  // feet above the elipsoid (approx. altitude MSL)
};


struct BrgDst {
    double bearing;   // degrees 0 to 360
    double distance;  // nautical miles
};


class navigation
{
public:
    navigation();
    void Update();
    void SetMagVar(double var);
    void SetILSRunwayEndPoints(double nearLat, double nearLon, double farLat, double farLon);
    void SetRwyLength(double len);
    void SetILSFrequency(float freq);
    void SetILSCourseMag(double course);
    void SetILSGS(double gs);
    void SetILSTDZE(double elev);
    void SetPosition(double lat, double lon, double alt);
    void SetTrueHeading(double hdg);
    void SetILSEngaged(bool set);
    void SetSelectedHdg(int val);
    void SetHDGEngaged(bool set);

    double GetMagHdg();
    BrgDst GetBearingDistance(GeoLoc start, GeoLoc finish);
    double GetCourseErrorDeg();
    double GetGSErrorDeg();
    double GetInterceptAngle();
    double GetDistance();
    int    GetSelectedHdg();
    double GetILSCourse();
    QString GetIdentifier();

    double d2r, r2d, ft2nm;

private:
    double magvar;
    double rwyNearLat, rwyNearLon, rwyFarLat, rwyFarLon;
    double rwyLen;
    float  ILSfreq;
    double ILSCourse;
    double ILSGS;
    double ILSTDZE;
    double trueHdg;
    int    selectedHDG;
    double courseError;
    double GSError;
    double interceptAngle;
    double distance;
    GeoLoc pos;
    GeoLoc dest;
    bool ILS;
    bool HDG;
};

#endif // NAVIGATION_H

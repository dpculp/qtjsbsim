/******************************************************************************/
// License:  GPLv3  (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     15 Dec 2015
// Copyright (C) 2015  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef VIEWER_H
#define VIEWER_H

////////////////////////////////////////////////////////////////////////////////

#include <QDialog>
#include <QWebFrame>
#include "hud.h"



////////////////////////////////////////////////////////////////////////////////
// The aircraft class contains the data that is made available to the javascript
// environment in the QWebView object.  This is used to update the camera position.
//
class aircraft : public QObject
{
    Q_OBJECT

public:
    aircraft(QObject* parent=0) : QObject(parent) {
        a_lat=a_lon=a_alt=a_pitch=a_roll=a_hdg_true=
        cameraH=cameraV=0;
    }

    Q_INVOKABLE inline double lat()      {return a_lat;}
    Q_INVOKABLE inline double lon()      {return a_lon;}
    Q_INVOKABLE inline double alt()      {return a_alt;}
    Q_INVOKABLE inline double pitch()    {return a_pitch;}
    Q_INVOKABLE inline double roll()     {return a_roll;}
    Q_INVOKABLE inline double hdg_true() {return a_hdg_true;}
    Q_INVOKABLE inline double camBiasH()    {return cameraH;}
    Q_INVOKABLE inline double camBiasV()    {return cameraV;}
    inline void setLat(double lat)          { a_lat = lat; }
    inline void setLon(double lon)          { a_lon = lon; }
    inline void setAlt(double alt)          { a_alt = alt; }
    inline void setPitch(double pitch)      { a_pitch = pitch; }
    inline void setRoll(double roll)        { a_roll = roll; }
    inline void setHdgTrue(double hdg_true) { a_hdg_true = hdg_true; }
    inline void setCameraH(int bias)     { cameraH = (double)bias; }
    inline void setCameraV(int bias)     { cameraV = (double)bias; }

private:
    double a_lat, a_lon, a_alt, a_pitch, a_roll, a_hdg_true,
           cameraH, cameraV;
};



namespace Ui {
class Viewer;
}

class Viewer : public QDialog
{
    Q_OBJECT

public:
    explicit Viewer(QWidget *parent = 0);
    ~Viewer();
    QWebFrame* frame;
    void load(QUrl url);
    void shareAircraftObject();

    void updateData(double lat,    // geocentric latitude in degrees
                    double lon,    // geocentric longitude in degrees
                    double alt,    // altitude MSL in feet
                    double pitch,  // degrees
                    double roll,   // degrees
                    double yaw);   // degrees

    void updateHUD(double alpha,   // degrees
                   double mach,    // mach number
                   double gs,      // altitude [feet above WGS-84 datum]
                   double vs);     // vertical speed [feet/minute]

    aircraft* Aircraft;
    void setCameraBias(int horizontal, int vertical);

private slots:
    void on_CloseButton_clicked();   // close the viewer
    void on_Button5_clicked();       // toggle HUD on/off
    void on_Button1_clicked();       // look left
    void on_Button2_clicked();       // look right
    void on_Button3_clicked();       // look up
    void on_Button4_clicked();       // look down
    void on_Button6_clicked();       // return view to center

private:
    Ui::Viewer *ui;
    int viewBiasH;
    int viewBiasV;
    HUD* hud;

};

#endif // VIEWER_H

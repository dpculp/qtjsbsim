/******************************************************************************/
// License:  GPLv3  (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     15 Dec 2015
// Copyright (C) 2015  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "viewer.h"
#include "ui_viewer.h"

////////////////////////////////////////////////////////////////////////////////


Viewer::Viewer(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Viewer)
{
    ui->setupUi(this);
    ui->CloseButton->setEnabled(true);
    Aircraft = new aircraft(this);
    frame = ui->webView->page()->mainFrame();
    viewBiasH = viewBiasV = 0;

    // can't share the C++ object until the javascript environment is ready for it
    connect(frame, &QWebFrame::javaScriptWindowObjectCleared, this, &Viewer::shareAircraftObject);

    // create the Heads Up Display
    hud = new HUD(ui->HUDView);
    hud->setVisible(false);

}


Viewer::~Viewer()
{
    delete ui;
    delete Aircraft;
    delete hud;
}


void Viewer::load(QUrl url)
{
    ui->webView->load(url);
}


void Viewer::updateData(double lat, double lon, double alt,
                        double pitch, double roll, double yaw)
{
    // this data is shared to the Cesium viewer
    Aircraft->setLat(lat);
    Aircraft->setLon(lon);
    Aircraft->setAlt(alt);
    Aircraft->setPitch(pitch);
    Aircraft->setRoll(roll);
    Aircraft->setHdgTrue(yaw);

    // share this data with the HUD
    hud->setPitch(pitch);
    hud->setAltitude(alt);
    hud->setRoll(roll);

}


void Viewer::updateHUD(double alpha, double mach, double gs, double vs)
{
    hud->setAlpha(alpha);
    hud->setMach(mach);
    hud->setGroundSpeed(gs);
    hud->setVerticalSpeedFPM(vs);
    hud->update();
}


void Viewer::shareAircraftObject()
{
    frame->addToJavaScriptWindowObject("aircraft", Aircraft);
}


void Viewer::on_CloseButton_clicked()
{
    this->close();
}


// Toggle the HUD on/off
void Viewer::on_Button5_clicked()
{
    hud->setVisible( !hud->getVisible() );
}


void Viewer::setCameraBias(int horizontal, int vertical)
{
    viewBiasH = horizontal;
    viewBiasV = vertical;
    Aircraft->setCameraH(viewBiasH);
    Aircraft->setCameraV(viewBiasV);
}


//////////////////////////////////////////////////////////////////
//
// Buttons 1,2,3,4,6 control the view offset.  The offsets are
// integers, but the JavaScript bridge expects real numbers.

// look left
void Viewer::on_Button1_clicked()
{
    viewBiasH -= 1;
    Aircraft->setCameraH(viewBiasH);
}

// look right
void Viewer::on_Button2_clicked()
{
    viewBiasH += 1;
    Aircraft->setCameraH(viewBiasH);
}

// look up
void Viewer::on_Button3_clicked()
{
    viewBiasV += 1;
    Aircraft->setCameraV(viewBiasV);
}

// look down
void Viewer::on_Button4_clicked()
{
    viewBiasV -= 1;
    Aircraft->setCameraV(viewBiasV);
}

// return view to center
void Viewer::on_Button6_clicked()
{
    viewBiasH = viewBiasV = 0;
    Aircraft->setCameraH(viewBiasH);
    Aircraft->setCameraV(viewBiasV);
}


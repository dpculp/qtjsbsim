/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     30 Apr 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "hud.h"

////////////////////////////////////////////////////////////////////////////////


HUD::HUD( QGraphicsView* gv)
{
    hudScene = new QGraphicsScene;
    hudView = gv;
    visible = false;
    pitch = roll = alpha = mach = gs = alt = vs = beta = 0.0;
    ppd = 9.6;       // screen y coordinates per degree of pitch
    DEG2RAD = 0.01745329251;
    HUDfont.setFamily("Arial");
    HUDfont.setPointSize(10);
    HUDfont.setWeight(QFont::Bold);
    QColor HUDcolor(100,255,100);

    // the base layer
    hudBasePixmap.load(":/images/hud/hud_base.png");
    hudScene->addPixmap(hudBasePixmap);

    // horizon line
    horizonPixmap.load(":/images/hud/horizon.png");
    horizonItem = hudScene->addPixmap(horizonPixmap);
    horizonStartPos.setX(100);
    horizonStartPos.setY(185);
    horizonItem->setTransformOriginPoint(horizonPixmap.rect().center());
    horizonItem->setPos(horizonStartPos);

    // flight path marker
    fpmPixmap.load(":/images/hud/fpm.png");
    fpmItem = hudScene->addPixmap(fpmPixmap);
    fpmStartPos.setX(292);
    fpmStartPos.setY(182);
    fpmItem->setPos(fpmStartPos);

    // mach display
    machItem = new QGraphicsTextItem( QString("9999") );
    machItem->setDefaultTextColor(HUDcolor);
    machItem->setFont(HUDfont);
    machItem->setPos(36,317);
    hudScene->addItem(machItem);

    // ground speed display
    gsItem = new QGraphicsTextItem( QString("9999") );
    gsItem->setDefaultTextColor(HUDcolor);
    gsItem->setFont(HUDfont);
    gsItem->setPos(36,40);
    hudScene->addItem(gsItem);

    // altitude display
    altItem = new QGraphicsTextItem( QString("9999") );
    altItem->setDefaultTextColor(HUDcolor);
    altItem->setFont(HUDfont);
    altItem->setPos(510,40);
    hudScene->addItem(altItem);

    // vertical speed display
    vsiItem = new QGraphicsTextItem( QString("9999") );
    vsiItem->setDefaultTextColor(HUDcolor);
    vsiItem->setFont(HUDfont);
    vsiItem->setPos(510,317);
    hudScene->addItem(vsiItem);

    hudView->setStyleSheet("background-color : rgba(0,0,0,0)");
    hudView->setScene(hudScene);

    update();
}


void HUD::setVisible(bool vis) {
    visible=vis;
    if (visible) {
        hudView->show();
    } else {
        hudView->hide();
    }
};


bool HUD::getVisible() { return visible; };


void HUD::show() { hudView->show(); }


void HUD::hide() { hudView->hide(); }


HUD::~HUD()
{
    delete hudScene;
}


void HUD::update()
{
   double cosRoll = cos(roll * DEG2RAD);
   if (cosRoll == 0.0) cosRoll = 0.000001;

   // horizon line
   QPoint pos = horizonStartPos;
   pos.setY(horizonStartPos.y() + (int)((pitch / cosRoll) * ppd));
   horizonItem->setPos(pos);
   horizonItem->setRotation(-roll);

   // flight path marker
   QPoint fpmPos = fpmStartPos;
   fpmPos.setY(fpmStartPos.y() + (int)(alpha * ppd));
   fpmItem->setPos(fpmPos);

   // mach display
   machItem->setPlainText( QString::number(mach, 'f', 2) + QString("M") );

   // ground speed display
   gsItem->setPlainText( QString("GS ") + QString::number((int)gs) );

   // altitude display
   altItem->setPlainText( QString::number((ulong)alt) + QString(" FT") );

   // vertical speed display
   vsiItem->setPlainText( QString::number((int)vs) + QString(" FPM") );
}


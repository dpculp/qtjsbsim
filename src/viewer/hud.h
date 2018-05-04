/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     30 Apr 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef HUD_H
#define HUD_H

////////////////////////////////////////////////////////////////////////////////

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QGraphicsTextItem>
#include <QPoint>
#include <QFont>
#include <math.h>

////////////////////////////////////////////////////////////////////////////////


class HUD
{
public:
    HUD( QGraphicsView *gv );
    ~HUD();
    void setVisible(bool vis);
    bool getVisible();
    void show();
    void hide();
    void update();
    inline void setPitch( double p ) { pitch = (float)p; }
    inline void setRoll( double r ) { roll = (float)r; }
    inline void setAlpha( double a ) { alpha = (float)a; }
    inline void setMach( double m ) { mach = (float)m; }
    inline void setGroundSpeed( double s ) { gs = (float)s; }
    inline void setAltitude( double a ) { alt = (float)a; }
    inline void setVerticalSpeedFPM( double v ) { vs = (float)v; }
    inline void setBeta( double b ) { beta = (float)b; }

private:

    QGraphicsScene* hudScene;
    QGraphicsView* hudView;
    QPixmap hudBasePixmap;
    QPixmap horizonPixmap;
    QPixmap fpmPixmap;
    QGraphicsPixmapItem* horizonItem;
    QGraphicsPixmapItem* fpmItem;
    QGraphicsTextItem* machItem;
    QGraphicsTextItem* gsItem;
    QGraphicsTextItem* altItem;
    QGraphicsTextItem* vsiItem;
    QPoint horizonStartPos;
    QPoint fpmStartPos;
    QFont HUDfont;
    bool visible;
    float pitch;
    float roll;
    float alpha;
    float ppd;
    float mach;
    float gs;
    float alt;
    float vs;
    float beta;
    double DEG2RAD;

};

#endif // HUD_H

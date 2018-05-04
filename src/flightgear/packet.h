/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     May 2018
// Purpose:  Maintains a copy of a FlightGear net_FDM object and handle proper
//           unit conversions and byte order conversions.
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef PACKET_H
#define PACKET_H

////////////////////////////////////////////////////////////////////////////////

#include <netinet/in.h>
#include "net_fdm.hxx"
#include <time.h>

////////////////////////////////////////////////////////////////////////////////


class Packet
{
public:
    Packet();
    ~Packet();
    void Init();
    bool EndianTest();               // returns true if little-endian
    void htond(double &x);
    void htonf(float &x);

    void setLongitude(double lon);   // degrees
    void setLatitude(double lat);    // degrees
    void setAltitude(double alt);    // feet MSL
    void setRoll(float r);           // degrees
    void setPitch(float p);          // degrees
    void setTrueHdg(float y);        // degrees
    void setVelocity(float v);       // knots calibrated airspeed
    void setClimbRate(float c);      // feet per minute
    void setAltAGL(float a);         // feet AGL
    void setBeta(float b);           // degrees
    void setP(float p);              // radians per second
    void setQ(float q);              // radians per second
    void setR(float r);              // radians per second
    void setVN(float vN);            // feet per second
    void setVE(float vE);            // feet per second
    void setVD(float vD);            // feet per second
    void setvU(float vU);            // feet per second
    void setvV(float vV);            // feet per second
    void setvW(float vW);            // feet per second


    FGNetFDM* getPacketPtr();

private:
    FGNetFDM* packet;
    bool isLittleEndian;
    float DEFAULT_VISIBILITY;
    double DEG2RAD;
    double FT2M;
    uint32_t DEFAULT_TIME;
    time_t curTime;
};

#endif // PACKET_H

/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     May 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "packet.h"

////////////////////////////////////////////////////////////////////////////////


Packet::Packet()
{
    packet = new FGNetFDM();
    isLittleEndian = true;           // default for intel x86
    isLittleEndian = EndianTest();
    DEFAULT_VISIBILITY = 30000.0;    // 30 kilometers
    DEFAULT_TIME = 1524700000;       // unix time
    DEG2RAD = 0.01745329251;
    FT2M = 0.3048;
    Init();
}


Packet::~Packet()
{
    delete packet;
}


FGNetFDM* Packet::getPacketPtr(){
    return packet;
}


void Packet::Init() {
    unsigned int i;

    packet->version = htonl(FG_NET_FDM_VERSION);
    packet->padding = htonl(0);
    htond(packet->longitude = 0.0);
    htond(packet->latitude = 0.0);
    htond(packet->altitude = 0.0);
    htonf(packet->agl = 0.0);
    htonf(packet->phi = 0.0);
    htonf(packet->theta = 0.0);
    htonf(packet->psi = 0.0);
    htonf(packet->alpha = 0.0);
    htonf(packet->beta = 0.0);
    htonf(packet->phidot = 0.0);
    htonf(packet->thetadot = 0.0);
    htonf(packet->psidot = 0.0);
    htonf(packet->vcas = 0.0);
    htonf(packet->climb_rate = 0.0);
    htonf(packet->v_north = 0.0);
    htonf(packet->v_east = 0.0);
    htonf(packet->v_down = 0.0);
    htonf(packet->v_body_u = 0.0);
    htonf(packet->v_body_v = 0.0);
    htonf(packet->v_body_w = 0.0);
    htonf(packet->A_X_pilot = 0.0);
    htonf(packet->A_Y_pilot = 0.0);
    htonf(packet->A_Z_pilot = 0.0);
    htonf(packet->stall_warning = 0.0);
    htonf(packet->slip_deg = 0.0);
    packet->num_engines = htonl(FGNetFDM::FG_MAX_ENGINES);
    for (i=0; i < FGNetFDM::FG_MAX_ENGINES; ++i) {
        packet->eng_state[i] = htonl(0);
        htonf(packet->rpm[i] = 0.0);
        htonf(packet->fuel_flow[i] = 0.0);
        htonf(packet->fuel_px[i] = 0.0);
        htonf(packet->egt[i] = 0.0);
        htonf(packet->cht[i] = 0.0);
        htonf(packet->mp_osi[i] = 0.0);
        htonf(packet->tit[i] = 0.0);
        htonf(packet->oil_temp[i] = 0.0);
        htonf(packet->oil_px[i] = 0.0);
    }
    packet->num_tanks = htonl(FGNetFDM::FG_MAX_TANKS);
    for (i=0; i < FGNetFDM::FG_MAX_TANKS; ++i) {
        htonf(packet->fuel_quantity[i] = 0.0);
    }
    packet->num_wheels = htonl(FGNetFDM::FG_MAX_WHEELS);
    for (i=0; i < FGNetFDM::FG_MAX_WHEELS; ++i) {
        packet->wow[i] = htonl(0);
        htonf(packet->gear_pos[i] = 0.0);
        htonf(packet->gear_steer[i] = 0.0);
        htonf(packet->gear_compression[i] = 0.0);
    }
    packet->cur_time = htonl((uint32_t)time(&curTime));
    packet->warp = htonl(0);
    htonf(packet->visibility = DEFAULT_VISIBILITY);
    htonf(packet->elevator = 0.0);
    htonf(packet->elevator_trim_tab = 0.0);
    htonf(packet->left_flap = 0.0);
    htonf(packet->right_flap = 0.0);
    htonf(packet->left_aileron = 0.0);
    htonf(packet->right_aileron = 0.0);
    htonf(packet->rudder = 0.0);
    htonf(packet->nose_wheel = 0.0);
    htonf(packet->speedbrake = 0.0);
    htonf(packet->spoilers = 0.0);

}

///////////////////////////////////////////////////////////////
/// Borrowed from flightgear/src/Network/native_fdm.cxx
/// Author:  Copyright 2001, Curtis L. Olson
void Packet::htond(double &x)
{
    if (isLittleEndian) {
        int* Double_Overlay;
        int  Holding_Buffer;
        Double_Overlay = (int*) &x;
        Holding_Buffer = Double_Overlay[0];
        Double_Overlay[0] = htonl(Double_Overlay[1]);
        Double_Overlay[1] = htonl(Holding_Buffer);
    } else {
        return;
    }
}


///////////////////////////////////////////////////////////////
/// Borrowed from flightgear/src/Network/native_fdm.cxx
/// Author:  Copyright 2001, Curtis L. Olson
void Packet::htonf(float &x)
{
    if (isLittleEndian) {
        int* Float_Overlay;
        int  Holding_Buffer;
        Float_Overlay = (int*) &x;
        Holding_Buffer = Float_Overlay[0];
        Float_Overlay[0] = htonl(Holding_Buffer);
    } else {
        return;
    }
}


//  Returns true if this processor is little-endian
bool Packet::EndianTest()
{
   union {short s; char c[sizeof(short)];} un;
   un.s = 0x0102;
   if (sizeof(short) == 2 ) {
       if (un.c[0] == 1 && un.c[1] == 2) { return false; }
       else if (un.c[0] == 2 && un.c[1] == 1) { return true; }
   }
   return true;
}


////////////////////////////////////////////////////////////////////////////////
// In these setting functions degrees are converted to radians, feet are
// converted to meters, velocity remains in knots calibrated, and climb
// rate is converted from fpm to fps.  Byte order of the data is converted
// from host native to network.
//
void Packet::setLongitude(double lon) { htond(packet->longitude = lon * DEG2RAD); }
void Packet::setLatitude(double lat) { htond(packet->latitude = lat * DEG2RAD); }
void Packet::setAltitude(double alt) { htond(packet->altitude = alt * FT2M); }
void Packet::setRoll(float r) { htonf(packet->phi = r * DEG2RAD); }
void Packet::setPitch(float p) { htonf(packet->theta = p * DEG2RAD); }
void Packet::setTrueHdg(float y) { htonf(packet->psi = y * DEG2RAD); }
void Packet::setVelocity(float v) { htonf(packet->vcas = v); }
void Packet::setClimbRate(float c) { htonf(packet->climb_rate = c / 60.0); }
void Packet::setAltAGL(float a) { htonf(packet->agl = a * FT2M); }
void Packet::setBeta(float b) { htonf(packet->beta = b * DEG2RAD); }
void Packet::setP(float p) { htonf(packet->phidot = p); }
void Packet::setQ(float q) { htonf(packet->thetadot = q); }
void Packet::setR(float r) { htonf(packet->psidot = r); }
void Packet::setVN(float vN) { htonf(packet->v_north = vN); }
void Packet::setVE(float vE) { htonf(packet->v_east = vE); }
void Packet::setVD(float vD) { htonf(packet->v_down = vD); }
void Packet::setvU(float vU) { htonf(packet->v_body_u = vU); }
void Packet::setvV(float vV) { htonf(packet->v_body_v = vV); }
void Packet::setvW(float vW) { htonf(packet->v_body_w = vW); }


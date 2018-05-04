/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     May 2018
// Purpose:  Maintains a copy of a UDP socket and a Packet object.
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef OUTSOCKET_H
#define OUTSOCKET_H

////////////////////////////////////////////////////////////////////////////////

#include <QUdpSocket>
#include <QtNetwork>
#include "packet.h"

////////////////////////////////////////////////////////////////////////////////


class OutSocket
{

public:
    OutSocket();
    ~OutSocket();
    Packet* getPacket();
    void setPort(int p);
    qint64 Send();
    inline bool Connected() { return connected; }

private:
    QUdpSocket* socket;
    QHostAddress hostAddress;
    bool connected;
    int port;
    Packet* packet;
    std::size_t size;
};

#endif // OUTSOCKET_H

/******************************************************************************/
// License:  GPLv3 (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     May 2018
// Copyright (C) 2018  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "outsocket.h"

////////////////////////////////////////////////////////////////////////////////


OutSocket::OutSocket()
{
    hostAddress = QHostAddress::LocalHost;
    packet = new Packet();
    connected = false;
    socket = new QUdpSocket();
    port = 0;
    size = sizeof(FGNetFDM);
}


OutSocket::~OutSocket()
{
    delete packet;
    delete socket;
}


Packet* OutSocket::getPacket()
{
    return packet;
}


void OutSocket::setPort(int p)
{
    port = p;
    connected = true;
}

////////////////////////////////////////////////////////////////////////////////
// Return value is the number of bytes actually sent.
//
qint64 OutSocket::Send()
{
    FGNetFDM* datagram = packet->getPacketPtr();
    return socket->writeDatagram(QByteArray((char*)datagram, size),
                                 hostAddress, port);
}


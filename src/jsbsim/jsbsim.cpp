/******************************************************************************/
// License:  GPLv3  (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     15 Dec 2015
// Copyright (C) 2015  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#include "jsbsim.h"

////////////////////////////////////////////////////////////////////////////////

jsbsim::jsbsim( void ) {

    setProcessChannelMode(QProcess::MergedChannels);
    connect (this, SIGNAL(readyReadStandardOutput()),
             this, SLOT(logOutput()));
}


jsbsim::~jsbsim() {

}


void jsbsim::logOutput() {
    QByteArray bytes = readAllStandardOutput();
    QStringList lines = QString(bytes).split("\n");
    foreach (QString line, lines) {
        line = process(line);
        emit logString(line);
    }
}


// remove the console control strings
//
QString jsbsim::process(const QString& str) {
    int startpos = 0, stoppos = 0;
    QString rstr = str;

    while (1){
    if ((startpos = rstr.indexOf(QChar(0x1b), startpos)) == -1) break;
    stoppos = rstr.indexOf(QChar(0x6d), startpos) + 1;
    rstr.remove(startpos, stoppos);
    }

    return rstr;
}

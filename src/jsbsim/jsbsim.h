/******************************************************************************/
// License:  GPLv3  (http://www.gnu.org/licenses/gpl.html)
// Author:   David Culp
// Date:     15 Dec 2015
// Copyright (C) 2015  David P. Culp (daveculp@cox.net)
// Purpose:  Encapsulates the JSBSim app as a QProcess, mainly so that the
//           stdout can be captured and displayed in a console widget.
/******************************************************************************/

#ifndef JSBSIM_H
#define JSBSIM_H

////////////////////////////////////////////////////////////////////////////////

#include <QObject>
#include <QProcess>

////////////////////////////////////////////////////////////////////////////////


class jsbsim : public QProcess {

    Q_OBJECT

  public:
    jsbsim();
    ~jsbsim();

  signals:
    void logString(const QString &str);

  public slots:
    void logOutput();

  private:
    QString process(const QString &str);
};

#endif // JSBSIM_H


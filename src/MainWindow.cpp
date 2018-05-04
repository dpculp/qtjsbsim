//
// This project is an adaptation of Marek Cel's QFlightInstruments tools to use
// the JSBSim flight dynamics engine (http://sourceforge.net/projects/jsbsim/).
// The PLIB libraries (http://plib.sourceforge.net) are used for joystick
// functionality.  The QFlightInstruments project is hosted at
// (http://sourceforge.net/projects/qfi/).
//
// License is GPLv3.   (http://www.gnu.org/licenses/gpl.html)
// David Culp, Jan 2015
// Copyright (C) 2015  David P. Culp (daveculp@cox.net)
/******************************************************************************/

#ifndef MAINWINDOW_CPP
#define MAINWINDOW_CPP
#endif

////////////////////////////////////////////////////////////////////////////////

#include <iostream>
#include <math.h>
#include "MainWindow.h"
#include "ui_MainWindow.h"
#include <QDesktopServices>


////////////////////////////////////////////////////////////////////////////////

using namespace std;

////////////////////////////////////////////////////////////////////////////////

MainWindow::MainWindow( QWidget * parent ) :
        QMainWindow( parent ),
        m_ui( new Ui::MainWindow )

{

    m_ui->setupUi( this );
    viewer = new Viewer(parent);
    debug = new debugTable(m_ui->tab_15);

    installPath = getInstallPath();
    iniPath = installPath + "/src/qtjsbsim.ini";

    // Set up the UI frame rate monitoring timer
    framerate = 0.0;
    dt = 0.0;
    count = 0;
    UItimer = new QTime();
    UItimer->start();

    // Set up time stamp for the outgoing UDP datagrams
    timeStamp = new QTime();
    timeStamp->start();

    // Initialize variables
    aileron = elevator = rudder = throttle = 0.0;
    pitch_trim = -0.15;
    pitch_trim_rate = 0.2;
    aileron_trim = rudder_trim = 0.0;
    pitch_trim_pos = aileron_trim_pos = rudder_trim_pos = 0.0;
    aileron_pos = elevator_pos = rudder_pos = throttle_pos = 0.0;
    aoa = beta = pitch = roll = yaw = 0.0;
    phi = theta = psi = P = Q = R = 0.0;
    altitude = latitude = longitude = GeoLat = 0.0;
    GeoAlt = altAGL = altTerrain = ECRadius = CGRadius = 0.0;
    altTerrain = 95.248;  // temporary hack *******************************
    climb_rate = airspeed = mach = vtrue = gs = 0.0;
    vN = vE = vD = vU = vV = vW = 0.0;
    fuel_total = weight = empty_weight = payload = 0.0;
    payload_cmd = 10000.0;
    wind_dir = wind_speed = deltaT = 0.0;
    Pt = Qbar = TAT = OAT = Re = g = dt = 0.0;
    steering = 0.0;
    numEngines = 0;
    selected_engine = -1.0;
    flap_pos = flap_cmd = speedbrakes_cmd = steering = 0.0;
    gear_pos = gear_cmd = 1.0;
    speedbrake_pos = simtime = fuel_total = payload = 0.0;
    weight = empty_weight = 1.0;
    payload_cmd = 0;
    wind_dir = wind_speed = deltaT = 0.0;
    Pt = Qbar = TAT = OAT = Re = g = rho = 0.0;
    jsbsimTerminate = jsbsim_dt = jsbsimPause = 0.0;
    jsbsimDataSocket = NULL;
    jsbsimControlsSocket = NULL;
    JSBSim = NULL;
    firstTime = true;
    FPM_enabled = false;
    rEllipse = 0.0;
    connected = false;
    counter = 0;
    ui_rate = 100.0;
    output_rate = 10;
    fg_rate = 20;
    ConnectToFG = false;
    for (uint i=0; i<10; i++) {
        thrust[i] = 0.0;
        n1[i] = 0.0;
        n2[i] = 0.0;
        epr[i] = 0.0;
        egt[i] = 0.0;
        ff[i] = 0.0;
        rpm[i] = 0.0;
        rpm_prop[i] = 0.0;
        gph[i] = 0.0;
        power[i] = 0.0;
        torque[i] = 0.0;
        prop_angle[i] = 0.0;
        starter[i] = 0.0;
        cutoff[i] = 0.0;
    }
    hostAddress = QHostAddress::LocalHost;
    inputPort = 5138;
    outputPort = 5139;
    telnetPort = 5137;
    localURL = remoteURL = QUrl("");
    useLocal = false;

    // Group some radio buttons
    gearCmd = new QButtonGroup(m_ui->ControlsTab);
    gearCmd->addButton(m_ui->GearUpButton);
    gearCmd->addButton(m_ui->GearDownButton);
    m_ui->GearDownButton->setChecked(true);
    sbCmd = new QButtonGroup(m_ui->ControlsTab);
    sbCmd->addButton(m_ui->SBUpButton);
    sbCmd->addButton(m_ui->SBDownButton);
    m_ui->SBDownButton->setChecked(true);


    // Set up joystick
    joystick = new Joystick();
    joystick->Init();
    m_ui->label_joystick->setText(joystick->getName());
    if (joystick->notWorking()) {
        m_ui->console->appendPlainText("Joystick not found!");
    } else {
        m_ui->console->appendPlainText("Joystick axes = " +
                                       QString::number(joystick->getNumAxes()));
        m_ui->console->appendPlainText("Joystick buttons = " +
                                       QString::number(joystick->getNumButtons()));
    }

    // Create a navigation instance
    nav = new navigation;
    nav->SetILSEngaged(false);

    // Create an autopilot instance
    ap = new autopilot;
    FDvisible = false;

    // Create interface with FlightGear
    FGSocket = new OutSocket();

    // Read settings from init file
    readSettings();

    // Set up JSBSim
    JSBSim = new jsbsim();
    connect(JSBSim, SIGNAL(error(QProcess::ProcessError)), this,
                    SLOT(processError(QProcess::ProcessError)));
    JSBSim->connect(JSBSim, SIGNAL(logString(const QString&)),
                 m_ui->jconsole, SLOT(appendPlainText(QString)));
    connect(m_ui->StartJSBSimButton, SIGNAL(clicked()), this,
                     SLOT(StartJSBSim()));

    // Set up timer for the UI main simulation loop (default = 100 hz).
    iterating = false;
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(MainLoop()));
    timer->start(1000.0/ui_rate);
    iterating = true;

    // Set up UDP output timer for sending datagrams to JSBSim. (default = 10 hz).
    UDPOutTimer = new QTimer(this);
    connect(UDPOutTimer, SIGNAL(timeout()), this, SLOT(setJSBSimControls()));
    UDPOutTimer->start(1000.0/output_rate);

    // Set up UDP output timer for sending datagrams to FlightGear. (default = 20 hz).
    FGTimer = new QTimer(this);
    connect(FGTimer, SIGNAL(timeout()), this, SLOT(sendFlightGearData()));
    FGTimer->start(1000.0/fg_rate);

}


////////////////////////////////////////////////////////////////////////////////

MainWindow::~MainWindow()
{
    delete UItimer;
    delete timeStamp;
    delete UDPOutTimer;
    delete timer;
    delete joystick;
    delete JSBSim;
    if (jsbsimDataSocket) delete jsbsimDataSocket;
    if (jsbsimControlsSocket) delete jsbsimControlsSocket;
    delete gearCmd;
    delete sbCmd;
    delete viewer;
    delete debug;
    delete nav;
    delete ap;
    delete FGSocket;
    if ( m_ui ) { delete m_ui; m_ui = 0; }
}


////////////////////////////////////////////////////////////////////////////////
// The main loop slot.  It includes a frame rate checker so that actual frame
// rate can be checked against requested frame rate.  Note that the functions that
// send data to JSBSim and FlightGear aren't in here because they're called by
// other timers with different rates.
//
void MainWindow::MainLoop()
{
    if (count == 100) {
        framerate = 100.0 / UItimer->restart() * 1000.0;
        dt = 1.0 / framerate;
        count = 0;
    }

    get_joystick_inputs();
    getJSBSimData();
    UpdateNavigation();
    UpdateAutopilot();
    UpdateDisplays();
    doDebugCalculations();
    debug->update();
    viewer->updateData(latitude, longitude, altitude, pitch, roll, yaw);
    viewer->updateHUD(aoa, mach, gs, climb_rate);
    count++;
}



////////////////////////////////////////////////////////////////////////////////
// Reads the joystick state and assigns the values to aircraft control inputs.
// The max number of axes supported is 16.
// Pitch trim rate is a normalized value (0-1) per second, so
// for example, 0.2 results in five seconds to go from 0 to 1.
//
void MainWindow::get_joystick_inputs(void)
{
    if (joystick->notWorking()) return;
    joystick->Read();

    aileron  = joystick->getAileron();
    elevator = joystick->getElevator();
    rudder   = joystick->getRudder();
    throttle = joystick->getThrottle();
    aileron_trim += joystick->getAileronTrim() * dt/5.0;
    pitch_trim -= joystick->getElevatorTrim() * dt * pitch_trim_rate;

    // update the joystick display
    m_ui->aileronSlider->setValue(aileron*100);
    m_ui->aileronLabel->setText(QString::number(aileron, 'f', 3));
    m_ui->elevatorSlider->setValue(elevator*100);
    m_ui->elevatorLabel->setText(QString::number(elevator, 'f', 3));
    m_ui->rudderSlider->setValue(-rudder*100);
    m_ui->rudderLabel->setText(QString::number(rudder, 'f', 3));
    m_ui->throttleSlider->setValue(throttle*100);
    m_ui->throttleLabel->setText(QString::number(throttle, 'f', 3));

    m_ui->jsButton_0->setChecked(joystick->getButtonState(0));
    m_ui->jsButton_1->setChecked(joystick->getButtonState(1));
    m_ui->jsButton_2->setChecked(joystick->getButtonState(2));
    m_ui->jsButton_3->setChecked(joystick->getButtonState(3));
    m_ui->jsButton_4->setChecked(joystick->getButtonState(4));
    m_ui->jsButton_5->setChecked(joystick->getButtonState(5));
    m_ui->jsButton_6->setChecked(joystick->getButtonState(6));
    m_ui->jsButton_7->setChecked(joystick->getButtonState(7));
    m_ui->jsButton_8->setChecked(joystick->getButtonState(8));
    m_ui->jsButton_9->setChecked(joystick->getButtonState(9));
    m_ui->jsButton_10->setChecked(joystick->getButtonState(10));
    m_ui->jsButton_11->setChecked(joystick->getButtonState(11));
    m_ui->jsButton_12->setChecked(joystick->getButtonState(12));

}


////////////////////////////////////////////////////////////////////////////////
// Gets keyboard key events and sets associated variables. These are later sent
// to JSBSim over the socket.  Note that the GUI can prevent this from working
// if an input widget has the keyboard focus. FIXME: find a work-around for this
//
void MainWindow::keyPressEvent(QKeyEvent* event)
{
    int keycode = event->key();
    switch (keycode) {
        case Qt::Key_W:
           m_ui->console->appendPlainText("W key pressed");  // for testing only
           break;
        case Qt::Key_G:
           toggle( &gear_cmd );           // toggle gear
           if (gear_cmd) m_ui->GearDownButton->setChecked(true);
           else m_ui->GearUpButton->setChecked(true);
           break;
        case Qt::Key_BracketRight:        // flaps down one step
           step( &flap_cmd, 9, 1 );
           break;
        case Qt::Key_BracketLeft:         // flaps up one step
           step( &flap_cmd, 9, -1 );
           break;
        case Qt::Key_T:
           toggle( &jsbsimTerminate );    // terminate JSBSim
           break;
        case Qt::Key_S:                   // engine starter(s)
           if (selected_engine == -1) {
           toggle( &starter[0] );
           toggle( &starter[1] );
           toggle( &starter[2] );
           toggle( &starter[3] );
           } else {
               toggle( &starter[(int)selected_engine - 1] );
           }
           break;
        case Qt::Key_Q:                   // engine fuel cutoff(s)
           if (selected_engine == -1) {
           toggle( &cutoff[0] );
           toggle( &cutoff[1] );
           toggle( &cutoff[2] );
           toggle( &cutoff[3] );
           } else {
               toggle( &cutoff[(int)selected_engine - 1] );
           }
           break;
        case Qt::Key_M:
           toggle( &steering );           // (nose/tail) wheel steering
           break;
        case Qt::Key_P:
           m_ui->PauseJSBSimButton->click();   // pause/resume JSBSim
           break;
        case Qt::Key_1:
           selected_engine = 1.0;         // select engine one
           break;
        case Qt::Key_2:
           selected_engine = 2.0;         // select engine two
           break;
        case Qt::Key_3:
           selected_engine = 3.0;         // select engine three
           break;
        case Qt::Key_4:
           selected_engine = 4.0;         // select engine four
           break;
        case Qt::Key_AsciiTilde:
           selected_engine = -1.0;        // select all engines
           break;

    }

}


////////////////////////////////////////////////////////////////////////////////
// Reads a waiting datagram from JSBSim and sets variables.  The datagram is a
// series of comma separated strings. Each string represents a value of type
// double.
//
void MainWindow::getJSBSimData(void)
{
    if (!connected) return;
    if (!jsbsimDataSocket->hasPendingDatagrams()) return;

    QByteArray datagram;
    QList<QByteArray> tokens;
    QVector<double> data(200);

    datagram.resize(jsbsimDataSocket->pendingDatagramSize());
    QHostAddress sender;
    quint16 senderPort;
    jsbsimDataSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
    if (datagram.at(0) == 0x3C) return;       // throw away JSBSim's header datagram

    datagram = datagram.trimmed();            // remove white space
    tokens = datagram.split(',');             // tokenize

    for (int i=0; i<tokens.size(); i++)
      data[i] = tokens[i].trimmed().toDouble();

    simtime = data.at(0);
    airspeed = data.at(1);
    altitude = data.at(2);
    pitch = data.at(3);
    roll = data.at(4);
    climb_rate = data.at(5) * 60.0;  // fps to fpm
    mach = data.at(6);
    beta = data.at(7);
    yaw = data.at(8);
    aoa = data.at(9);
    P = data.at(10);
    Q = data.at(11);
    R = data.at(12);
    vtrue = data.at(13);
    gs = data.at(14) * 0.5924838;    // fps to kts
    vN = data.at(15);
    vE = data.at(16);
    vD = data.at(17);
    vU = data.at(18);
    vV = data.at(19);
    vW = data.at(20);
    altAGL = data.at(21);
    latitude = data.at(22);
    longitude = data.at(23);
    GeoLat = data.at(24);
    thrust[0] = data.at(25);
    thrust[1] = data.at(26);
    n1[0] = data.at(27);
    n1[1] = data.at(28);
    n2[0] = data.at(29);
    n2[1] = data.at(30);
    ff[0] = data.at(31) * 3600.0;    // lbs/sec to lbs/hr
    ff[1] = data.at(32) * 3600.0;    // lbs/sec to lbs/hr
    fuel_total = data.at(33);
    weight = data.at(34);
    empty_weight = data.at(35);
    gear_pos = data.at(36);
    flap_pos = data.at(37);
    speedbrake_pos = data.at(38);
    aileron_pos = data.at(39);
    elevator_pos = data.at(40);
    rudder_pos = data.at(41);
    payload = data.at(42);
    Pt = data.at(43);
    Qbar = data.at(44);
    TAT = data.at(45);
    OAT = (data.at(46) -491.67) * 0.5555555556;     // Rankine to Celsius
    Re = data.at(47);
    g = data.at(48) * -1.0;
    rho = data.at(49);
    jsbsim_dt = data.at(50);
    GeoAlt = data.at(51);
    ECRadius = data.at(52);

    // now that we have our first data from JSBSim
    if( firstTime ) {
        setDebugWatch();
        firstTime = false;
    }

}



////////////////////////////////////////////////////////////////////////////////
// Sends commands via UDP datagram to JSBSim as comma-separated strings.  The 
// first one must be a strictly increasing time stamp, since JSBSim will throw
// away any datagram it gets that is older than the previous datagram.  The rest
// of the commands must match the properties specified in the JSBSim aircraft
// configuration file.
//
void MainWindow::setJSBSimControls(void)
{
   if (!connected) return;

   if (ap->getEngaged()){
       if (ap->getVMode() != autopilot::VNone) elevator = ap_elevator_cmd;
       if (ap->getLMode() != autopilot::LNone) aileron = ap_aileron_cmd;
   }

   QByteArray datagram;
   datagram.clear();

   datagram.append(QString::number((double)timeStamp->elapsed(), 'f', 6));
   datagram.append(',');
   datagram.append(QString::number(aileron, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(elevator, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(rudder, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(pitch_trim, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(aileron_trim, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(rudder_trim, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(throttle, 'f', 5));   // one for each engine
   datagram.append(',');
   datagram.append(QString::number(throttle, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(flap_cmd, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(gear_cmd, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(speedbrakes_cmd, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(wind_dir*0.017453293, 'f', 5));   // degrees to radians
   datagram.append(',');
   datagram.append(QString::number(wind_speed*1.687809857, 'f', 5));   // knots to fps
   datagram.append(',');
   datagram.append(QString::number(deltaT*1.8, 'f', 5));    // Celsius to Rankine
   datagram.append(',');
   datagram.append(QString::number(payload_cmd, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(altTerrain, 'f', 5));
   datagram.append(',');
   datagram.append(QString::number(jsbsimTerminate, 'f', 0));
   datagram.append(',');
   datagram.append(QString::number(jsbsimPause, 'f', 0));
   datagram.append(',');

   jsbsimControlsSocket->writeDatagram(datagram, hostAddress, outputPort);

   if (jsbsimTerminate) jsbsimTerminate = 0.0;

}



////////////////////////////////////////////////////////////////////////////////
// Updates the UI with data mostly obtained from JSBSim.
//
void MainWindow::UpdateDisplays()
{

  // Send data to PFD
  m_ui->widgetPFD->setAirspeed(airspeed);
  m_ui->widgetPFD->setAltitude(altitude);
  m_ui->widgetPFD->setPitch(pitch);
  m_ui->widgetPFD->setRoll(roll);
  m_ui->widgetPFD->setClimbRate(climb_rate * 0.001);
  m_ui->widgetPFD->setMachNo(mach);
  m_ui->widgetPFD->setSlipSkid(beta);
  m_ui->widgetPFD->setHeading(nav->GetMagHdg());
  if (FPM_enabled) {
      m_ui->widgetPFD->setFlightPathMarker(-aoa, 0, true);
  } else {
      m_ui->widgetPFD->setFlightPathMarker(0, 0, false);
  }

  // Send data to tab 1 (Attitude)
  m_ui->dataLabel_1->setText(QString::number(pitch, 'f', 4));
  m_ui->dataLabel_2->setText(QString::number(roll, 'f', 4));
  m_ui->dataLabel_3->setText(QString::number(yaw, 'f',4));
  m_ui->dataLabel_4->setText(QString::number(aoa, 'f', 4));
  m_ui->dataLabel_5->setText(QString::number(beta, 'f', 4));
  m_ui->dataLabel_6->setText(QString::number(phi, 'f', 6));
  m_ui->dataLabel_7->setText(QString::number(theta, 'f', 4));
  m_ui->dataLabel_8->setText(QString::number(psi, 'f', 4));
  m_ui->dataLabel_9->setText(QString::number(P, 'f', 4));
  m_ui->dataLabel_10->setText(QString::number(Q, 'f', 4));
  m_ui->dataLabel_11->setText(QString::number(R, 'f', 4));
               // 12

  // Send data to tab 2 (Speed)
  m_ui->dataLabel_13->setText(QString::number(airspeed, 'f', 2));
  m_ui->dataLabel_14->setText(QString::number(vtrue, 'f', 2));
  m_ui->dataLabel_15->setText(QString::number(mach, 'f', 3));
  m_ui->dataLabel_16->setText(QString::number(gs, 'f', 2));
  m_ui->dataLabel_17->setText(QString::number(climb_rate, 'f', 2));
  m_ui->dataLabel_18->setText(QString::number(vN, 'f', 4));
  m_ui->dataLabel_19->setText(QString::number(vE, 'f', 4));
  m_ui->dataLabel_20->setText(QString::number(vD, 'f', 4));
  m_ui->dataLabel_21->setText(QString::number(vU, 'f', 4));
  m_ui->dataLabel_22->setText(QString::number(vV, 'f', 4));
  m_ui->dataLabel_23->setText(QString::number(vW, 'f', 4));
               // 24

  // Send data to tab 3 (Position)
  m_ui->dataLabel_25->setText(QString::number(altitude, 'f', 2));
  m_ui->dataLabel_26->setText(QString::number(altAGL, 'f', 2));
  m_ui->dataLabel_27->setText(QString::number(latitude, 'f', 5));
  m_ui->dataLabel_28->setText(QString::number(longitude, 'f', 5));
  m_ui->dataLabel_29->setText(QString::number(GeoLat, 'f', 5));
  m_ui->dataLabel_30->setText(QString::number(GeoAlt, 'f', 4));
  m_ui->dataLabel_31->setText(QString::number(ECRadius, 'f', 8));
               // 32-36

  // Send data to tab 4 (Propulsion)
  m_ui->dataLabel_37->setText(QString::number(thrust[0], 'f', 0));
  m_ui->dataLabel_38->setText(QString::number(thrust[1], 'f', 0));
  m_ui->dataLabel_39->setText(QString::number(thrust[2], 'f', 0));
  m_ui->dataLabel_40->setText(QString::number(thrust[3], 'f', 0));
  m_ui->dataLabel_41->setText(QString::number(n1[0], 'f', 2));
  m_ui->dataLabel_42->setText(QString::number(n1[1], 'f', 2));
  m_ui->dataLabel_43->setText(QString::number(n1[2], 'f', 2));
  m_ui->dataLabel_44->setText(QString::number(n1[3], 'f', 2));
  m_ui->dataLabel_45->setText(QString::number(n2[0], 'f', 2));
  m_ui->dataLabel_46->setText(QString::number(n2[1], 'f', 2));
  m_ui->dataLabel_47->setText(QString::number(n2[2], 'f', 2));
  m_ui->dataLabel_48->setText(QString::number(n2[3], 'f', 2));
  m_ui->dataLabel_49->setText(QString::number(epr[0], 'f', 2));
  m_ui->dataLabel_50->setText(QString::number(epr[1], 'f', 2));
  m_ui->dataLabel_51->setText(QString::number(epr[2], 'f', 2));
  m_ui->dataLabel_52->setText(QString::number(epr[3], 'f', 2));
  m_ui->dataLabel_53->setText(QString::number(egt[0], 'f', 2));
  m_ui->dataLabel_54->setText(QString::number(egt[1], 'f', 2));
  m_ui->dataLabel_55->setText(QString::number(egt[2], 'f', 2));
  m_ui->dataLabel_56->setText(QString::number(egt[3], 'f', 2));
  m_ui->dataLabel_57->setText(QString::number(ff[0], 'f', 0));
  m_ui->dataLabel_58->setText(QString::number(ff[1], 'f', 0));
  m_ui->dataLabel_59->setText(QString::number(ff[2], 'f', 0));
  m_ui->dataLabel_60->setText(QString::number(ff[3], 'f', 0));
  m_ui->dataLabel_61->setText(QString::number(rpm[0], 'f', 2));
  m_ui->dataLabel_62->setText(QString::number(rpm[1], 'f', 2));
  m_ui->dataLabel_63->setText(QString::number(rpm[2], 'f', 2));
  m_ui->dataLabel_64->setText(QString::number(rpm[3], 'f', 2));
               // 65-84

  // Send data to tab 5 (Sim)
  m_ui->dataLabel_85->setText("737");
  m_ui->dataLabel_86->setText(QString::number(simtime, 'f', 2));
  m_ui->dataLabel_87->setText(QString::number(jsbsim_dt, 'f', 6));
  //m_ui->dataLabel_88->setText(QString::number(_simtime, 'f', 2));
  m_ui->dataLabel_89->setText(QString::number(Pt, 'f', 2));
  m_ui->dataLabel_90->setText(QString::number(Qbar, 'f', 2));
  m_ui->dataLabel_91->setText(QString::number(TAT, 'f', 2));
  m_ui->dataLabel_92->setText(QString::number(OAT, 'f', 2));
  m_ui->dataLabel_93->setText(QString::number(Re, 'f', 0));
  m_ui->dataLabel_94->setText(QString::number(g, 'f', 2));
  m_ui->dataLabel_95->setText(QString::number(rho, 'f', 9));
  m_ui->dataLabel_96->setText(QString::number(framerate, 'f', 2));

  // Send data to tab 6 (Wt/Bal)
  m_ui->dataLabel_97->setText(QString::number(weight, 'f', 2));
  m_ui->dataLabel_98->setText(QString::number(empty_weight, 'f', 2));
  m_ui->dataLabel_99->setText(QString::number(fuel_total, 'f', 2));
  m_ui->dataLabel_100->setText(QString::number(payload, 'f', 2));
  //m_ui->dataLabel_101->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_102->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_103->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_104->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_105->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_106->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_107->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_108->setText(QString::number(_simtime, 'f', 2));

  // Send data to tab 7 (Controls)
  m_ui->dataLabel_109->setText(QString::number(elevator_pos, 'f', 3));
  m_ui->dataLabel_110->setText(QString::number(pitch_trim_pos, 'f', 2));
  m_ui->dataLabel_111->setText(QString::number(aileron_pos, 'f', 3));
  m_ui->dataLabel_112->setText(QString::number(aileron_trim_pos, 'f', 2));
  m_ui->dataLabel_113->setText(QString::number(rudder_pos, 'f', 3));
  m_ui->dataLabel_114->setText(QString::number(rudder_trim_pos, 'f', 2));
  m_ui->dataLabel_115->setText(QString::number(gear_pos, 'f', 2));
  m_ui->dataLabel_116->setText(QString::number(flap_pos, 'f', 2));
  m_ui->dataLabel_117->setText(QString::number(speedbrake_pos, 'f', 2));
  //m_ui->dataLabel_118->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_119->setText(QString::number(_simtime, 'f', 2));
  //m_ui->dataLabel_120->setText(QString::number(_simtime, 'f', 2));


  m_ui->widgetPFD->update();
  m_ui->FlapsDisplay->setValue((int)(flap_pos * 100));
  m_ui->GearDisplay->setValue((int)(gear_pos * 100));

}



////////////////////////////////////////////////////////////////////////////////
// Send updated position information from JSBSim to the navigation computer
//
void MainWindow::UpdateNavigation()
{
  nav->SetPosition(latitude, longitude, altitude);
  nav->SetTrueHeading(yaw);
  nav->Update();

  // Display ILS raw data on the PFD
  if (m_ui->ILSButton->isChecked()) {
    int rev = (abs(nav->GetILSCourse()-nav->GetMagHdg()) > 90.0) ? -1:1;
    m_ui->widgetPFD->setDotH(rev * -nav->GetCourseErrorDeg()*0.4, true);
    m_ui->widgetPFD->setDotV(-nav->GetGSErrorDeg(), true);
    m_ui->widgetPFD->setIdentifier(nav->GetIdentifier(), true);
    m_ui->widgetPFD->setDistance((float)nav->GetDistance(), true);
  } else {
    m_ui->widgetPFD->setDotH(0.0, false);
    m_ui->widgetPFD->setDotV(0.0, false);
    m_ui->widgetPFD->setIdentifier(QString(""), false);
    m_ui->widgetPFD->setDistance(0.0, false);
  }

  // debug
  intercept = -nav->GetCourseErrorDeg();
  dist = nav->GetDistance();
  gserror = -nav->GetGSErrorDeg();
}



////////////////////////////////////////////////////////////////////////////////
// A utility function to toggle a value between 1 and 0.
//
double MainWindow::toggle(double *number)
{
    if (*number != 0.0) *number = 0.0;
    else *number = 1.0;
    return *number;
}



////////////////////////////////////////////////////////////////////////////////
// A utility function to step through a defined number of steps between 1 and 0.
// This is used for adjusting flap position by steps.
//
double MainWindow::step(double *number, int numsteps, int direction)
{
    if (direction > 0){
        if (*number > 0.99999) return *number;
        *number = (*number) + 1.0/numsteps;
        if (*number > 1.0) *number = 1.0;
    } else if (direction < 0){
        if (*number < 0.00001) return *number;
        *number = (*number) - 1.0/numsteps;
        if (*number < 0.0) *number = 0.0;
    }
    return *number;
}



////////////////////////////////////////////////////////////////////////////////
// This displays error messages associated with the JSBSim standalone process.
//
void MainWindow::processError(QProcess::ProcessError err) {
    switch(err)
    {
    case QProcess::FailedToStart:
        QMessageBox::information(0,"FailedToStart","FailedToStart");
        break;
    case QProcess::Crashed:
        QMessageBox::information(0,"Crashed","Crashed");
        break;
    case QProcess::Timedout:
        QMessageBox::information(0,"FailedToStart","FailedToStart");
        break;
    case QProcess::WriteError:
        QMessageBox::information(0,"Timedout","Timedout");
        break;
    case QProcess::ReadError:
        QMessageBox::information(0,"ReadError","ReadError");
        break;
    case QProcess::UnknownError:
        QMessageBox::information(0,"UnknownError","UnknownError");
        break;
    default:
        QMessageBox::information(0,"default","default");
        break;
    }
}



////////////////////////////////////////////////////////////////////////////////
// Starts JSBSim standalone as a separate process.
//
void MainWindow::StartJSBSim(void) {

    if (!JSBSim) return;

    if (JSBSim->state() != (QProcess::Running)) {

          JSBSim->start(JSBSimExecutable, JSBSimArgs);

          m_ui->console->appendPlainText("Starting " + JSBSimExecutable);
          m_ui->StopJSBSimButton->setChecked(false);

          QString AircraftDir = installPath + "/JSBSim/aircraft/";

          AircraftConfig = new QFile(AircraftDir + "737/737.xml");
          if(AircraftConfig->open(QIODevice::ReadWrite | QIODevice::Text)){
              m_ui->AircraftEditor->setPlainText(AircraftConfig->readAll());
              AircraftConfig->close();
          }

          InitFile = new QFile(AircraftDir + "737/reset00.xml");
          if(InitFile->open(QIODevice::ReadWrite | QIODevice::Text)){
              m_ui->InitFileEditor->setPlainText(InitFile->readAll());
              InitFile->close();
          }

          delete AircraftConfig;
          delete InitFile;
          m_ui->ConnectButton->setEnabled(true);
          m_ui->PauseJSBSimButton->setEnabled(true);
          m_ui->StopJSBSimButton->setEnabled(true);

    }
}


////////////////////////////////////////////////////////////////////////////////
// Reads the ini file settings.
//
void MainWindow::readSettings(void) {

    settings = new QSettings(iniPath, QSettings::IniFormat);
    m_ui->console->appendPlainText("Reading settings from: " + settings->fileName());

    JSBSimExecutable = settings->value("cmdline/executable").toString();
    JSBSimArgs << "--root=" + installPath + "/JSBSim";
    JSBSimArgs << settings->value("cmdline/realtime").toString();
    JSBSimArgs << settings->value("cmdline/aircraft").toString();
    JSBSimArgs << settings->value("cmdline/initfile").toString();
    QString arg1 = settings->value("cmdline/arg1").toString();
    QString arg2 = settings->value("cmdline/arg2").toString();
    QString arg3 = settings->value("cmdline/arg3").toString();
    QString arg4 = settings->value("cmdline/arg4").toString();
    QString arg5 = settings->value("cmdline/arg5").toString();
    QString arg6 = settings->value("cmdline/arg6").toString();

    if (arg1.length() > 0) JSBSimArgs << arg1;
    if (arg2.length() > 0) JSBSimArgs << arg2;
    if (arg3.length() > 0) JSBSimArgs << arg3;
    if (arg4.length() > 0) JSBSimArgs << arg4;
    if (arg5.length() > 0) JSBSimArgs << arg5;
    if (arg6.length() > 0) JSBSimArgs << arg6;

    inputPort =  settings->value("ports/input").toInt();
    outputPort = settings->value("ports/output").toInt();
    telnetPort = settings->value("ports/telnet").toInt();
    FGSocket->setPort(settings->value("ports/flightgear").toInt());

    ui_rate =     settings->value("rates/UI").toFloat();
    output_rate = settings->value("rates/output").toFloat();
    fg_rate =     settings->value("rates/FlightGear").toFloat();

    pitch_trim =      settings->value("aircraft/pitch-trim").toDouble();
    pitch_trim_rate = settings->value("aircraft/pitch-trim-rate").toFloat();

    joystick->setElevatorAxis( settings->value("joystick/elevator-axis").toInt() );
    joystick->setAileronAxis( settings->value("joystick/aileron-axis").toInt() );
    joystick->setRudderAxis( settings->value("joystick/rudder-axis").toInt() );
    joystick->setThrottleAxis( settings->value("joystick/throttle-axis").toInt() );
    joystick->setAileronTrimAxis( settings->value("joystick/aileron-trim-axis").toInt() );
    joystick->setElevatorTrimAxis( settings->value("joystick/elevator-trim-axis").toInt() );
    joystick->setDeadbands(
                   settings->value("joystick/axis-0-deadband").toFloat(),
                   settings->value("joystick/axis-1-deadband").toFloat(),
                   settings->value("joystick/axis-2-deadband").toFloat() );

    localURL  = QUrl(settings->value("viewer/localURL").toString());
    remoteURL = QUrl(settings->value("viewer/remoteURL").toString());
    useLocal  =      settings->value("viewer/use-local").toBool();

    nav->SetMagVar(settings->value("airport/magvar").toDouble());
    nav->SetILSRunwayEndPoints(settings->value("airport/ILS-runway-near-latitude").toDouble(),
                               settings->value("airport/ILS-runway-near-longitude").toDouble(),
                               settings->value("airport/ILS-runway-far-latitude").toDouble(),
                               settings->value("airport/ILS-runway-far-longitude").toDouble());
    nav->SetILSFrequency(settings->value("airport/ILS-frequency").toFloat());
    nav->SetILSCourseMag(settings->value("airport/ILS-course-mag").toDouble());
    nav->SetILSGS(settings->value("airport/ILS-GS").toDouble());
    nav->SetILSTDZE(settings->value("airport/ILS-TDZE").toDouble());
    nav->SetRwyLength(settings->value("airport/runway-length-ft").toDouble());

    delete settings;

}


////////////////////////////////////////////////////////////////////////////////
// Writes settings to the ini file.  This is not used yet.
//
void MainWindow::writeSettings(void) {

    settings = new QSettings(iniPath, QSettings::IniFormat);
    m_ui->console->appendPlainText("Writing settings to: " + settings->fileName());

    // do nothing, yet

    delete settings;

}


////////////////////////////////////////////////////////////////////////////////
// Connect/disconnect socket communication with JSBSim. Presently the input and
// output sockets are controlled together.
//
void MainWindow::on_ConnectButton_clicked()
{

    if (!connected) {
      // Set up UDP data input from JSBSim
      jsbsimDataSocket = new QUdpSocket(this);
      jsbsimDataSocket->bind(inputPort, QUdpSocket::ShareAddress);
      if (jsbsimDataSocket) {
          m_ui->console->appendPlainText("Established JSBSim input socket at port " +
                                         QString::number(inputPort));
      }

      // Set up UDP data output to JSBSim
      jsbsimControlsSocket = new QUdpSocket(this);
      jsbsimControlsSocket->bind(outputPort, QUdpSocket::ShareAddress);
      if (jsbsimControlsSocket) {
          m_ui->console->appendPlainText("Established JSBSim output socket at port " +
                                         QString::number(outputPort));
      }

      if (jsbsimDataSocket && jsbsimControlsSocket) connected = true;
      m_ui->ConnectButton->setChecked(true);
      m_ui->ConnectButton->setEnabled(false);

    } else {

        delete jsbsimDataSocket;
        delete jsbsimControlsSocket;
        jsbsimDataSocket = NULL;
        jsbsimControlsSocket = NULL;
        connected = false;
        m_ui->console->appendPlainText("Closed JSBSim input socket.");
        m_ui->console->appendPlainText("Closed JSBSim output socket.");
        m_ui->ConnectButton->setChecked(false);
    }
}


////////////////////////////////////////////////////////////////////////////////
// These are UI slots that connect UI widgets to commands to JSBSim.
//
void MainWindow::on_GearUpButton_clicked()
{
    gear_cmd = 0.0;   // command gear up
}

void MainWindow::on_GearDownButton_clicked()
{
    gear_cmd = 1.0;   // command gear down
}

void MainWindow::on_FlapsUpButton_clicked()
{
    step( &flap_cmd, 9, -1 );   // raise flaps one step
}

void MainWindow::on_FlapsDownButton_clicked()
{
    step( &flap_cmd, 9, 1 );    // lower flaps one step
}

void MainWindow::on_SBUpButton_clicked()
{
    speedbrakes_cmd = 1.0;      // extend speed brakes
}

void MainWindow::on_SBDownButton_clicked()
{
    speedbrakes_cmd = 0.0;      // retract speed brakes
}

// set a new payload weight
void MainWindow::on_payloadWeightEdit_returnPressed()
{
    payload_cmd = (m_ui->payloadWeightEdit->text()).toDouble();
}

void MainWindow::on_windDirEdit_returnPressed()
{
    wind_dir = (m_ui->windDirEdit->text()).toDouble();
}

void MainWindow::on_windSpeedEdit_returnPressed()
{
    wind_speed = (m_ui->windSpeedEdit->text()).toDouble();
}

// Set temperature deviation from standard
void MainWindow::on_temperatureEdit_returnPressed()
{
    deltaT = (m_ui->temperatureEdit->text()).toDouble();
}

// Send a Pause signal to JSBSim
void MainWindow::on_PauseJSBSimButton_clicked()
{
   toggle( &jsbsimPause );
   if (jsbsimPause) {
       m_ui->PauseJSBSimButton->setText("Resume");
   }  else {
       m_ui->PauseJSBSimButton->setText("Pause");
   }
}

// Send a Stop signal to JSBSim
void MainWindow::on_StopJSBSimButton_clicked()
{
    toggle( &jsbsimTerminate );              // terminate JSBSim
    m_ui->StartJSBSimButton->setChecked(false);
    m_ui->ConnectButton->setEnabled(true);
    m_ui->console->appendPlainText("Stop signal sent to JSBSim");
}

// Close this application.  Make sure JSBSim closes first.
void MainWindow::on_QuitButton_clicked()
{
    if (JSBSim->state() == QProcess::Running) {
        JSBSim->kill();
    } else {
        viewer->close();
        this->close();
    }
}


// Launch the default browser and show home help file
void MainWindow::on_helpButton_clicked()
{
    QDesktopServices::openUrl(QUrl("file:///" + installPath + "/docs/en/home.html"));
}

// Begin ILS calculations, show ILS displays on PFD
void MainWindow::on_ILSButton_clicked()
{
    nav->SetILSEngaged(m_ui->ILSButton->isChecked());
}

// set desired heading (degrees magnetic)
void MainWindow::on_spinBox_2_valueChanged(int arg1)
{
    ap->setDesiredHdg((double)arg1);
}

// toggle Flight Path Marker on/off
void MainWindow::on_FPMButton_toggled(bool checked)
{
    FPM_enabled = checked;
}

// set desired altitude (feet MSL)
void MainWindow::on_spinBox_3_valueChanged(int arg1)
{
    ap->setDesiredAlt((double)arg1);
}

// set desired vertical speed (feet per minute)
void MainWindow::on_spinBox_4_valueChanged(int arg1)
{
    ap->setDesiredVS((double)arg1);
}

// toggle flight director visibility
void MainWindow::on_FDButton_clicked(bool checked)
{
    FDvisible = checked;
}

// engage lateral mode = Heading
void MainWindow::on_HDGButton_clicked(bool checked)
{
    if (checked) {
        ap->setLMode(autopilot::Heading);
        m_ui->LOCButton->setChecked(false);
    }
}

// hold current altitude (feet MSL)
void MainWindow::on_AltHoldButton_clicked(bool checked)
{
    if (checked) {
        int alt = ((int)(altitude/100.0)) * 100;
        m_ui->spinBox_3->setValue(alt);
        ap->setDesiredAlt((double)alt);
        ap->setVMode(autopilot::AltHold);
        m_ui->VSButton->setChecked(false);
        m_ui->FLCHButton->setChecked(false);
        m_ui->GSButton->setChecked(false);
    }
}

// engage vertical mode = Vertical Speed (fpm)
void MainWindow::on_VSButton_clicked(bool checked)
{
    if (checked) {
        int vs = ((int)(climb_rate/100.0)) * 100;
        m_ui->spinBox_4->setValue(vs);
        ap->setDesiredVS((double)vs);
        ap->setVMode(autopilot::VertSpeed);
        m_ui->AltHoldButton->setChecked(false);
        m_ui->FLCHButton->setChecked(false);
        m_ui->GSButton->setChecked(false);
    }
}

// engage vertical mode = Speed (control speed with pitch)
void MainWindow::on_FLCHButton_clicked(bool checked)
{
    if (checked) {
        ap->setVMode(autopilot::Speed);
        m_ui->VSButton->setChecked(false);
        m_ui->AltHoldButton->setChecked(false);
        m_ui->GSButton->setChecked(false);
    }
}

// engage vertical mode = ILS glideslope
void MainWindow::on_GSButton_clicked(bool checked)
{
    if (checked) {
        ap->setVMode(autopilot::ILS);
        m_ui->VSButton->setChecked(false);
        m_ui->FLCHButton->setChecked(false);
        m_ui->AltHoldButton->setChecked(false);
    }
}

// start/stop sending data to FlightGear
void MainWindow::on_FGButton_clicked(bool checked)
{
    ConnectToFG = checked;
    if (checked) {
        m_ui->console->appendPlainText("Sending data to FlightGear.");
    }  else {
        m_ui->console->appendPlainText("Stopped sending data to FlightGear.");
    }
}

////////////////////////////////////////////////////////////////////////////////
// Display the Cesium viewer window
//
void MainWindow::on_ViewerButton_clicked()
{
    viewer->setWindowTitle("CesiumJS Viewer");
    if (useLocal) {
         viewer->load(localURL);
    } else {
         viewer->load(remoteURL);
    }
    viewer->show();
    m_ui->ViewerButton->setChecked(false);
}


////////////////////////////////////////////////////////////////////////////////
// Find the path name of your install directory.  This needed to find the path
// to the ini file.
//
QString MainWindow::getInstallPath(void)
{
    // First, get the current working directory
    QDir currentDir = QDir::current();
    QString DirNameStr = currentDir.dirName();

    if (DirNameStr == "build-qtjsbsim-Desktop-Debug") {
        // we are running within QtCreator
        currentDir.cdUp();
        return currentDir.absolutePath();
    }

    while (true){
        if (DirNameStr.contains("qtjsbsim", Qt::CaseInsensitive)) {
            return currentDir.absolutePath();
        }
        else {
            if (currentDir.cdUp()) {
                DirNameStr = currentDir.dirName();
            } 
            else {
                break;
            }
        } 
    }

    qWarning("Could not find the qtjsbsim directory!");
    return currentDir.absolutePath();
}


////////////////////////////////////////////////////////////////////////////////
// This function is called once after the first data from JSBSim arrives.  This
// is a good place to add a variable to watch in the debug table widget.  Syntax
// to add an item is:
//                      debug->addItem( "myvariable", &variable );
//
// The variable should be of type double and have global scope.
//
void MainWindow::setDebugWatch(void)
{
  debug->addItem( "ellipsoid radius (ft)", &rEllipse );
  debug->addItem( "radius to CG (ft)", &CGRadius);
  debug->addItem("LOC error (deg)", &intercept);
  debug->addItem("ILS distance (nm)", &dist);
  debug->addItem("GS error (deg)", &gserror);
}


////////////////////////////////////////////////////////////////////////////////
// This function is called by the main loop, and is a good place to do calc-
// ulations for variables that aren't otherwise updated.
//
void MainWindow::doDebugCalculations(void)
{
   // checking the expected ellipsoid radius for this geocentric latitude
   double DEG2RAD = 3.141592654 / 180.0;
   double M2FT = 3.28083989501312;
   double b = 6356752.3142 * M2FT;
   double e2 = 0.00669437999014;
   double clat = cos(latitude * DEG2RAD);
   rEllipse = b / sqrt( 1 - e2 * clat * clat );

   CGRadius = rEllipse + altTerrain + 3.62; // 3.62 for 737 model sitting height
}


////////////////////////////////////////////////////////////////////////////////
// This is still a work in progress
//
//
void MainWindow::UpdateAutopilot()
{
    double _dt = 1.0 / framerate;

    ap_elevator_cmd = ap->updateVMode(_dt, pitch, altitude);
    ap_aileron_cmd = ap->updateLMode(_dt, roll, nav->GetMagHdg());

    // Flight Director
    double Vscale = 0.076923077;  // 13 degrees pitch is full deflection
    double Hscale = 0.04;         // 25 degrees bank is full deflection
    if (FDvisible) {
        if (ap->getVMode() != autopilot::VNone) {
              m_ui->widgetPFD->setBarH(ap->getFDPitchCmd() * Vscale, true);
        }
        if (ap->getLMode() != autopilot::LNone) {
              m_ui->widgetPFD->setBarV(ap->getFDRollCmd() * Hscale, true);
        }
    } else {
        m_ui->widgetPFD->setBarH(0.0, false);
        m_ui->widgetPFD->setBarV(0.0, false);
    }

}


////////////////////////////////////////////////////////////////////////////////
// Send JSBSim data to Flightgear via a UDP socket.  The units are
// converted by the packet if needed, and byte order conversions are
// also done by the packet.
//
void MainWindow::sendFlightGearData(void)
{
   if (!ConnectToFG) return;
   Packet* packet = FGSocket->getPacket();
   packet->setLongitude(longitude);         // degrees
   packet->setLatitude(latitude);           // degrees
   packet->setAltitude(altitude);           // feet
   packet->setRoll(roll);                   // degrees
   packet->setPitch(pitch);                 // degrees
   packet->setTrueHdg(yaw);                 // degrees
   packet->setVelocity(airspeed);           // knots calibrated
   packet->setClimbRate(climb_rate);        // feet/minute
   packet->setAltAGL(altAGL);               // feet
   packet->setBeta(beta);                   // deg
   packet->setP(P);                         // radians per second
   packet->setQ(Q);                         // radians per second
   packet->setR(R);                         // radians per second
   packet->setVN(vN);                       // feet per second
   packet->setVE(vE);                       // feet per second
   packet->setVD(vD);                       // feet per second
   packet->setvU(vU);                       // feet per second
   packet->setvV(vV);                       // feet per second
   packet->setvW(vW);                       // feet per second

   FGSocket->Send();
}


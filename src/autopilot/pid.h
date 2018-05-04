////////////////////////////////////////////////////////////////////////////////
//  Thanks to Bradley Snyder for this implementation of a PID controller
//  https://github.com/bradley219
//
//  Added to QTJSBSim project, 2018
////////////////////////////////////////////////////////////////////////////////

#ifndef _PID_H_
#define _PID_H_

////////////////////////////////////////////////////////////////////////////////

class PIDImpl;
class PID
{
    public:
        // Kp -  proportional gain
        // Ki -  Integral gain
        // Kd -  derivative gain
        // dt -  loop interval time
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and current process value
        double calculate( double dt, double setpoint, double pv );
        ~PID();

    private:
        PIDImpl *pimpl;
};

#endif


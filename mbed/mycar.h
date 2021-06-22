#ifndef MYCAR_H
#define MYCAR_H

#include "bbcar.h"
#include "mbed_rpc.h"

class mycar : public BBCar {
public:
    mycar( PwmOut &pin_servo0, PwmOut &pin_servo1, Ticker &servo_ticker );

    void stopCalib();

    void goByCm(double cm);

    void turnCalib(double speed, double factor);

    // left/right -> R = +/-. R: outer wheel radius, should > 11
    void turnRadius(double speed, double radius);

    void turnDeg(double speed, double radius, double degree);

    // spin left/right in place
    void spin(double speed);

    void spinDeg(double degree);

    void rPark(double d1, double d2);

    void faceTarget(volatile int &x_offset);

    void parkDistance(volatile float &dist, float goal);

    void lineFollow(double speed, volatile int &x_offset);
};

extern mycar car;

double cmToSec(double cm);
void rpc_stopCalib  (Arguments *in, Reply *out);
void rpc_go         (Arguments *in, Reply *out);
void rpc_turnRadius (Arguments *in, Reply *out);
void rpc_turnDeg    (Arguments *in, Reply *out);
void rpc_spinDeg    (Arguments *in, Reply *out);
void rpc_spin_2s    (Arguments *in, Reply *out);
void rpc_goByCm     (Arguments *in, Reply *out);
void rpc_rPark      (Arguments *in, Reply *out);

#endif
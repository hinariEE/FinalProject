#include "mycar.h"
using namespace std::chrono;

mycar::mycar( PwmOut &pin_servo0, PwmOut &pin_servo1, Ticker &servo_ticker ) : BBCar(pin_servo0, pin_servo1, servo_ticker) {}

void mycar::stopCalib(){
    servo0.set_factor(1);
    servo1.set_factor(1);
    servo0.set_speed_by_cm(0);
    servo1.set_speed_by_cm(0);
}

void mycar::goByCm(double cm){
    double sec = cmToSec(cm);
    auto ms = duration_cast<milliseconds>(duration<double>(sec));
    if(cm >= 0)
        car.goStraightCalib(10);
    else
        car.goStraightCalib(-10);
    ThisThread::sleep_for(ms);
    stopCalib();
}

void mycar::turnCalib(double speed, double factor){
    if(factor>0){
        servo0.set_factor(factor);
        servo1.set_factor(1);
    }
    else if(factor<0){
        servo0.set_factor(1);
        servo1.set_factor(-factor);
    }
    servo0.set_speed_by_cm(speed);
    servo1.set_speed_by_cm(-speed);
}

void mycar::turnRadius(double speed, double radius){
    // left/right -> R = +/-. R: outer wheel radius, should > 11
    // R = 11 / (1 - factor) (cm)
    double factor = 1.0 - 11.0 / abs(radius);
    if(radius < 0)
        factor = -factor;
    turnCalib(speed, factor);
}

void mycar::turnDeg(double speed, double radius, double degree){
    // left/right -> R = +/-. R: outer wheel radius, should > 11
    double path = 6.2832 * abs(radius) * abs(degree) / 360.0;
    double sec = path / speed + 0.3; // tuning based on trial-and-error
    sec = cmToSec(path);
    //if(speed < 0)
    //    sec = sec * 18.0 / 17.0 - 0.119;
    auto ms = duration_cast<milliseconds>(duration<double>(sec));
    if(degree < 0){
        speed = -speed;
    }
    turnRadius(speed, radius);
    ThisThread::sleep_for(ms);
    stopCalib();
}

void mycar::spin(double speed){
    servo0.set_factor(1);
    servo1.set_factor(1);
    servo0.set_speed_by_cm(-speed);
    servo1.set_speed_by_cm(-speed);
}

void mycar::spinDeg(double degree){
    double speed = 5.0;
    double path = degree / 360.0f * 3.141593f * 11.0f;

    // tuning based on trial-and-error
    double sec = abs(path) / speed + 0.3;
    if(path > 0)
        sec = sec * 18.0 / 17.0;
    auto ms = duration_cast<milliseconds>(duration<double>(sec));
    if(path >= 0){
        spin(speed);
    }
    else{
        spin(-speed);
    }
    ThisThread::sleep_for(ms);
    stopCalib();
}

/*void mycar::spinDeg(double degree){
    double speed = 10.0;
    double path = degree / 360.0f * 3.141593f * 11.0f;

    // tuning based on trial-and-error
    double sec = cmToSec(path) * 1.3;
    auto ms = duration_cast<milliseconds>(duration<double>(sec));
    if(path >= 0){
        spin(speed);
    }
    else{
        spin(-speed);
    }
    ThisThread::sleep_for(ms);
    stopCalib();
}*/

void mycar::rPark(double d1, double d2){
               // wheel-car spacing: 1.5 cm
    d2 += 8.0; // edge-wheel: 8 cm, slot width: 4 cm wider than the car
    double goDist = d1 - d2;
    goByCm(goDist);
    ThisThread::sleep_for(1500ms);
    double R = d1 + 11.0;           // turning radius: R; car width: 11 cm
    double factor = 1.0 - 11.0 / R; // R = 11 / (1 - factor) (cm);
    if(factor == 0.0)
        factor = 1e-6;
    double path = 6.28 * R / 4.0;   // 3.14 * 2 => 6.28, a quarter circle => 1 / 4
    double sec = cmToSec(path);
    auto ms = duration_cast<milliseconds>(duration<double>(sec));
    car.turnCalib(-10, -factor);
    ThisThread::sleep_for(ms);
    stopCalib();
    ThisThread::sleep_for(1500ms);
    goByCm(-5);
}

void mycar::faceTarget(volatile int &x_offset){
    Timer timer;
    bool converging = false;

    while(1){
        car.spin(-x_offset / 9.0);
        if(!converging){
            if(abs(x_offset) < 7){
                timer.start();
                converging = true;
            }
        }
        else{
            if(timer.elapsed_time().count() > 1000000){
                timer.stop();
                timer.reset();
                break;
            }
            if(abs(x_offset) >= 7){
                timer.stop();
                timer.reset();
                converging = false;
            }
        }
    }
    car.stopCalib();
}

void mycar::parkDistance(volatile float &dist, float goal){
    Timer timer;
    bool converging = false;

    while(1){
        car.goStraightCalib(0.3f * (dist - goal));  // Kp
        if(!converging){
            if(abs(dist - goal) < 3.0f){  // tolerance: 3 cm
                timer.start();
                converging = true;
            }
        }
        else{
            if(timer.elapsed_time().count() > 1000000){
                timer.stop();
                timer.reset();
                break;
            }
            if(abs(dist - goal) >= 3.0f){
                timer.stop();
                timer.reset();
                converging = false;
            }
        }
    }
    car.stopCalib();
}

void mycar::lineFollow(double speed, volatile int &x_offset){
    while(1){
        int error = -5 * x_offset;
        if(error >= 180)
            error = 179;
        if(error <= -180)
            error = -179;

        double factor;
        if(error >= 0){
            factor = 1.0 / (1.0 + error / 100.0);
        }
        else{
            factor = -1.0 / (1.0 - error / 100.0);
        }
        car.turnCalib(6, factor);
    }
}

double cmToSec(double cm){
    if(cm < 0)
        cm = -cm;
    return cm / 9.9 + 0.3;
}

RPCFunction rpcStopCalib (&rpc_stopCalib,  "stopCalib");
RPCFunction rpcGo        (&rpc_go,         "go");
RPCFunction rpcGoByCm    (&rpc_goByCm,     "goByCm");
RPCFunction rpcTurnRadius(&rpc_turnRadius, "turnRadius");
RPCFunction rpcTurnDeg   (&rpc_turnDeg,    "turnDeg");
RPCFunction rpcSpinDeg   (&rpc_spinDeg,    "spinDeg");
RPCFunction rpcSpin2s    (&rpc_spin_2s,    "spin2s");
RPCFunction rpcRPark     (&rpc_rPark,      "rPark");

void rpc_stopCalib(Arguments *in, Reply *out){
    car.stopCalib();
}

void rpc_go(Arguments *in, Reply *out){
    double speed = in->getArg<double>();
    car.goStraightCalib(speed);
}

void rpc_goByCm(Arguments *in, Reply *out){
    double cm = in->getArg<double>();
    car.goByCm(cm);
}

void rpc_turnRadius(Arguments *in, Reply *out){
    double speed = in->getArg<double>();
    double radius = in->getArg<double>();
    car.turnRadius(speed, radius);
}

void rpc_turnDeg(Arguments *in, Reply *out){
    double speed = in->getArg<double>();
    double radius = in->getArg<double>();
    double degree = in->getArg<double>();
    car.turnDeg(speed, radius, degree);
}

void rpc_spinDeg(Arguments *in, Reply *out){
    double degree = in->getArg<double>();
    car.spinDeg(degree);
}

void rpc_spin_2s(Arguments *in, Reply *out){
    double speed = in->getArg<double>();
    car.spin(speed);
    ThisThread::sleep_for(2s);
    car.stopCalib();
}

void rpc_rPark(Arguments *in, Reply *out){
    double d1 = in->getArg<double>();
    double d2 = in->getArg<double>();
    car.rPark(d1, d2);
}
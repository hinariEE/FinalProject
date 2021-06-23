#include "mbed.h"
using namespace std::chrono;




#include "mycar.h"
Ticker servo_ticker;
PwmOut servoL(D12), servoR(D13);
mycar car(servoL, servoR, servo_ticker);




DigitalIn encoder(D10);
Ticker encoder_ticker;
volatile int steps;
volatile int last;
void encoder_control(){
    int value = encoder;
    if(!last && value) steps++;
    last = value;
}

void rpc_secVsDist(Arguments *in, Reply *out){
    double sec = in->getArg<double>();
    steps = 0;
    car.goStraightCalib(10.0);
    ThisThread::sleep_for(duration_cast<milliseconds>(duration<double>(sec)));
    car.stopCalib();
    ThisThread::sleep_for(2500ms);
    char buf[16];
    sprintf(buf, "%7.3f", steps * 6.5f * 3.141593f / 32.0f);
    out->putData(buf);
}

RPCFunction rpcGoBySec(&rpc_secVsDist, "secVsDist");




#include "myXbee.h"
BufferedSerial xbee(D1, D0);
DigitalOut RPCstandbyLED(LED2); // LED1 is D13, having used by servoR




#include "mysensor.h"
BufferedSerial openmv(A1, A0); // -> (P5, P4) on the OpenMV

DigitalInOut ping(D2);
Ticker ping_ticker;
Timer ping_timer;
volatile float pingDist;

void pingMeas(){
    ping.output();
    ping = 0; wait_us(200);
    ping = 1; wait_us(5);
    ping = 0; wait_us(5);

    ping.input();
    while(ping.read() == 0);
    ping_timer.start();
    while(ping.read() == 1);
    long ToF = ping_timer.elapsed_time().count();  // 64-bit signed microsecond
    pingDist = ToF * 0.0177004f;                   // distance in cm
    ping_timer.stop();
    ping_timer.reset();
}

void replySensor(){
    char buf[64];
    while(1){
        if(fresh_line){
            int n = sprintf(buf, "[LINE] x: %3d| theta: %3d deg ", line_x, line_theta);
            xbee.write(buf, n);  // n = 30
        }
        else{
            xbee.write("[LINE] x: NaN| theta: NaN deg ", 30);
        }
        if(fresh_apriltag){
            int n = sprintf(buf, "[AprT] Tx: %3d| Ry: %3d deg| dist: %5.1f cm\r", Tx, Ry, pingDist);
            xbee.write(buf, n); // n = 44
        }
        else{
            int n = sprintf(buf, "[AprT] Tx: NaN| Ry: NaN deg| dist: %5.1f cm\r", pingDist);
            xbee.write(buf, n);
        }
        ThisThread::sleep_for(200ms);
    }
}

void rpc_replySensor(Arguments *in, Reply *out){
    replySensor();
}

RPCFunction rpcReplySensor(&rpc_replySensor, "replySensor");




volatile double action;        // correction intensity

void integrator(volatile float *F){
    *F += line_x * 0.01f;  // called per 10 ms
}

void lineFollow(double speed, volatile bool *running){
    volatile float error_I = 0.0f;
    //Ticker integ_ticker;
    //integ_ticker.attach(callback(integrator, &error_I), 10ms);
    //while(1){  // for test
    while(fresh_line && *running){
        double Kp = 0.06;
        double Ki = 0.0;
        //if(error_I >= 50.0f) error_I = 50.0f;
        //else if(error_I <= -50.0f) error_I = -50.0f;
        if(line_x < 20){
            action = -(Kp * (line_x + 0.02 * line_theta) + Ki * error_I);
        }else{
            action = -(Kp * line_x * 0.8);
        }

        double factor;
        if(action >= 0){
            factor = 1.0 / (1.0 + action);
        }
        else{
            factor = -1.0 / (1.0 - action);
        }
        car.turnCalib(speed, factor);
    }
    car.stopCalib();
    //integ_ticker.detach();
}

void posCalib(){
    if(fresh_apriltag){
        car.faceTarget(Tx);

        double dist = pingDist;
        double headingAngle_1 = (double)Ry;
        double dx = dist * sin(headingAngle_1 / 180.0 * 3.141593);
        double dy = dist * cos(headingAngle_1 / 180.0 * 3.141593) - 20.0;
        double headingAngle_2 = atan(dx / dy) / 3.141593 * 180.0;
        double path = sqrt(dx * dx + dy * dy);
        ThisThread::sleep_for(100ms);
        car.spinDeg(headingAngle_1 - headingAngle_2);
        ThisThread::sleep_for(500ms);
        car.goByCm(path);
        ThisThread::sleep_for(1000ms);
        car.spinDeg(headingAngle_2);
        ThisThread::sleep_for(1000ms);
    }
    if(fresh_apriltag){
        car.faceTarget(Tx);
        //car.parkDistance(pingDist, 22.0f);
    }
}

void rpc_hw4_2(Arguments *in, Reply *out){
    volatile bool running = true;
    lineFollow(6.0, &running);
}

void rpc_hw4_3(Arguments *in, Reply *out){
    posCalib();
}

void finalProject(double speed){
    Thread t_car;
    EventQueue q_car;
    t_car.start(callback(&q_car, &EventQueue::dispatch_forever));
    volatile bool running;
    Timer timer;
    bool straight = true;
    while(1){
        running = true;
        q_car.call(lineFollow, speed, &running);
        ThisThread::sleep_for(600ms);
        while(abs(action) < 0.7);
        xbee.write("\r\nFinish straight line.\r\n", 27);
        while(!fresh_apriltag);
        ThisThread::sleep_for(500ms);
        running = false;
        xbee.write("\r\nFinish half circle.\r\n", 25);
        ThisThread::sleep_for(1000ms);
        posCalib();
        ThisThread::sleep_for(100ms);
        car.spinDeg(-180.0);
        xbee.write("\r\nFinish position calibration.\r\n", 34);
        ThisThread::sleep_for(1000ms);
    }
}

void rpc_finalProject(Arguments *in, Reply *out){
    double speed = in->getArg<double>();
    finalProject(speed);
}

RPCFunction myrpc_hw4_2(&rpc_hw4_2, "hw4_2");
RPCFunction myrpc_hw4_3(&rpc_hw4_3, "hw4_3");
RPCFunction myrpc_finalProject(&rpc_finalProject, "final");




EventQueue queue;
InterruptIn button(USER_BUTTON);

int main()
{
    double pwm_table0[] = {-150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};
    double speed_table0[] = {-15.626, -15.628, -15.309, -14.990, -14.990, -14.352, -13.714, -12.758, -12.119, -10.206, -8.292, -6.060, -3.827, -1.834, -0.000, 0.638, 2.631, 4.943, 7.017, 9.249, 10.844, 12.119, 13.395, 14.033, 14.671, 14.990, 15.309, 15.309, 15.628, 15.628, 15.628};
    //double speed_table0[] = {-16.264, -15.947, -15.947, -15.947, -15.628, -14.990, -14.352, -13.395, -12.438, -10.525, -8.611, -6.379, -4.146, -1.834, -0.000, 0.558, 2.551, 4.943, 7.017, 9.249, 10.844, 12.438, 13.714, 14.352, 14.990, 15.628, 15.947, 15.947, 16.266, 16.266, 16.266};
    double speed_table1[] = {-15.626, -15.628, -15.628, -14.990, -15.309, -14.671, -14.352, -13.714, -12.757, -11.163, -9.887, -7.335, -5.422, -3.109, -0.638, 0.000, 2.073, 4.225, 6.379, 8.292, 10.525, 12.119, 13.395, 13.714, 14.352, 14.990, 14.990, 15.309, 15.309, 15.628, 15.628};
    //double speed_table1[] = {-16.264, -16.266, -16.265, -15.947, -15.627, -15.309, -14.671, -14.033, -13.076, -11.481, -9.568, -7.335, -5.103, -2.870, -0.717, 0.000, 2.392, 4.624, 7.016, 9.249, 10.843, 12.438, 13.714, 14.671, 15.309, 15.627, 15.946, 16.266, 16.265, 16.266, 16.584};
    car.setCalibTable(31, pwm_table0, speed_table0, 31, pwm_table0, speed_table1);
    car.stopCalib();

    //encoder_ticker.attach(&encoder_control, 10ms);

    ping_ticker.attach(&pingMeas, 500ms);

    xbee_init();

    openmv.set_baud(9600);
    Thread t_openmv;
    Thread t_report;
    t_openmv.start(openmvReader);
    t_report.start(replySensor);
    //button.rise(queue.event(posCalib));

    button.rise(queue.event(finalProject, 8));
    queue.dispatch_forever();
}
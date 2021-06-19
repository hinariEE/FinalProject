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




static volatile bool running;  // for passing signal into `lineFollow()`

void integrator(volatile float *F){
    *F += line_x * 0.01f;  // called per 10 ms
}

void lineFollow(double speed){
    volatile float error_I = 0.0f;
    Ticker integ_ticker;
    integ_ticker.attach(callback(integrator, &error_I), 10ms);
    while(fresh_line && running){
        int Kp = 5;
        int Ki = 1;
        int action = -(Kp * line_x + Ki * (int)error_I);
        if(action >= 180)
            action = 179;
        if(action <= -180)
            action = -179;

        double factor;
        if(action >= 0){
            factor = 1.0 / (1.0 + action / 100.0);
        }
        else{
            factor = -1.0 / (1.0 - action / 100.0);
        }
        car.turnCalib(speed, factor);
    }
    car.stopCalib();
    integ_ticker.detach();
}

void lineFollow(double *speed){
    lineFollow(*speed);
}

void posCalib(){
    if(fresh_apriltag){
        car.faceTarget(Tx);

        double headingAngle_1 = (double)Ry;
        double headingAngle_2 = 2.5 * headingAngle_1;
        if(headingAngle_2 >= 90.0)
            headingAngle_2 = 80.0;
        else if(headingAngle_2 <= -90.0)
            headingAngle_2 = -80.0;
        double goal_x = pingDist * sin(headingAngle_1 / 180.0 * 3.14);
        double path = goal_x / sin(headingAngle_2 / 180.0 * 3.14);
        car.spinDeg(headingAngle_1 - headingAngle_2);
        ThisThread::sleep_for(500ms);
        car.goByCm(path);
        ThisThread::sleep_for(500ms);
        car.spinDeg(headingAngle_2);
    }
    if(fresh_apriltag){
        car.faceTarget(Tx);
    }
}

void rpc_hw4_2(Arguments *in, Reply *out){
    running = true;
    lineFollow(6.0);
}

void rpc_hw4_3(Arguments *in, Reply *out){
    posCalib();
}

RPCFunction myrpc_hw4_2(&rpc_hw4_2, "hw4_2");
RPCFunction myrpc_hw4_3(&rpc_hw4_3, "hw4_3");




int main()
{
    double pwm_table0[] = {-150, -140, -130, -120, -110, -100, -90, -80, -70, -60, -50, -40, -30, -20, -10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150};
    double speed_table0[] = {-15.626, -15.628, -15.309, -14.990, -14.990, -14.352, -13.714, -12.758, -12.119, -10.206, -8.292, -6.060, -3.827, -1.834, -0.000, 0.638, 2.631, 4.943, 7.017, 9.249, 10.844, 12.119, 13.395, 14.033, 14.671, 14.990, 15.309, 15.309, 15.628, 15.628, 15.628};
    //double speed_table0[] = {-16.264, -15.947, -15.947, -15.947, -15.628, -14.990, -14.352, -13.395, -12.438, -10.525, -8.611, -6.379, -4.146, -1.834, -0.000, 0.558, 2.551, 4.943, 7.017, 9.249, 10.844, 12.438, 13.714, 14.352, 14.990, 15.628, 15.947, 15.947, 16.266, 16.266, 16.266};
    double speed_table1[] = {-15.626, -15.628, -15.628, -14.990, -15.309, -14.671, -14.352, -13.714, -12.757, -11.163, -9.887, -7.335, -5.422, -3.109, -0.638, 0.000, 2.073, 4.225, 6.379, 8.292, 10.525, 12.119, 13.395, 13.714, 14.352, 14.990, 14.990, 15.309, 15.309, 15.628, 15.628};
    //double speed_table1[] = {-16.264, -16.266, -16.265, -15.947, -15.627, -15.309, -14.671, -14.033, -13.076, -11.481, -9.568, -7.335, -5.103, -2.870, -0.717, 0.000, 2.392, 4.624, 7.016, 9.249, 10.843, 12.438, 13.714, 14.671, 15.309, 15.627, 15.946, 16.266, 16.265, 16.266, 16.584};
    car.setCalibTable(31, pwm_table0, speed_table0, 31, pwm_table0, speed_table1);
    car.stopCalib();

    encoder_ticker.attach(&encoder_control, 10ms);

    ping_ticker.attach(&pingMeas, 500ms);

    xbee_init();

    openmv.set_baud(9600);
    Thread t_openmv;
    Thread t_report;
    t_openmv.start(openmvReader);
    t_report.start(replySensor);

    while(1){
        double speed = 6.0;
        running = true;
        Thread t_car;
        t_car.start(callback(lineFollow, &speed));
        while(!fresh_apriltag);
        running = false;
        t_car.join();
        car.stopCalib();
        ThisThread::sleep_for(500ms);
        posCalib();
        car.spinDeg(180.0);
    }
    /*
    BufferedSerial pc(USBTX, USBRX);
    FILE *devin = fdopen(&pc, "r");
    char buf[128], outbuf[256];
    while(1){
        unsigned i;
        for(i = 0; i < sizeof(buf) - 1; i++){
            char recv = fgetc(devin);
            if(recv == '\n'){
                break;
            }
            buf[i] = putchar(recv);
        }
        putchar('\n');
        buf[i++] = '\0';
        outbuf[0] = '\0';
        RPC::call(buf, outbuf);
        printf("%s\n", outbuf);
    }
    */
}
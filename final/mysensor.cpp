#include "mbed.h"
#include "mysensor.h"

volatile bool fresh_line = false;
volatile bool fresh_apriltag = false;
volatile int line_x;
volatile int line_theta;
volatile int Tx;
volatile int Ry;

static Timeout line_monitor;
static Timeout aprt_monitor;

static void setExpire(volatile bool *freshIndicator){
    *freshIndicator = false;
}

void openmvReader(){
    extern BufferedSerial openmv;
    char buf[64];
    unsigned i = 0;
    while(1){
        while(openmv.readable()){
            char c;
            openmv.read(&c, 1);
            if(c != '\r' && c != '\n' && i < sizeof(buf) - 1){
                buf[i++] = c;
            }
            else if(i == 0) // unnecessary: ignore redundant char in buffer
                ;
            else{
                buf[i++] = '\0';
                i = 0;
                // buf format:
                // "[LINE]  x: %d theta: %d"
                // "[AprT] Tx: %d    Ry: %d"
                if(buf[1] == 'L'){
                    line_monitor.detach();
                    line_monitor.attach(callback(setExpire, &fresh_line), 1s);
                    fresh_line = true;
                    sscanf(buf, "%*[^:]: %d %*[^:]: %d", &line_x, &line_theta);
                }
                else if(buf[1] == 'A'){
                    aprt_monitor.detach();
                    aprt_monitor.attach(callback(setExpire, &fresh_apriltag), 1s);
                    fresh_apriltag = true;
                    sscanf(buf, "%*[^:]: %d %*[^:]: %d", &Tx, &Ry);
                    Tx -= 7;
                }
            }
        }
    }
}
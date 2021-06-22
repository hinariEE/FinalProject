#include "mbed.h"
#include "mbed_rpc.h"
#include "myXbee.h"
#define TICKER_PERIOD 10ms

static Ticker RPCstandbyTicker;

static void RPCstandbyIndicating(){
    static int counter = 0;
    counter++;
    if(counter == 1)
        RPCstandbyLED = 0;
    if(counter >= 100){
        RPCstandbyLED = 1;
        counter = 0;
    }
}

void reply_messange(char *xbee_reply, const char *messange){
    xbee.read(&xbee_reply[0], 1);
    xbee.read(&xbee_reply[1], 1);
    xbee.read(&xbee_reply[2], 1);
    if(xbee_reply[0] == 'O' && xbee_reply[1] == 'K')
        printf("%s\n", messange);
    else{
        printf("Please reset\n");
    }
}

void check_addr(char *xbee_reply, const char *messenger){
    xbee.read(&xbee_reply[0], 1);
    xbee.read(&xbee_reply[1], 1);
    xbee.read(&xbee_reply[2], 1);
    xbee.read(&xbee_reply[3], 1);
    printf("%s = %c%c%c\n", messenger, xbee_reply[0], xbee_reply[1], xbee_reply[2]);
}

void xbee_rx(){
    static unsigned i = 0;
    static char buf[128], outbuf[128];
    while(xbee.readable()){
        char c;
        xbee.read(&c, 1);
        if(c != '\r' && c != '\n' && i < sizeof(buf) - 1){
            buf[i++] = c;
            xbee.write(&c, 1);
        }
        else{ // unnecessary: ignore redundant char in buffer
            RPCstandbyTicker.detach();
            RPCstandbyLED = 0;
            xbee.write("\r\n", 2);
            buf[i++] = '\0';
            outbuf[0] = '\0';
            printf("%s\n", buf);
            RPC::call(buf, outbuf);
            int len = 0;
            while(outbuf[len])
                len++;
            xbee.write(outbuf, len);
            xbee.write("\r\n", 2);
            printf("%s\n", outbuf);
            i = 0;
            RPCstandbyTicker.attach(&RPCstandbyIndicating, TICKER_PERIOD);
        }
    }
    ThisThread::sleep_for(100ms);
}

void xbee_init(){
    while(xbee.readable()) {
        char k;
        xbee.read(&k, 1);
    }
    char xbee_reply[4];
    xbee.set_baud(9600);
    /*xbee.write("+++", 3);
    reply_messange(xbee_reply, "enter AT mode");
    xbee.write("ATMY 0x220\r", 11);
    reply_messange(xbee_reply, "setting MY : 0x220");
    xbee.write("ATDL 0x120\r", 11);
    reply_messange(xbee_reply, "setting DL : 0x120");
    xbee.write("ATID 0x1\r", 9);
    reply_messange(xbee_reply, "setting PAN ID : 0x1");
    xbee.write("ATWR\r", 5);
    reply_messange(xbee_reply, "write config");
    xbee.write("ATMY\r", 5);
    check_addr(xbee_reply, "MY");
    xbee.write("ATDL\r", 5);
    check_addr(xbee_reply, "DL");
    xbee.write("ATCN\r", 5);
    reply_messange(xbee_reply, "exit AT mode");*/
    while(xbee.readable()) {
        char k;
        xbee.read(&k, 1);
        printf("clear\n");
    }
    printf("start\n");
    xbee.set_blocking(false);
    xbee.sigio(mbed_event_queue()->event(xbee_rx));
    RPCstandbyTicker.attach(&RPCstandbyIndicating, TICKER_PERIOD);
}
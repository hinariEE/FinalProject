#ifndef MYXBEE_H
#define MYXBEE_H

extern BufferedSerial xbee;
extern DigitalOut RPCstandbyLED;

void reply_messange(char *xbee_reply, char *messange);
void check_addr(char *xbee_reply, char *messenger);
void xbee_rx_interrupt();
void xbee_rx();
void xbee_init();

#endif
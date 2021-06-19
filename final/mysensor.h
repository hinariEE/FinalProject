#ifndef MYSENSOR_H
#define MYSENSOR_H

extern volatile bool fresh_line;
extern volatile bool fresh_apriltag;
extern volatile int line_x;
extern volatile int line_theta;
extern volatile int Tx;
extern volatile int Ry;

void openmvReader();

#endif
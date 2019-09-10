#ifndef _MB_SOKC
#define _MB_SOKC

typedef struct{
  int motionEndFlag;
  /* double time; */
  float vel;
  float omega;
} MbSendData;

typedef struct{
  int motionEndFlag;
  /* double time; */
  float th;
  float vx;
  float vy;
  float vth;
  float timeLast;
  float accX;
  float accY;
  float accZ;
  float gyrX;
  float gyrY;
  float gyrZ;
} MbRecvData;

int flagamcl;
#endif /* _MB_SOCK */

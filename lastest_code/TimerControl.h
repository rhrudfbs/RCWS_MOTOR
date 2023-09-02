#ifndef _TIMERCONTROL_
#define _DUECANLAYER_

struct Timer
{
  int  nCount;
  bool bStart;
  bool bExpired;
};

#define TIMERFREQUENCY                    100000    // Time = 1 sec / TIMERFREQUENCY

#define TIMERS                            1       // Number of timers

#endif

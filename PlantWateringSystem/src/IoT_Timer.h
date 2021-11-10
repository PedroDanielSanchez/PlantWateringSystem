#ifndef _IOT_TIMER_H_
#define _IOT_TIMER_H_

class IoT_Timer {

    unsigned int _timerStart, _timerTarget;
    
    public:
        void startTimer(unsigned int msec) {
        _timerStart = millis();
        _timerTarget = msec;
      }

      bool isTimerReady() {
        return ( (millis() - _timerStart) >= _timerTarget);
      }

};
#endif // _IOT_TIMER_H_

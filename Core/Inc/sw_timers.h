#ifndef SW_TIMERS_H_
#define SW_TIMERS_H_

#include <stdlib.h>
#include <stdint.h>

#define MAX_TIMERS  (20)

enum StateTimer {INVALID, IDLE, ACTIVE, RDY, DONE};
typedef struct{
  uint32_t time;          
  uint32_t period;       
  enum StateTimer state;    
  void (*pFunc)(void);       
}SoftTimer;

void SoftTimersInit(void);
void CreateTimer(SoftTimer *CurSoftTimer, uint8_t rewrite, unsigned int time, unsigned int period , enum StateTimer state, void (*pFunc)(void));
void CheckTimer(void);
enum StateTimer GetTimerState(SoftTimer *CurSoftTimer);
void ProcessTimer(void);



#endif //SW_TIMERS_H_
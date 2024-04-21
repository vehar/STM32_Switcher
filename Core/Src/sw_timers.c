#include "sw_timers.h"
#include <stdint.h>
//=======================SOFT_TIMERS_START++========================================

SoftTimer* SoftTimers[MAX_TIMERS];
uint16_t AmountTimers = 0;

//=================================================================================
void SoftTimersInit(void)
{
	memset(&SoftTimers, 0, sizeof(SoftTimer)*MAX_TIMERS);
	
	//TODO Fix error
    for(int i = 0; i < MAX_TIMERS; i++)
    {
     // SoftTimers[i]->state = DONE;
     // SoftTimers[i]->pFunc = NULL;
    }
}

void CreateTimer(SoftTimer *CurSoftTimer, uint8_t rewrite, unsigned int time, unsigned int period , enum StateTimer state, void (*pFunc)(void))
{
  for(int i = 0; i < AmountTimers; i++)
  {
    if(SoftTimers[i] == CurSoftTimer) //If timer is exist
    {
      if(CurSoftTimer->state == ACTIVE)
      {
        if(rewrite)
        {
         CurSoftTimer->time = time;
         CurSoftTimer->period = period;
         CurSoftTimer->state = state;
         CurSoftTimer->pFunc = pFunc;  
        }
      }
      else if(CurSoftTimer->state == DONE)
      {
        CurSoftTimer->time = time;
        CurSoftTimer->period = period;
        CurSoftTimer->state = state;
        CurSoftTimer->pFunc = pFunc;  
      }
      return;
    }
  }
  //If timer is new
  SoftTimers[AmountTimers] = CurSoftTimer;
  CurSoftTimer->time = time;
  CurSoftTimer->period = period;
  CurSoftTimer->state = state;
  CurSoftTimer->pFunc = pFunc;  
  AmountTimers++;
}

void CheckTimer(void)
{
  for(unsigned char i = 0; i < AmountTimers; i++)
  {
    if (SoftTimers[i]->state == ACTIVE)
    {
      if (SoftTimers[i]->time != 0)
      {
        SoftTimers[i]->time --;
      }
      else
      {
         SoftTimers[i]->state = RDY;
      }      
    }
  }
} 

enum StateTimer GetTimerState(SoftTimer *CurSoftTimer)
{
  for(unsigned char i = 0; i < AmountTimers; i++)
  {
    if(SoftTimers[i] == CurSoftTimer) //If timer is exist
    {
      return SoftTimers[i]->state;
    }
  }
  return INVALID;
} 

void ProcessTimer(void)
{
  void (*worker)(void) = NULL;
  
  for(unsigned char i = 0; i < AmountTimers; i++)
  {
    if (SoftTimers[i]->state == RDY)
    {
        worker = SoftTimers[i]->pFunc;
        if(worker != NULL) {worker();}
        
        if(SoftTimers[i]->period > 0)
        {
          SoftTimers[i]->time = SoftTimers[i]->period - 1;
          SoftTimers[i]->state = ACTIVE; //Restart timer
        }
        else
        {
          SoftTimers[i]->state = DONE;
        }
    }
  }
}

//=============================USAGE===============================================
//SoftTimer timer1;
//SoftTimer timer2;

/* //Usage Example
CreateTimer(&timer1, 0, 10000, 0, ACTIVE, Clk1); //10000
void Clk1(void){  timersRetVal = TimElapsed;}
*/

//=======================SOFT_TIMERS_STOP++========================================
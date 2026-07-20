#include "task.h"

/* 周期任务按“姿态、编码器、航向外环、速度内环”的顺序执行。 */
Task tasks[] = {
    {5, 0, Get_EulerAngles},
    {10, 0, Get_CalibratedAngles},
    {20, 0, encoder_update},
    {20, 0, Straight_Control_20ms},
    {20, 0, Motion_Handle},
};

void Scheduler_Run(void) {

    uint32_t now = Get_Time();
    for(int i=0; i<sizeof(tasks)/sizeof(Task); i++) {
        if(now - tasks[i].last_call >= tasks[i].interval) {
            tasks[i].task();
            tasks[i].last_call = now;
        }
    }
}




#ifndef __SCHEDULER__
#define __SCHEDULER__

#define TASK1_INDEX 0
#define TASK2_INDEX 1
#define TASK3_INDEX 2

#define TASK1_SCHEDULE (1 << TASK1_INDEX)
#define TASK2_SCHEDULE (1 << TASK2_INDEX)
#define TASK3_SCHEDULE (1 << TASK3_INDEX)

#define MAX_TASK_COUNT 32
typedef void (*SchedulerFunctionPointer)(void);

void Scheduler_Init(void);
void Scheduler_Start(void);
uint32_t Scheduler_AddTask(SchedulerFunctionPointer task, uint32_t index);
void Scheduler_SetTask(uint32_t task);
void Scheduler_ClearTask(uint32_t task);

#endif

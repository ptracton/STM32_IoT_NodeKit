
#include <stdio.h>
#include <stdint.h>
#include "task1.h"
#include "scheduler.h"

static uint32_t task1_count;

void Task1_Init(void){
	task1_count = 0;
	Scheduler_AddTask(&Task1, TASK1_INDEX);
}

void Task1(void){
	printf("Task 1 Count = %lu\n\r", task1_count++);
	Scheduler_SetTask(TASK2_SCHEDULE);
}

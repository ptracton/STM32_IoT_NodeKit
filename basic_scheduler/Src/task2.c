
#include <stdio.h>
#include <stdint.h>
#include "task2.h"
#include "scheduler.h"

static uint32_t task2_count;

void Task2_Init(void){
	task2_count = 0;
	Scheduler_AddTask(&Task2, TASK2_INDEX);
}

void Task2(void){
	printf("Task 2 Count = %lu\n\r", task2_count++);
	Scheduler_SetTask(TASK3_SCHEDULE);
}



#include <stdio.h>
#include <stdint.h>
#include "task3.h"
#include "scheduler.h"

static uint32_t task3_count;

void Task3_Init(void){
	task3_count = 0;
	Scheduler_AddTask(&Task3, TASK3_INDEX);
}

void Task3(void){
	printf("Task 3 Count = %lu\n\r", task3_count++);
}

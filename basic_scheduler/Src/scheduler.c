
#include <stdint.h>
#include "stm32l4xx.h"
#include "scheduler.h"


static uint32_t SchedulerTaskFlag;
static SchedulerFunctionPointer SchedulerTaskList[MAX_TASK_COUNT];

/**
  * @brief  Initialize the scheduler to have no tasks to run and empty the list of tasks
  *
  */
void Scheduler_Init(void){
	uint32_t i;
	SchedulerTaskFlag = 0;
	for (i=0; i<MAX_TASK_COUNT; i++){
		SchedulerTaskList[i] = NULL;
	}
	return;
}

/**
  * @brief  Add a task at the specified index
  * @note   index must be less than MAX_TASK_COUNT
  * @param  SchedulerFunctionPointer task
  * @param  uint32_t task
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: if task is installed
  *          - ERROR: Task failed to install due to index or invalid pointer
  */
uint32_t Scheduler_AddTask(SchedulerFunctionPointer task, uint32_t index){
	if (index > MAX_TASK_COUNT){
		return ERROR;
	}
	if (task == NULL){
		return ERROR;
	}

	SchedulerTaskList[index] = task;
	return SUCCESS;
}

/**
  * @brief  This is our while(1) loop that we never return from
  *
  */
void Scheduler_Start(void){
	uint32_t i;

	// Run forever
	while(1){

		// Only bother to check if something is scheduled
		if (SchedulerTaskFlag){

			// Check all the bits to see who is scheduled
			for (i=0; i<32; i++){

				//Is this one scheduler?
				if (SchedulerTaskFlag & (1<<i)){

					// Clear this task in case the task re-sets itself to run
					Scheduler_ClearTask(1<<i);

					// Make sure the function is not NULL
					if (SchedulerTaskList[i] != NULL)

						// Run scheduled task
						(SchedulerTaskList[i])();
				}
			}
		} else{
			// SLEEP
		}
	}

	return;
}
void Scheduler_SetTask(uint32_t task){
	SchedulerTaskFlag |= task;
	return;
}
void Scheduler_ClearTask(uint32_t task){
	SchedulerTaskFlag &= ~task;
	return;
}

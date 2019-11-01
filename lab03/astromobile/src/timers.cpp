#include "timers.h"

#include <stdio.h>        /* printf */

/******************************************************************************
 * Timer initialization routine
 * The function will initialize a timer given the parameters.
 *****************************************************************************/
int32_t init_timer(struct sigevent* event, struct itimerspec* itime,
		           timer_t* timer, const int32_t chanel_id,
				   const uint32_t period) {
	int32_t error;
	int32_t period_s;
	int32_t period_ns;

	/* Set event as pulse and attach to channel */
	event->sigev_notify = SIGEV_PULSE;
	event->sigev_coid   = ConnectAttach(ND_LOCAL_NODE, 0, chanel_id,
									    _NTO_SIDE_CHANNEL, 0);
	/* Set basic priority and set event code */
	event->sigev_priority = 0;
	event->sigev_code     = TASK_PULSE_CODE;

	/* Create timer and associate to event */
	error = timer_create(CLOCK_MONOTONIC, event, timer);
	if(0 != error) {
		printf("Error creating timer\n");
		return error;
	}

	/* Set the itime structure */
	period_s  = period / 1000;
	period_ns = (1000000 * period) - (period_s * 1000000000);
	itime->it_value.tv_sec = period_s;
	itime->it_value.tv_nsec = period_ns;
	itime->it_interval.tv_sec = period_s;
	itime->it_interval.tv_nsec = period_ns;

	/* Set the timer period */
	return timer_settime(*timer, 0, itime, NULL);
}

/******************************************************************************
 * Task routine
 * Just a dummy task saying hello to the user.
 *****************************************************************************/
void* task_routine(void* args) {

	struct timespec tp;
	sem_t*          sync_sem;
	uint32_t        task_id;
	uint32_t        starttime;
	uint32_t        elapsed_time;

	/* Get the arguments */
	sync_sem  = ((thread_args_t*)args)->semaphore;
	task_id   = ((thread_args_t*)args)->id;
	starttime = ((thread_args_t*)args)->starttime;

	/* Routine loop */
	while(1<2) {
		/* Wait for the pulse handler to release the semaphore */
		if(0 == sem_wait(sync_sem)) {
			/* Get the current time */
			if(0 == clock_gettime(CLOCK_REALTIME, &tp)) {
				elapsed_time = tp.tv_sec - starttime;
				printf("Hello from task %d at %d\n", task_id, elapsed_time);
			}
			else {
				/* Print error */
				printf("Task %d could not get time: %d\n", task_id, errno);
			}
		}
		else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
}

/******************************************************************************
 * Task pulse handler routine
 * Handles a pulse and release the semaphore.
 *****************************************************************************/
void* task_pulse_handler(void* args) {

	sem_t*      sync_sem;
	int32_t     rcvid;
	pulse_msg_t msg;
	int32_t     task_chid;

	/* Get the arguments */
	sync_sem  = ((thread_args_t*)args)->semaphore;
	task_chid = ((thread_args_t*)args)->chid;


	while(1) {
		/* Get the pulse message */
		rcvid = MsgReceive(task_chid, &msg, sizeof(pulse_msg_t), NULL);
		printf("test %d\n", task_chid);

		if (0 == rcvid) {
			if (TASK_PULSE_CODE == msg.pulse.code) {
				/* Release semaphore */
				if(0 != sem_post(sync_sem)) {
					/* Print error */
					printf("Could not post semaphore: %d\n", errno);
				}
			}
			else {
				/* Print error */
				printf("Unknown message received: %d\n", rcvid);
			}
		}
		else {
			/* Print error */
			printf("Message receive failed: %d (%d)\n", rcvid, errno);
		}
	}
}

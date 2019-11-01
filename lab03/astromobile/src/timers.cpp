#include "timers.h"

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

	while(1<2) {
		/* Get the pulse message */
		rcvid = MsgReceive(task_chid, &msg, sizeof(pulse_msg_t), NULL);
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

//int main(void) {
//	struct timespec tp;
//
//	/* Semaphores for pulse synchronization */
//	sem_t task0_sync;
//	sem_t task1_sync;
//
//	/* PThread structures */
//	pthread_t task0;
//	pthread_t task0_pulse_handler;
//	pthread_t task1;
//	pthread_t task1_pulse_handler;
//
//	/* Timers event structures */
//	struct sigevent   task0_event;
//	struct itimerspec task0_itime;
//	timer_t           task0_timer;
//	struct sigevent   task1_event;
//	struct itimerspec task1_itime;
//	timer_t           task1_timer;
//
//	/* Tasks arguments */
//	thread_args_t task0_args;
//	thread_args_t task1_args;
//
//	/* Get the start time */
//	if(0 != clock_gettime(CLOCK_REALTIME, &tp)) {
//		/* Print error */
//		printf("Could not get start time: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	/* Initialize the semaphore */
//	if(0 != sem_init(&task0_sync, 0, 0)) {
//		/* Print error */
//		printf("Could not get init semaphore: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != sem_init(&task1_sync, 0, 0)) {
//		/* Print error */
//		printf("Could not get init semaphore: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	/* Initialize the tasks arguments */
//	task0_args.id   = 0;
//	task0_args.semaphore = &task0_sync;
//	task0_args.starttime = tp.tv_sec;
//	task0_args.chid      = ChannelCreate(0);
//	if(-1 == task0_args.chid) {
//		/* Print error */
//		printf("Could not create channel: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	task1_args.id   = 1;
//	task1_args.semaphore = &task1_sync;
//	task1_args.starttime = tp.tv_sec;
//	task1_args.chid      = ChannelCreate(0);
//	if(-1 == task1_args.chid) {
//		/* Print error */
//		printf("Could not create channel: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	/* Create the different tasks and their associated pulse handlers */
//	if(0 != pthread_create(&task0, NULL, task_routine, &task0_args)) {
//		/* Print error */
//		printf("Could not create thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != pthread_create(&task0_pulse_handler, NULL,
//			               task_pulse_handler, &task0_args)) {
//		/* Print error */
//		printf("Could not create thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	if(0 != pthread_create(&task1, NULL, task_routine, &task1_args)) {
//		/* Print error */
//		printf("Could not create thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != pthread_create(&task1_pulse_handler, NULL,
//			               task_pulse_handler, &task1_args)) {
//		/* Print error */
//		printf("Could not create thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	/* Create timers */
//	if(0 != init_timer(&task0_event, &task0_itime, &task0_timer,
//			           task0_args.chid, 500)) {
//		/* Print error */
//		printf("Could not create timer: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != init_timer(&task1_event, &task1_itime, &task1_timer,
//			           task1_args.chid, 1000)) {
//		/* Print error */
//		printf("Could not create timer: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	/* Wait for the threads to finish */
//	if(0 != pthread_join(task0, NULL)) {
//		/* Print error */
//		printf("Could not wait for thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != pthread_join(task0_pulse_handler, NULL)) {
//		/* Print error */
//		printf("Could not wait for thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != pthread_join(task1, NULL)) {
//		/* Print error */
//		printf("Could not wait for thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//	if(0 != pthread_join(task1_pulse_handler, NULL)) {
//		/* Print error */
//		printf("Could not wait for thread: %d\n", errno);
//		return EXIT_FAILURE;
//	}
//
//	return EXIT_SUCCESS;
//}

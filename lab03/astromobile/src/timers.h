#ifndef TIMERS_H
#define TIMERS_H

#include <stdio.h>        /* printf */
#include <stdlib.h>       /* EXIT_SUCCESS */
#include <stdint.h>       /* int32_t uint32_t */
#include <pthread.h>      /* pthread_t pthread_create pthread_join */
#include <semaphore.h>    /* sem_t sem_init sem_wait sem_post */
#include <errno.h>        /* errno */
#include <signal.h>       /* struct sigevent */
#include <sys/neutrino.h> /* ChannelCreate ConnectAttach MsgReceive */
#include <sys/netmgr.h>   /* ND_LOCAL_NODE */
#include <time.h>         /* struct itimerspec struct timespec
                             timer_create tier_settime clock_gettime */

/* Pulse code definition */
#define TASK_PULSE_CODE _PULSE_CODE_MINAVAIL

/* Thread arguments structure */
typedef struct thread_arg {
	sem_t*   semaphore; /* Synchronization semaphore pointer */
	uint32_t id;        /* Task id */
	uint32_t starttime; /* Global start time */
	int32_t  chid;      /* Task channel id */
} thread_args_t;

/* Pulse dumy structure */
typedef union pulse_msg{
	struct _pulse pulse;
} pulse_msg_t;

void* task_pulse_handler(void* args);
int32_t init_timer(struct sigevent* event, struct itimerspec* itime,
		           timer_t* timer, const int32_t chanel_id,
				   const uint32_t period);
void* task_routine(void* args);

#endif /* TIMERS_H */

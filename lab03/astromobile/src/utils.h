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
#include <sys/sched_aps.h> /* sched_aps */
#include <string.h>        /* memset */
#include <unistd.h>        /* getpid */

#ifndef UTILS_H
#define UTILS_H

#define CONTINUOUS_BUDGET_PART 30
#define DISCREET_BUDGET_PART 30

/* Pulse code definition */
#define TASK_PULSE_CODE _PULSE_CODE_MINAVAIL

int create_partitions(sched_aps_create_parms* partitions,
					  sched_aps_parms* sched_param,
					  unsigned int* sched_pol);
int assign_partitions(sched_aps_join_parms* join_params,
		             const sched_aps_create_parms* partitions,
				     const pthread_t* threads,
					 int thread_num);

/* Thread arguments structure */
typedef struct thread_arg {
	sem_t*   semaphore; /* Synchronization semaphore pointer */
	uint32_t id;        /* Task id */
	uint32_t starttime; /* Global start time */
	int32_t  chid;      /* Task channel id */
	uint32_t period;    /* Task period */
} thread_args_t;

/* Pulse dumy structure */
typedef union pulse_msg{
	struct _pulse pulse;
} pulse_msg_t;

void* task_pulse_handler(void* args);
int32_t init_timer(struct sigevent* event, struct itimerspec* itime,
		           timer_t* timer, const int32_t chanel_id,
				   const uint32_t period);

#endif // UTILS_H

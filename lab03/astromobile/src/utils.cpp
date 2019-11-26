#include "utils.h"

/******************************************
* Fonction de création des partitions QNX *
******************************************/
int create_partitions(sched_aps_create_parms* partitions,
					  sched_aps_parms* sched_param,
					  unsigned int* sched_pol) {


	/* Set QNX partition to limit CPU budget */
	memset(&sched_param[0], 0, sizeof(sched_aps_parms));
	sched_param[0].windowsize_ms = -1;
	sched_param[0].scheduling_policy_flagsp = sched_pol;

	if (EOK != SchedCtl(SCHED_APS_SET_PARMS, &sched_param[0], sizeof(sched_aps_parms))) {
	   printf("Could not set scheduler limit CPU usage (%d).\n", errno);
	   return EXIT_FAILURE;
	}

	/* Create the partition for continuous tasks */
	memset(&partitions[0], 0, sizeof(sched_aps_create_parms));
	partitions[0].budget_percent     = CONTINUOUS_BUDGET_PART;
	partitions[0].max_budget_percent = CONTINUOUS_BUDGET_PART;
	partitions[0].name               = (char *)"CONTINUOUS_PART";

//	printf("%d\n", SchedCtl(SCHED_APS_CREATE_PARTITION, &partitions[0], sizeof(sched_aps_create_parms)));

	if(EOK != SchedCtl(SCHED_APS_CREATE_PARTITION, &partitions[0], sizeof(sched_aps_create_parms))) {
		printf("Could not create partition 0: %d\n", errno);
		perror("Error ");
		return EXIT_FAILURE;
	}

	/* Create the partition for the discreet tasks */
	memset(&partitions[1], 0, sizeof(sched_aps_create_parms));
	partitions[1].budget_percent     = DISCREET_BUDGET_PART;
	partitions[1].max_budget_percent = DISCREET_BUDGET_PART;
	partitions[1].name               = (char *)"DISCREET_PART";

	if(EOK != SchedCtl(SCHED_APS_CREATE_PARTITION, &partitions[1], sizeof(sched_aps_create_parms))) {
		printf("Could not create partition 1: %d\n", errno);
		return EXIT_FAILURE;
	}

	return EXIT_SUCCESS;
}

/****************************************
* Fonction d'assignation des partitions *
****************************************/
int assign_partitions(sched_aps_join_parms* join_params,
		             const sched_aps_create_parms* partitions,
				     const pthread_t* threads,
					 int thread_num ) {

	/* Assigner les taches continues à une la partition 0 (4 taches) */
	int i;
	for (i = 0; i < 4; ++i) {
		memset(&join_params[i], 0, sizeof(sched_aps_join_parms));
		join_params[i].id  = partitions[0].id;
		join_params[i].pid = getpid();
	}

	join_params[0].tid = threads[1]; // camera_worker
	join_params[1].tid = threads[2]; // battery_worker
	join_params[2].tid = threads[5]; // angle_worker
	join_params[3].tid = threads[6]; // speed_worker

	for (i = 0; i < 4; ++i) {
		if(EOK != SchedCtl(SCHED_APS_JOIN_PARTITION, &join_params[i], sizeof(sched_aps_join_parms))) {
			printf("Could not assign partition to thread\n");
			return EXIT_FAILURE;
		}
	}

	/* Assigner les autres taches (discretes) a la seconde partition */
	for (i = 4; i < thread_num; ++i) {
		memset(&join_params[i], 0, sizeof(sched_aps_join_parms));
		join_params[i].id  = partitions[1].id;
		join_params[i].pid = getpid();
	}

	join_params[4].tid  = threads[0];  // cameraControl_worker
	join_params[5].tid  = threads[3];  // battLow_worker
	join_params[6].tid  = threads[4];  // battHigh_worker
	join_params[7].tid  = threads[7];  // currPos_worker
	join_params[8].tid  = threads[8];  // navControl_worker
	join_params[9].tid  = threads[9];  // destControl_worker
	join_params[10].tid = threads[10]; // display_worker
	join_params[11].tid = threads[11]; // trace_worker

	for (i = 4; i < thread_num; ++i) {
		if(EOK != SchedCtl(SCHED_APS_JOIN_PARTITION, &join_params[i], sizeof(sched_aps_join_parms))) {
			printf("Could not assign partition to thread\n");
			return EXIT_FAILURE;
		}
	}

	return EXIT_SUCCESS;
}

/******************************************************************************
 * Timer initialization routine                                               *
 * The function will initialize a timer given the parameters.                 *
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

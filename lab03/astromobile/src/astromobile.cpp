#include <iostream>
#include <sched.h> 
#include <stdio.h> 
#include <stdlib.h> 
#include <string.h> 
#include <mqueue.h> 
#include <unistd.h> 
#include <pthread.h> 
#include <semaphore.h> 
#include <sys/types.h> 
#include <time.h>
#include <signal.h>
#include <sys/syspage.h>
#include <sys/stat.h>
#include <sys/netmgr.h>   /* ND_LOCAL_NODE */
#include <sys/neutrino.h> /* ChannelCreate ConnectAttach MsgReceive */
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <iomanip>  // needed to use manipulators with parameters (precision, width)
#include <cmath>
#include "genMap.h"
#include "astromobile.h"
#include "utils.h"

using namespace std;

PathMap pm;

struct physicsData myData;

enum carState state = GOTO_DEST; // état initial à GOTO_DEST
enum speeds speedState;

double orderedAngle;

coord_t nextStep;
coord_t dest;
coord_t stationPos;

bool takePx;
bool inCharge;
bool destReached;

bool lowBat;
bool highBat;

// nombre d'étapes atteintes entre deux destinations
unsigned int nb_stepReached; 
// nombre de destinations atteintes
unsigned int nb_destReached;

/* Semaphores for synchronization */
sem_t taskCamera_sync;
sem_t taskBattlow_sync;
sem_t taskBatthigh_sync;

int main() {

	init();

	return 0;
}

void init()
{
	// Valeurs initiales

	speedState       = VIT0;
	orderedAngle 	 = 0;
	myData.battLevel = 13;
	myData.currPos.x = 0;
	myData.currPos.y = 0;
	destReached      = true; // pour generer un destination initiale
	stationPos.x     = 0;
	stationPos.y     = 0;
	nb_stepReached   = 0;
	nb_destReached   = -1; // car on l'incremente a l'initialisation



	/* Initialize the semaphores */
	if(0 != sem_init(&taskCamera_sync, 0, 0)) {
		cout << "Could not init semaphore taskCamera_sync:" <<  errno << endl;
		return;
	}
	if(0 != sem_init(&taskBattlow_sync, 0, 0)) {
		printf("Could not init semaphore taskBattlow_sync: %d\n", errno);
		return;
	}
	if(0 != sem_init(&taskBatthigh_sync, 0, 0)) {
		printf("Could not init semaphore taskBatthigh_sync: %d\n", errno);
		return;
	}



	struct timespec tp;
	/* Get the start time */
	if(0 != clock_gettime(CLOCK_REALTIME, &tp)) {
		printf("Could not get start time: %d\n", errno);
		return;
	}


	sem_t * sem_syncs               = (sem_t *)calloc(THREAD_NUM, sizeof(sem_t)); // timer sync semaphores
	pthread_t * pulseHandlers       = (pthread_t *)calloc(THREAD_NUM, sizeof(pthread_t)); // pulse handlers
	struct sigevent * sigevents     = (struct sigevent *)calloc(THREAD_NUM, sizeof(struct sigevent));
	struct itimerspec * itimerspecs = (struct itimerspec *)calloc(THREAD_NUM, sizeof(struct itimerspec));
	timer_t * timers                = (timer_t *)calloc(THREAD_NUM, sizeof(timer_t));
	thread_args_t * task_args       = (thread_args_t *)calloc(THREAD_NUM, sizeof(thread_args_t));
	pthread_t * tid                 = (pthread_t *)calloc(THREAD_NUM, sizeof(pthread_t));
	pthread_attr_t attrib;
	struct sched_param mySchedParam;
	pthread_attr_init (&attrib);
	pthread_attr_setinheritsched (&attrib, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&attrib, SCHED_FIFO);

	// périodes
	int periods[THREAD_NUM] = {100,  // cameraControl_worker
							   PERIOD_CONT,  // camera_worker
							   PERIOD_CONT,  // battery_worker
							   100,  // battLow_worker
							   100,  // battHigh_worker
							   PERIOD_CONT,  // angle_worker
							   PERIOD_CONT,  // speed_worker
							   PERIOD_CONT,  // currPos_worker
							   100,  // navControl_worker
							   100,  // destControl_worker
							   1000}; // display_worker

	// priorités
	int prios[THREAD_NUM] = {4,  // cameraControl_worker
							 3,  // camera_worker
							 3,  // battery_worker
							 2,  // battLow_worker
							 2,  // battHigh_worker
							 3,  // angle_worker
							 3,  // speed_worker
							 3,  // currPos_worker
							 4,  // navControl_worker
							 4,  // destControl_worker
							 5}; // display_worker

	int i;
	for (i = 0; i < THREAD_NUM; ++i) {
		// Init semaphore
		if (sem_init(&sem_syncs[i], 0, 0) != 0) {
			printf("Could not get init semaphore %d: %d\n", i, errno);
			return;
		}
		// Init thread args
		task_args[i].id        = i;
		task_args[i].semaphore = &sem_syncs[i];
		task_args[i].starttime = tp.tv_sec;
		task_args[i].chid      = ChannelCreate(0);
		if(-1 == task_args[i].chid) {
			/* Print error */
			printf("Could not create channel: %d\n", errno);
			return;
		}
		mySchedParam.sched_priority = prios[i];
		pthread_attr_setschedparam(&attrib, &mySchedParam);
		if (pthread_create(&tid[i], &attrib, main_worker, (void *)&task_args[i]) < 0) {
			cout << "Thread " << i << " creation failed" << endl;
			return;
		}
		mySchedParam.sched_priority = 1; // maximum priority
		pthread_attr_setschedparam(&attrib, &mySchedParam);
		if(0 != pthread_create(&pulseHandlers[i], &attrib,
					           task_pulse_handler, (void *)&task_args[i])) {
			printf("Could not create pulse thread: %d\n", errno);
			return;
		}
		if(0 != init_timer(&sigevents[i], &itimerspecs[i], &timers[i],
						   task_args[i].chid, periods[i])) {
			printf("Could not create timer: %d\n", errno);
			return;
		}
	}

	sleep(60);
	pm.dumpImage("./map.bmp");

	// join des threads
	for (i = 0; i < THREAD_NUM; ++i) {
		if (pthread_join(tid[i], NULL) < 0)
			cout << "Join failed for thread " << i << endl;
		if (pthread_join(pulseHandlers[i], NULL) < 0)
			cout << "Join failed for pulse thread " << i << endl;
	}

	// Free the arrays
	free(sem_syncs);
	free(pulseHandlers);
	free(sigevents);
	free(itimerspecs);
	free(timers);
	free(task_args);
	free(tid);

	return;
}

void * main_worker(void * data) {

	int task_id = ((thread_args_t *)data)->id;

	switch(task_id) {
		case 0:  cameraControl_worker(data); break;
		case 1:  camera_worker(data);        break;
		case 2:  battery_worker(data);       break;
		case 3:  battLow_worker(data);       break;
		case 4:  battHigh_worker(data);      break;
		case 5:  angle_worker(data);         break;
		case 6:  speed_worker(data);         break;
		case 7:  currPos_worker(data);       break;
		case 8:  navControl_worker(data);    break;
		case 9:  destControl_worker(data);   break;
		case 10: display_worker(data);       break;
		default: break;
	}
	return NULL;

}

void cameraControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

    double delta, x, y;
	while(1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataCurrPos);
			x = myData.currPos.x;
			y = myData.currPos.y;
			pthread_mutex_unlock(&mutDataCurrPos);
			while (delta < 10) {
					pthread_mutex_lock(&mutDataCurrPos);
					delta = sqrt(pow((myData.currPos.x - x), 2) + pow((myData.currPos.y - y), 2));
					pthread_mutex_unlock(&mutDataCurrPos);
			}
			//release semaphore
			if(0 != sem_post(&taskCamera_sync)) {
				printf("Could not post semaphore: %d\n", errno);
			}
			delta = 0;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}

void camera_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	rgb_t image;
	while(1) {
		if (sem_wait(sync_sem) == 0) {
			/* Wait to release the semaphore */
			if(0 == sem_wait(&taskCamera_sync)) {
				pthread_mutex_lock(&mutDataCurrPos);
				image = pm.takePhoto(myData.currPos);
				pthread_mutex_unlock(&mutDataCurrPos);
			} else {
				printf("Task camera_worker could not get time: %d\n", errno);
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}

void battery_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	float speed_local, batt_local;

	while(1)	{
		if (sem_wait(sync_sem) == 0) {
			if (inCharge)	{
				pthread_mutex_lock(&mutDataBattLevel);
				myData.battLevel += CONST_CHARGE;
				pthread_mutex_unlock(&mutDataBattLevel);
			}
			pthread_mutex_lock(&mutDataSpeed);
			speed_local = myData.speed;
			pthread_mutex_unlock(&mutDataSpeed);
			pthread_mutex_lock(&mutDataBattLevel);
			myData.battLevel -= (float)(COEFF_DECHARGE * speed_local + CONST_DECHARGE) * (float)((float)PERIOD_CONT/1000);
			batt_local = myData.battLevel;
			pthread_mutex_unlock(&mutDataBattLevel);
			if (batt_local <= 10)	{
				if(0 != sem_post(&taskBattlow_sync)) {
						printf("Could not post semaphore: %d\n", errno);
					}
			}
			else if (batt_local >= 80)	{
				if(0 != sem_post(&taskBatthigh_sync)) {
						printf("Could not post semaphore: %d\n", errno);
					}
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

void battLow_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	//float bat;
	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			/*pthread_mutex_lock(&mutDataBattLevel);
			bat = myData.battLevel;
			pthread_mutex_unlock(&mutDataBattLevel);	*/
			if(0 == sem_wait(&taskBattlow_sync)) {
				lowBat  = true;
				highBat = false;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}	

void battHigh_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	//float bat;
	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			/*pthread_mutex_lock(&mutDataBattLevel);
			bat = myData.battLevel;
			pthread_mutex_unlock(&mutDataBattLevel);	*/
			if(0 == sem_wait(&taskBatthigh_sync)) {
				highBat = true;
				lowBat  = false;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}

/*************************************************************
* Thread representant la partie continue qui modifie l'angle *
* Pas forcement necessaire dans le programme mais c'est une  *
* representation plus correcte du reel                       *
*************************************************************/
void angle_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataAngle);
			myData.angle = orderedAngle;
			pthread_mutex_unlock(&mutDataAngle);
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
}

/*************************************************************
* Thread representant la partie continue qui modifie la      *
* vitesse                                                   *
*************************************************************/
void speed_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			switch (speedState) {
				case VIT0:
					pthread_mutex_lock(&mutDataSpeed);
					myData.speed = 0;
					pthread_mutex_unlock(&mutDataSpeed);
					break;
				case VIT30:
					pthread_mutex_lock(&mutDataSpeed);
					myData.speed = 30;
					pthread_mutex_unlock(&mutDataSpeed);
					break;
				case VIT50:
					pthread_mutex_lock(&mutDataSpeed);
					myData.speed = 50;
					pthread_mutex_unlock(&mutDataSpeed);
					break;
				default:
					pthread_mutex_lock(&mutDataSpeed);
					myData.speed = 0;
					pthread_mutex_unlock(&mutDataSpeed);
					break;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
}

/******************************************************
* currPos_worker : met à jour la position en fonction *
* de la vitesse et de la l'angle					  *
******************************************************/
void currPos_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	float deltaX, deltaY, dist;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataSpeed);
			dist = SIMU_ACCEL * 1000 * myData.speed * PERIOD_CONT / (1000 * 3600) ;
			pthread_mutex_unlock(&mutDataSpeed);
			pthread_mutex_lock(&mutDataAngle);
			deltaX = dist * cos(myData.angle * PI / 180);
			deltaY = dist * sin(myData.angle * PI / 180);
			pthread_mutex_unlock(&mutDataAngle);
			pthread_mutex_lock(&mutDataCurrPos);
			myData.currPos.x += deltaX;
			myData.currPos.y += deltaY;
			pthread_mutex_unlock(&mutDataCurrPos);
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}	

bool nextStepReached(coord_t currPos, coord_t nextStep) {
	return (sqrt(pow((nextStep.x - currPos.x),2) + pow((nextStep.y - currPos.y),2)) <= 10);
}

double getAngle(coord_t currPos, coord_t nextPos) {
    if (nextPos.y - currPos.y > 0) {
        return 90 - (atan((nextPos.x - currPos.x)/(nextPos.y - currPos.y)) * (180/PI));
    } else if (nextPos.y - currPos.y < 0) {
        return 90 - (atan((nextPos.x - currPos.x)/(nextPos.y - currPos.y)) * (180/PI) + 180);
    } else {
        if (nextPos.x - currPos.x > 0) {
            return 90;
        } else {
            return -90;
        }
    }
}

void navControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	coord_t currPos_local;

	while(1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataCurrPos);
			currPos_local = myData.currPos;
			pthread_mutex_unlock(&mutDataCurrPos);
			switch(state) {
				case GOTO_DEST:
					speedState = VIT50;
					if (lowBat) {
						state = PRE_BATT_LOW;
					} else {
						if (destReached) {
							// Destination atteinte donc on en génère une nouvelle
							nb_destReached += 1;
							pm.genDest(currPos_local, dest);
							pm.genWp(currPos_local, dest, nextStep);
							nb_stepReached = 0; // resetting this variable
							destReached = false;
						} else {
							// Étape atteinte ?
							if (nextStepReached(currPos_local, nextStep)) {
								nb_stepReached += 1;
								// On génère la prochaine étape
								pm.genWp(currPos_local, dest, nextStep);
							} else {
								// Sinon on met à jour l'angle
								orderedAngle = getAngle(currPos_local, nextStep);
							}
						}
					}
					break;
				case PRE_BATT_LOW:
					speedState = VIT30;
					pm.getClosestStation(currPos_local, stationPos);
					state = BATT_LOW;
					break;
				case BATT_LOW:
					orderedAngle = getAngle(currPos_local, stationPos);
					if (nextStepReached(currPos_local, stationPos))
						state = CHARGING;
					break;
				case CHARGING:
					speedState = VIT0;
					inCharge = true;
					if (highBat) {
						// On genere une nouvelle etape a partir de la station
						// avant de retourner à l'état GOTO_DEST
						pm.genWp(currPos_local, dest, nextStep);
						inCharge = false;
						state = GOTO_DEST;
					}
					break;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

/********************************************************************
* Ce thread determine quand la voiture est arrivée à la destination *
********************************************************************/
void destControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	coord_t currPos_local;
	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataCurrPos);
			currPos_local = myData.currPos;
			pthread_mutex_unlock(&mutDataCurrPos);
			if (nextStepReached(currPos_local, dest))
				destReached = true;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

char * getCarState(enum carState state) {
	switch (state) {
	  case GOTO_DEST:    return (char *)"GOTO_DEST";
	  case PRE_BATT_LOW: return (char *)"PRE_BATT_LOW";
	  case BATT_LOW:     return (char *)"BATT_LOW";
	  case CHARGING:     return (char *)"CHARGING";
	  default:           return (char *)"UNKNOWN";
   }
}

void display_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	float bat, speed_local, angle_local;
	double x, y;

	while (1)	{
		if (sem_wait(sync_sem) == 0) {

			pthread_mutex_lock(&mutDataBattLevel);
			bat = myData.battLevel;
			pthread_mutex_unlock(&mutDataBattLevel);

			pthread_mutex_lock(&mutDataSpeed);
			speed_local = myData.speed;
			pthread_mutex_unlock(&mutDataSpeed);

			pthread_mutex_lock(&mutDataAngle);
			angle_local = myData.angle;
			pthread_mutex_unlock(&mutDataAngle);

			pthread_mutex_lock(&mutDataCurrPos);
			x = myData.currPos.x;
			y = myData.currPos.y;
			pthread_mutex_unlock(&mutDataCurrPos);

			cout << "******************DISPLAY******************\n"
				 << "battLevel: " << bat         << "%.\n"
				 << "speed: "     << speed_local << "km/h        carState: " << getCarState(state) << "\n"
				 << "angle: "     << angle_local << "°\n"
				 << " currPos.x: " << setw(9) << x              << "  currPos.y: " << y              << "\n"
				 << "nextStep.x: " << setw(9) << nextStep.x     << " nextStep.y: " << nextStep.y     << "\n"
				 << "    dest.x: " << setw(9) << dest.x         << "     dest.y: " << dest.y         << "\n"
				 << " station.x: " << setw(9) << stationPos.x   << "  station.y: " << stationPos.y   << "\n"
				 << "   nb step: " << setw(9) << nb_stepReached << "    nb dest: " << nb_destReached << "\n"
				 << "*******************************************" << endl;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}	

/* vim: set ts=4 sw=4 tw=90 noet :*/

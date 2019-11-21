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
#include <ctime>
#include <chrono>
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

FILE *trace_data;

coord_t obstacle;

int main() {

	init();

	return 0;
}

void init()
{

	if ((trace_data = fopen("./data.csv", "w+")) == NULL) {
		perror("Error while opening file");
		return;
	}

	// Valeurs initiales

	speedState       = VIT0;
	orderedAngle 	 = 0;
	myData.battLevel = 12;
	myData.currPos.x = 0;
	myData.currPos.y = 0;
	destReached      = true; // pour generer un destination initiale
	stationPos.x     = 0;
	stationPos.y     = 0;
	nb_stepReached   = 0;
	nb_destReached   = -1; // car on l'incremente a l'initialisation
	obstacle.x = 0;
	obstacle.y = 0;

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

	/* Time manager */
	struct timespec tp;
	/* Get the start time */
	if(0 != clock_gettime(CLOCK_REALTIME, &tp)) {
		printf("Could not get start time: %d\n", errno);
		return;
	}

	/* Timers and threads */
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

	// périodes em ms
	uint32_t periods[THREAD_NUM] = {2,  		 // cameraControl_worker
							   	    2,  		 // camera_worker
									2,  		 // battery_worker
									2,  		 // battLow_worker
									2,  		 // battHigh_worker
									2,  		 // angle_worker
									2,  		 // speed_worker
									2,  		 // currPos_worker
									2,  		 // navControl_worker
									2,  		 // destControl_worker
									1000,		 // display_worker
									10   		 // trace_worker
	};
	// priorités
	int prios[THREAD_NUM] = {2,  // cameraControl_worker
							 4,  // camera_worker
							 4,  // battery_worker
							 3,  // battLow_worker
							 3,  // battHigh_worker
							 4,  // angle_worker
							 4,  // speed_worker
							 3,  // currPos_worker
							 3,  // navControl_worker
							 3,  // destControl_worker
							 1,  // display_worker
							 1	 // trace_worker
	};

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
		task_args[i].period    = periods[i];
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
		mySchedParam.sched_priority = 5; // maximum priority
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

	/* Partition data and parameters */
	unsigned int sched_pol = SCHED_APS_SCHEDPOL_LIMIT_CPU_USAGE;
	sched_aps_create_parms sched_partitions[2]; // Partition pour continu et discret
	sched_aps_parms        sched_params[2];
	sched_aps_join_parms   sched_join_param[THREAD_NUM];

	/* Partitions creations */
	if(0 != create_partitions(sched_partitions, sched_params, &sched_pol)) {
		return;
	}
	/* Assign threads to partitions */
	if(0 != assign_partitions(sched_join_param, sched_partitions, tid, THREAD_NUM)) {
		return;
	}

	sleep(SIMU_TIME);
//	pm.dumpImage("./map.bmp");

	// join des threads
	for (i = 0; i < THREAD_NUM; ++i) {
		pthread_cancel(tid[i]);
		pthread_cancel(pulseHandlers[i]);
	}

	// Close file
	fclose(trace_data);

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
		case 11: trace_worker(data);         break;
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
	uint32_t task_id, period;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;
	period    = ((thread_args_t*)data)->period;

	float speed_local, batt_local;

	while(1)	{
		if (sem_wait(sync_sem) == 0) {
			if (inCharge)	{
				pthread_mutex_lock(&mutDataBattLevel);
				myData.battLevel += (float)CONST_CHARGE * period / 1000 ;
				pthread_mutex_unlock(&mutDataBattLevel);
			}
			pthread_mutex_lock(&mutDataSpeed);
			speed_local = myData.speed;
			pthread_mutex_unlock(&mutDataSpeed);
			pthread_mutex_lock(&mutDataBattLevel);
			myData.battLevel -= SIMU_ACCEL * ((float)COEFF_DECHARGE * speed_local + (float)CONST_DECHARGE) * ((float)period/1000);
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
	uint32_t task_id, period;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;
	period    = ((thread_args_t*)data)->period;

	float deltaX, deltaY, dist;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataSpeed);
			dist = SIMU_ACCEL * 1000 * myData.speed * period / (1000 * 3600) ;
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
	auto start = std::chrono::system_clock::now();
	bool has_obstacle = false;

	while(1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataCurrPos);
			currPos_local = myData.currPos;
			pthread_mutex_unlock(&mutDataCurrPos);
			switch(state) {
				case GOTO_DEST:
					speedState = VIT50;
					// Critique donc on teste en premier
					if (has_obstacle && nextStepReached(currPos_local, obstacle)) {
						state = PRE_OBSTACLE;
						break;
					}
					if (lowBat) {
						state = PRE_BATT_LOW;
						break;
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
								// Generation aleatoire d'obstacle
								has_obstacle = pm.genObstacle(currPos_local,
															  nextStep,
															  obstacle);
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
					// Il peut également y avoir un obstacle sur le chemin de la station
					pm.genObstacle(currPos_local, stationPos, obstacle);
					state = BATT_LOW;
					break;
				case BATT_LOW:
					orderedAngle = getAngle(currPos_local, stationPos);
					if (has_obstacle && nextStepReached(currPos_local, obstacle)) {
						state = PRE_OBSTACLE;
						break;
					}
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
				case PRE_OBSTACLE:
					speedState = VIT0;
					start = std::chrono::system_clock::now();
					state = OBSTACLE;
					break;
				case OBSTACLE:
					auto end = std::chrono::system_clock::now();
					std::chrono::duration<double> elapsed_time = end - start;
					// délai de 5 secondes avant de repartir
					if (elapsed_time.count() > 1) {
						if (lowBat)
							state = BATT_LOW;
						else
							state = GOTO_DEST;
						has_obstacle = false;
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
	  case PRE_OBSTACLE: return (char *)"PRE_OBSTACLE";
	  case OBSTACLE:     return (char *)"OBSTACLE";
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
				 << "  currPos.x: "  << setw(9) << x              << "   currPos.y: "  << y              << "\n"
				 << " nextStep.x: "  << setw(9) << nextStep.x     << "  nextStep.y: "  << nextStep.y     << "\n"
				 << "     dest.x: "  << setw(9) << dest.x         << "      dest.y: "  << dest.y         << "\n"
				 << "  station.x: "  << setw(9) << stationPos.x   << "   station.y: "  << stationPos.y   << "\n"
				 << " obstacle.x: " << setw(9) << obstacle.x      << "  obstacle.y: " << obstacle.y     << "\n"
				 << "    nb step: "  << setw(9) << nb_stepReached << "     nb dest: "  << nb_destReached << "\n"
				 << "*******************************************" << endl;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}	

void trace_worker(void * data) {

	sem_t* sync_sem;
	sync_sem  = ((thread_args_t*)data)->semaphore;

	auto start = std::chrono::system_clock::now();

	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_time = end - start;
			// Pas de mutex car pas besoin de précision
			fprintf(trace_data,
					"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
					elapsed_time.count() * SIMU_ACCEL,
					myData.battLevel,
					myData.speed,
					myData.angle,
					myData.currPos.x,
					myData.currPos.y,
					nextStep.x,
					nextStep.y,
					dest.x,
					dest.y,
					obstacle.x,
					obstacle.y,
					stationPos.x,
					stationPos.y,
					nb_stepReached,
					nb_destReached);
		}
	}

	return;
}

/* vim: set ts=4 sw=4 tw=90 noet :*/

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
/* #include <sys/syspage.h> */ 
/* #include <sys/neutrino.h> */ 
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <iomanip>  // needed to use manipulators with parameters (precision, width)
#include <cmath>
#include <sys/netmgr.h>   /* ND_LOCAL_NODE */
#include <sys/neutrino.h> /* ChannelCreate ConnectAttach MsgReceive */
#include "genMap.h"
#include "astromobile.h"

//Manque a creer un timer et un pulse handler par tache

using namespace std;

PathMap pm;

struct physicsData myData;

enum carState state = GOTO_DEST; // état initial à GOTO_DEST

coord_t nextStep;
coord_t dest;
coord_t stationPos;

bool takePx;
bool inCharge;
bool destReached;

bool lowBat;
bool highBat;

/* Semaphores for pulse synchronization */
	sem_t task0_sync;
	sem_t task1_sync;
	sem_t task2_sync;


	int32_t init_timer(struct sigevent* event, struct itimerspec* itime,
			timer_t* timer, const int32_t chanel_id,   const uint32_t period) {
		int32_t error;
		int32_t period_s;
		int32_t period_ns;

		/* Set event as pulse and attach to channel */
		event->sigev_notify = SIGEV_PULSE;
		event->sigev_coid   = ConnectAttach(ND_LOCAL_NODE, 0, chanel_id, _NTO_SIDE_CHANNEL, 0);

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


int main() {

	init();

	return 0;
}

void * cameraControl_worker(void * data) {
    double delta, x, y;
	while(1) {
		pthread_mutex_lock(&mutDataCurrPos);
		x = myData.currPos.x;
		y = myData.currPos.y;
		pthread_mutex_unlock(&mutDataCurrPos);
		while (delta < 10) {
				pthread_mutex_lock(&mutDataCurrPos);
				delta = sqrt(pow((myData.currPos.x - x), 2) + pow((myData.currPos.y - y), 2));
				pthread_mutex_unlock(&mutDataCurrPos);
		}
		//takePx = true;
		//release semaphore
		if(0 != sem_post(&task0_sync)) {
			/* Print error */
			printf("Could not post semaphore: %d\n", errno);
		}
		delta = 0;

		sleep(PERIOD);
	}
	return NULL;
}

void * camera_worker(void * data) {
        rgb_t image;
        while(1) {
        	/* Wait to release the semaphore */
        	if(0 == sem_wait(&task0_sync)) {
			//if (takePx) {
				pthread_mutex_lock(&mutDataCurrPos);
				image = pm.takePhoto(myData.currPos);
				pthread_mutex_unlock(&mutDataCurrPos);
				//takePx = false;
			}
        	else {
        		/* Print error */
        		printf("Task camera_worker could not get time: %d\n", errno);	}
        	}
			sleep(PERIOD);

        return NULL;
}

void * battery_worker(void * data) {

	float speed_local;
	float batt_local;

	while(1)	{
		if (inCharge)	{
			pthread_mutex_lock(&mutDataBattLevel);
			myData.battLevel += CONST_CHARGE;
			pthread_mutex_unlock(&mutDataBattLevel);
		}
		pthread_mutex_lock(&mutDataSpeed);
		speed_local = myData.speed;
		pthread_mutex_unlock(&mutDataSpeed);
		pthread_mutex_lock(&mutDataBattLevel);
		myData.battLevel -= (COEFF_DECHARGE * speed_local + CONST_DECHARGE);
		batt_local = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);
		if (batt_local <= 10)	{
			if(0 != sem_post(&task1_sync)) {
					/* Print error */
					printf("Could not post semaphore: %d\n", errno);
				}
		}
		else if (batt_local >= 80)	{
			if(0 != sem_post(&task2_sync)) {
					/* Print error */
					printf("Could not post semaphore: %d\n", errno);
				}
		}

		sleep(1);
	}
	return NULL; 
}	


/******************************************************
* currPos_worker : met à jour la position en fonction *
* de la vitesse et de la l'angle					  *
******************************************************/
void * currPos_worker(void * data) {

	float deltaX, deltaY, dist;

	while (1) { 

		pthread_mutex_lock(&mutDataSpeed);
		dist = 1000 * (myData.speed * (PERIOD/3600)) * 10;
		pthread_mutex_unlock(&mutDataSpeed);
		pthread_mutex_lock(&mutDataAngle);
		deltaX = dist * cos(myData.angle * PI / 180);
		deltaY = dist * sin(myData.angle * PI / 180);
		pthread_mutex_unlock(&mutDataAngle);
		pthread_mutex_lock(&mutDataCurrPos);
		myData.currPos.x += deltaX;	
		myData.currPos.y += deltaY;	
		pthread_mutex_unlock(&mutDataCurrPos);	

		sleep(PERIOD); // period
	}
	return NULL; 
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

void * navControl_worker(void * data) {

	coord_t currPos_local;

	while(1) {
		pthread_mutex_lock(&mutDataCurrPos);
		currPos_local = myData.currPos;
		pthread_mutex_unlock(&mutDataCurrPos);
		switch(state) {
			case GOTO_DEST:
				pthread_mutex_lock(&mutDataSpeed);
				myData.speed = 50;
				pthread_mutex_unlock(&mutDataSpeed);
				if (lowBat) {
					state = PRE_BATT_LOW;
				} else {
					if (destReached) {
						// Destination atteinte donc on en génère une nouvelle
						cout << "INFO : Destination reached" << endl;
						pm.genDest(currPos_local, dest);
						pm.genWp(currPos_local, dest, nextStep);
						destReached = false;
					} else {	
						// Étape atteinte ?
						if (nextStepReached(currPos_local, nextStep)) {
							// On génère la prochaine étape
							//cout << "INFO : Step reached" << endl;
							pm.genWp(currPos_local, dest, nextStep);
						} else {
							// Sinon on met à jour l'angle
							pthread_mutex_lock(&mutDataAngle);
							myData.angle = getAngle(currPos_local, nextStep);
							pthread_mutex_unlock(&mutDataAngle);
						}
					}
				}
				break;
			case PRE_BATT_LOW:
				pthread_mutex_lock(&mutDataSpeed);
				myData.speed = 30;
				pthread_mutex_unlock(&mutDataSpeed);
				pm.getClosestStation(currPos_local, stationPos);
				state = BATT_LOW;
				break;
			case BATT_LOW:
				pthread_mutex_lock(&mutDataAngle);
				myData.angle = getAngle(currPos_local, stationPos);
				pthread_mutex_unlock(&mutDataAngle);
				if (nextStepReached(currPos_local, stationPos))
					state = CHARGING;
				break;
			case CHARGING:
				pthread_mutex_lock(&mutDataSpeed);
				myData.speed = 0;
				pthread_mutex_unlock(&mutDataSpeed);
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
	sleep(PERIOD);
	}
	return NULL; 
}

/********************************************************************
* Ce thread determine quand la voiture est arrivée à la destination *
********************************************************************/
void * destControl_worker(void * data) {
	coord_t currPos_local;
	while (1) {
		pthread_mutex_lock(&mutDataCurrPos);
		currPos_local = myData.currPos;
		pthread_mutex_unlock(&mutDataCurrPos);	
		if (nextStepReached(currPos_local, dest))
			destReached = true;
		sleep(0.1);
	}
	return NULL; 
}	

void * battLow_worker(void * data) {
	//float bat;
	while (1)	{
		/*pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);	*/
		if(0 == sem_wait(&task1_sync)) {
		//if (bat <= 10 && !inCharge)
			lowBat = true;
		}


		/*else
			lowBat = false;*/
		sleep(PERIOD);
	}
	return NULL; 
}	

void * battHigh_worker(void * data) {
	//float bat;
	while (1)	{
		/*pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);	*/
		if(0 == sem_wait(&task2_sync))
			highBat = true;
		/*else
			highBat = false;*/
		sleep(PERIOD);
	}
	return NULL; 
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

void * display_worker(void * data) {

	float bat, speed_local, angle_local;
	double x, y;

	while (1)	{


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
		     << "battLevel: " << bat << "%.\n"
		     << "speed: " << speed_local << "km/h        carState: " << getCarState(state) << "\n"
		     << "angle: " << angle_local << "°\n"
		     << " currPos.x: " << setw(9) << x            << "  currPos.y: " << y << "\n"
		     << "nextStep.x: " << setw(9) << nextStep.x   << " nextStep.y: " << nextStep.y << "\n"
		     << "    dest.x: " << setw(9) << dest.x       << "     dest.y: " << dest.y << "\n"
		     << " station.x: " << setw(9) << stationPos.x << "  station.y: " << stationPos.y << "\n"
		     << "*******************************************" << endl;

		sleep(0.5);
	}
	return NULL; 
}	

void init()
{
	myData.speed = 30;
	myData.angle = 90;
	myData.battLevel = 13;
	myData.currPos.x = 0;
	myData.currPos.y = 0;

	destReached = true;
	stationPos.x = 0; stationPos.y = 0;


	/* Initialize the semaphore */
	if(0 != sem_init(&task0_sync, 0, 0)) {
		/* Print error */
		printf("Could not get init semaphore: %d\n", errno);
		//return EXIT_FAILURE;
	}

	if(0 != sem_init(&task1_sync, 0, 0)) {
		/* Print error */
		printf("Could not get init semaphore: %d\n", errno);
		//return EXIT_FAILURE;
	}

	if(0 != sem_init(&task2_sync, 0, 0)) {
		/* Print error */
		printf("Could not get init semaphore: %d\n", errno);
		//return EXIT_FAILURE;
	}


	pthread_t tid[THREAD_NUM];
	pthread_attr_t attrib;
	struct sched_param mySchedParam;
	int i; 

	// creation des threads
	//setprio(0,20);
	pthread_attr_init (&attrib);
	pthread_attr_setinheritsched (&attrib, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&attrib, SCHED_FIFO);

	void ** workerList[THREAD_NUM] = {cameraControl_worker, camera_worker, battery_worker, currPos_worker,
				  destControl_worker, battLow_worker, battHigh_worker, display_worker};


	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[0], NULL, cameraControl_worker, NULL ) < 0)
		cout << "taskSpawn cameraControl_worker failed!" << endl;

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[1], NULL, camera_worker, NULL ) < 0)
		cout << "taskSpawn camera_worker failed!" << endl;

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[2], NULL, battery_worker, NULL ) < 0)
		cout << "taskSpawn battery_worker failed!" << endl;

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[3], NULL, currPos_worker, NULL ) < 0)
		cout << "taskSpawn currPos_worker failed!" << endl;

	 mySchedParam.sched_priority = 20;
	 pthread_attr_setschedparam(&attrib, &mySchedParam);
	 if ( pthread_create(&tid[4], NULL, navControl_worker, NULL ) < 0)
		 printf("taskSpawn navControl_worker failed!\n");


	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[5], NULL, destControl_worker, NULL ) < 0)
		cout << "taskSpawn destControl_worker failed!" << endl;

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[6], NULL, battLow_worker, NULL ) < 0)
		cout << "taskSpawn battLow_worker failed!" << endl;

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[7], NULL, battHigh_worker, NULL ) < 0)
		cout << "taskSpawn battHigh_worker failed!" << endl;

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[8], NULL, display_worker, NULL ) < 0)
		cout << "taskSpawn display_worker failed!" << endl;

	sleep(1);
	pm.dumpImage("./map.bmp");

	// join des threads
	for (i = 0; i < 10; ++i) {
		if (pthread_join(tid[i], NULL) < 0)
			printf("Join failed for thread %d\n", i);
	}
}

/* vim: set ts=4 sw=4 tw=90 noet :*/

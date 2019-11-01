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
		//release semaphore
		if(0 != sem_post(&taskCamera_sync)) {
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
        	if(0 == sem_wait(&taskCamera_sync)) {
				pthread_mutex_lock(&mutDataCurrPos);
				image = pm.takePhoto(myData.currPos);
				pthread_mutex_unlock(&mutDataCurrPos);
			} else {
        		printf("Task camera_worker could not get time: %d\n", errno);	}
        	}
			sleep(PERIOD);

        return NULL;
}

void * battery_worker(void * data) {

	float speed_local, batt_local;

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
			if(0 != sem_post(&taskBattlow_sync)) {
					printf("Could not post semaphore: %d\n", errno);
				}
		}
		else if (batt_local >= 80)	{
			if(0 != sem_post(&taskBatthigh_sync)) {
					printf("Could not post semaphore: %d\n", errno);
				}
		}

		sleep(1);
	}
	return NULL; 
}

void * battLow_worker(void * data) {
	//float bat;
	while (1)	{
		/*pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);	*/
		if(0 == sem_wait(&taskBattlow_sync)) {
			lowBat  = true;
			highBat = false;
		}
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
		if(0 == sem_wait(&taskBatthigh_sync)) {
			highBat = true;
			lowBat  = false;
		}
		sleep(PERIOD);
	}
	return NULL;
}

/*************************************************************
* Thread representant la partie continue qui modifie l'angle *
* Pas forcement necessaire dans le programme mais c'est une  *
* representation plus correcte du reel                       *
*************************************************************/
void * angle_worker(void * data) {
	while (1) {
		pthread_mutex_lock(&mutDataAngle);
		myData.angle = orderedAngle;
		pthread_mutex_unlock(&mutDataAngle);
		sleep(PERIOD);
	}
}

/*************************************************************
* Thread representant la partie continue qui modifie la      *
* vitesse                                                   *
*************************************************************/
void * speed_worker(void * data) {
	while (1) {
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
		sleep(PERIOD);
	}
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
		     << "battLevel: " << bat         << "%.\n"
		     << "speed: "     << speed_local << "km/h        carState: " << getCarState(state) << "\n"
		     << "angle: "     << angle_local << "°\n"
		     << " currPos.x: " << setw(9) << x              << "  currPos.y: " << y              << "\n"
		     << "nextStep.x: " << setw(9) << nextStep.x     << " nextStep.y: " << nextStep.y     << "\n"
		     << "    dest.x: " << setw(9) << dest.x         << "     dest.y: " << dest.y         << "\n"
		     << " station.x: " << setw(9) << stationPos.x   << "  station.y: " << stationPos.y   << "\n"
			 << "   nb step: " << setw(9) << nb_stepReached << "    nb dest: " << nb_destReached << "\n"
		     << "*******************************************" << endl;

		sleep(1);
	}
	return NULL; 
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
	nb_destReached   = 0;

	/* Initialize the semaphores */
	if(0 != sem_init(&taskCamera_sync, 0, 0)) 
		printf("Could not init semaphore: %d\n", errno);
	
	if(0 != sem_init(&taskBattlow_sync, 0, 0))
		printf("Could not init semaphore: %d\n", errno);

	if(0 != sem_init(&taskBatthigh_sync, 0, 0)) 
		printf("Could not init semaphore: %d\n", errno);

	pthread_t tid[THREAD_NUM];
	pthread_attr_t attrib;
	struct sched_param mySchedParam;
	int i; 

	// creation des threads
	//setprio(0,20);
	pthread_attr_init (&attrib);
	pthread_attr_setinheritsched (&attrib, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&attrib, SCHED_FIFO);

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

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[9], NULL, speed_worker, NULL ) < 0)
		cout << "taskSpawn display_worker failed!" << endl;

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[10], NULL, angle_worker, NULL ) < 0)
		cout << "taskSpawn display_worker failed!" << endl;

	/* sleep(1); */
	/* pm.dumpImage("./map.bmp"); */

	// join des threads
	for (i = 0; i < 10; ++i) {
		if (pthread_join(tid[i], NULL) < 0)
			printf("Join failed for thread %d\n", i);
	}
}

/* vim: set ts=4 sw=4 tw=90 noet :*/

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

#include "genMap.h"
#include "astromobile.h"

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
		takePx = true;
		delta = 0;

		sleep(PERIOD);
	}
	return NULL;
}

void * camera_worker(void * data) {
        rgb_t image;
        while(1) {
			if (takePx) {
				pthread_mutex_lock(&mutDataCurrPos);
				image = pm.takePhoto(myData.currPos);
				pthread_mutex_unlock(&mutDataCurrPos);
				takePx = false;
			}
			sleep(PERIOD);
        }
        return NULL;
}

void * battery_worker(void * data) {

	float speed_local;

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
		pthread_mutex_unlock(&mutDataBattLevel);

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
		dist = 1000 * (myData.speed * (PERIOD/3600));
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
							cout << "INFO : Step reached" << endl;
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
	sleep(0.1);
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
	float bat;
	while (1)	{
		pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);
		if (bat <= 10 && !inCharge)
			lowBat = true;
		else
			lowBat = false;
		sleep(PERIOD);
	}
	return NULL; 
}	

void * battHigh_worker(void * data) {
	float bat;
	while (1)	{
		pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);
		if (bat >= 80 && inCharge)
			highBat = true;
		else
			highBat = false;
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

		cout << "******************DISPLAY******************" << endl;

		pthread_mutex_lock(&mutDataBattLevel);
		bat = myData.battLevel;
		pthread_mutex_unlock(&mutDataBattLevel);
		cout << "battLevel: " << bat << "%." <<  endl;

		pthread_mutex_lock(&mutDataSpeed);
		speed_local = myData.speed;
		pthread_mutex_unlock(&mutDataSpeed);
		cout << "speed: " << speed_local << "km/h        carState: " << getCarState(state) << endl;

		pthread_mutex_lock(&mutDataAngle);
		angle_local = myData.angle;
		pthread_mutex_unlock(&mutDataAngle);
		cout << "angle: " << angle_local << "°" << endl;

		pthread_mutex_lock(&mutDataCurrPos);
		x = myData.currPos.x;
		y = myData.currPos.y;
		pthread_mutex_unlock(&mutDataCurrPos);
		cout << " currPos.x: " << setw(9) << x            << "  currPos.y: " << y << endl;
		cout << "nextStep.x: " << setw(9) << nextStep.x   << " nextStep.y: " << nextStep.y << endl;
		cout << "    dest.x: " << setw(9) << dest.x       << "     dest.y: " << dest.y << endl;
		cout << " station.x: " << setw(9) << stationPos.x << "  station.y: " << stationPos.y << endl;

		cout << "*******************************************" << endl;

		sleep(3);
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
		printf("taskSpawn cameraControl_worker failed!\n");

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[1], NULL, camera_worker, NULL ) < 0)
		printf("taskSpawn camera_worker failed!\n");

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[2], NULL, battery_worker, NULL ) < 0)
		printf("taskSpawn battery_worker failed!\n");

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[3], NULL, currPos_worker, NULL ) < 0)
		cout << "taskSpawn currPos_worker failed!" << endl;

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[5], NULL, destControl_worker, NULL ) < 0)
		printf("taskSpawn destControl_worker failed!\n");

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[6], NULL, battLow_worker, NULL ) < 0)
		printf("taskSpawn battLow_worker failed!\n");

	mySchedParam.sched_priority = 1;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[7], NULL, battHigh_worker, NULL ) < 0)
		printf("taskSpawn battHigh_worker failed!\n");

	mySchedParam.sched_priority = 20;
	pthread_attr_setschedparam(&attrib, &mySchedParam);
	if ( pthread_create(&tid[8], NULL, display_worker, NULL ) < 0)
		printf("taskSpawn display_worker failed!\n");

	sleep(30);
	pm.dumpImage("./map.bmp");

	// join des threads
	for (i = 0; i < 10; ++i) {
		if (pthread_join(tid[i], NULL) < 0)
			printf("Join failed for thread %d\n", i);
	}
}

/* vim: set ts=4 sw=4 tw=90 noet :*/

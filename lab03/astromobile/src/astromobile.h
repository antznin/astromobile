#ifndef ASTROMOBILE_H
#define ASTROMOBILE_H

#include "genMap.h"

void init();

// Tâches
void * main_worker(void * data); // tâche principale qui lance les autres
void cameraControl_worker(void * data);
void camera_worker(void * data);
void battery_worker(void * data);
void battLow_worker(void * data);
void battHigh_worker(void * data);
void angle_worker(void * data);
void speed_worker(void * data);
void currPos_worker(void * data);
void navControl_worker(void * data);
void destControl_worker(void * data);
void display_worker(void * data);
void trace_worker(void * data);

// Nombre total de tâches
#define THREAD_NUM 12

#define PI 3.14159265359

// Dans tous les cas, la batterie gagne CONST_CHARGE % par seconde
#define CONST_CHARGE 5          
// Coefficient devant la vitesse
#define COEFF_DECHARGE 0.0001  
// Quelle que soit la vitesse, la batterie se décharge
#define CONST_DECHARGE 0.001666	

#define DIST_STEP 10    // Distance de détection d'une étape
#define DIST_OBSTACLE 5 // Distance de détection d'un obstacle

// Structure de données servant à stocker les données physiques de la voiture
struct physicsData {
	float speed;
	float battLevel;
	double angle;
	coord_t currPos;
};

// État de la voiture pour la machine à états
enum carState {GOTO_DEST, PRE_BATT_LOW, BATT_LOW, CHARGING, PRE_OBSTACLE, OBSTACLE};

// vitesses possibles : 0, 30, 50 km/h
enum speeds {VIT0, VIT30, VIT50};

// Mutex pour protéger les données physiques
pthread_mutex_t mutDataSpeed     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataBattLevel = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataAngle     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataCurrPos   = PTHREAD_MUTEX_INITIALIZER;

#endif /* ifndef ASTROMOBILE_H */

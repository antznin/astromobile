#ifndef ASTROMOBILE_H
#define ASTROMOBILE_H

#include "genMap.h"

void init();

// Taches
void * main_worker(void * data); // tache principale
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
void stepControl_worker(void * data);

#define THREAD_NUM 12

// periode pour les threads continus
#define PERIOD_BASE 200

#define PI 3.14159265359

// Maximum de message dans un queue
#define MAX_NUM_MSG 50
// Priorités des messages
#define MSG_PRI_LOW  10
#define MSG_PRI_MED  20
#define MSG_PRI_HIGH 30

#define CONST_CHARGE 5          // la batterie gagne 5% par période
#define COEFF_DECHARGE 0.0001   // 1/(60*30)
								// la batterie perd coeff*vitesse par
								// minute en décharge, donc 1% perdu par minute quand elle roule a 30
#define CONST_DECHARGE 0.001666	// 1/(60*10)

struct physicsData {
	float speed;
	float battLevel;
	double angle;
	coord_t currPos;
};

// État de la machine pour la machine à états
enum carState {GOTO_DEST, PRE_BATT_LOW, BATT_LOW, CHARGING};

// vitesses possibles
enum speeds {VIT0, VIT30, VIT50};

// Mutex pour protéger
pthread_mutex_t mutDataSpeed     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataBattLevel = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataAngle     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataCurrPos   = PTHREAD_MUTEX_INITIALIZER;

#endif /* ifndef ASTROMOBILE_H */

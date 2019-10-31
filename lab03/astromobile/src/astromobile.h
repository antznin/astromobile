#ifndef ASTROMOBILE_H
#define ASTROMOBILE_H

#include "genMap.h"

void init();

#define THREAD_NUM 10

#define PERIOD 0.1

#define PI 3.14159265359

// Maximum de message dans un queue
#define MAX_NUM_MSG 50
// Priorités des messages
#define MSG_PRI_LOW  10
#define MSG_PRI_MED  20
#define MSG_PRI_HIGH 30

#define CONST_CHARGE 5	// = 1/60		//la batterie gagne 1% par minute en charge
#define COEFF_DECHARGE 0.00555 //1/(60*30)		//la batterie perd coeff*vitesse par
//minute en décharge, donc 1% perdu par minute quand elle roule a 30
#define CONST_DECHARGE 0.001666	//1/(60*10)		//la batterie perd constament 1%
//toutes les 10mn indépendamment de la vitesse

struct physicsData {
	float speed;
	float battLevel;
	double angle;
	coord_t currPos;
};

// État de la machine pour la machine à états
enum carState {GOTO_DEST, PRE_BATT_LOW, BATT_LOW, CHARGING};

// Mutex pour protéger
pthread_mutex_t mutDataSpeed     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataBattLevel = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataAngle     = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t mutDataCurrPos   = PTHREAD_MUTEX_INITIALIZER;

#endif /* ifndef ASTROMOBILE_H */

#ifndef ASTROMOBILE_H
#define ASTROMOBILE_H

#include "genMap.h"

void init();

#define THREAD_NUM 10

#define PERIOD 0.1

// Maximum de message dans un queue
#define MAX_NUM_MSG 50
// Priorités des messages
#define MSG_PRI_LOW  10
#define MSG_PRI_MED  20
#define MSG_PRI_HIGH 30

#define COEFF_CHARGE 1/60		//la batterie gagne 1% par minute en charge
#define COEFF_DECHARGE 1/(60*30)		//la batterie perd coeff*vitesse par
//minute en décharge, donc 1% perdu par minute quand elle roule a 30
#define CONST_DECHARGE 1/(60*10)		//la batterie perd constament 1%
//toutes les 10mn indépendamment de la vitesse

#define MQ_NAV  "/MQ_Navigation"
#define MQ_CAM  "/MQ_Camera"
#define MQ_BATT "/MQ_Batterie"

struct physicsData {
	float speed;
	float battLevel;
	float angle;
	coord_t currPos;
};

typedef enum {MSG_DATA, MSG_ALERTE} msgType;

// structure de message partagé entre les tâches
typedef struct {
	int ID;
	msgType type;
	unsigned int size;
} message;

mqd_t msgQCam, msgQNav, msgQBatt;

#endif /* ifndef ASTROMOBILE_H */

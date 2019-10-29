#ifndef ASTROMOBILE_H
#define ASTROMOBILE_H

#include "genMap.h"

void init();

#define THREAD_NUM 10

// Maximum de message dans un queue
#define MAX_NUM_MSG 50
// Priorités des messages
#define MSG_PRI_LOW  10
#define MSG_PRI_MED  20
#define MSG_PRI_HIGH 30

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

#define S_FUNCTION_NAME ctrlauto
#include "ttkernel.cpp"
#include "mex.h"

#include <time.h>
#include <math.h>

#define PI 3.141592

#define MAX_STEP_DIST 5000.0
#define NEW_DEST_MAX_DIST_M 50000
#define STATION_MAX_DIST_M 5000
#define PX_TO_M 10
#define T_INSTRUCTION 0.000001
#define T_READ 0.001


typedef struct
{
    double x;
    double y;
} coord_t;

/* Generates a step between actual position and the destination*/
/* Returns the simulated time of execution */
double getNextStep(int32_t destX, int32_t destY, 
                 int32_t posCourX, int32_t posCourY,
                 int32_t &stepX, int32_t &stepY)
{
    int nb_operations = 0;
    double d = sqrt((destX-posCourX)*(destX-posCourX) + 
                      (destY-posCourY)*(destY-posCourY));
    nb_operations++;
    if(d <= 100)
    {
        stepX = destX;
        stepY = destY;
        nb_operations+=2;
        return nb_operations / 1000000;
    }
    nb_operations++;
    
    float alpha = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
    float tmp = (MAX_STEP_DIST/static_cast <float> (d));
    nb_operations+=2;
    if(tmp < 1)
    {
        alpha = alpha * tmp;
        nb_operations++;
    }
    nb_operations++;
    
    if(alpha>0.5)
    {
        alpha = 1 - alpha;
        nb_operations++;
    }
    nb_operations++;

    float beta = static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*d*alpha))) - d*alpha;

    stepX = alpha*(destX-posCourX) + posCourX;

    stepY = alpha*(destY-posCourY) + posCourY;

    stepX += static_cast <float> (beta*(posCourY-destY))/static_cast <float> (d);
    stepY += beta*(destX-posCourX)/d;
    nb_operations += 5;
    return nb_operations / 1000000;
}

/* Generates a new destination */
/* Returns the simulated time of execution */
double gen_dest(coord_t actualPos, coord_t &dest)
{
    int nb_operations = 0;
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.x = (int) (actualPos.x + NEW_DEST_MAX_DIST_M*randVal/3);
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    dest.y = (int) (actualPos.y + NEW_DEST_MAX_DIST_M*randVal/3);
    nb_operations = 20;
    return nb_operations / 1000000;
}

/* Generates a new waypoint between actual position and the destination*/ 
/* Returns the simulated time of execution */
double gen_wp(coord_t actualPos, coord_t dest, coord_t &wp)
{
    int nb_operations = 0;
    
    int32_t stepX;
    int32_t stepY;
    
    mexPrintf("gen_waypoint seg 1\n");
    getNextStep((int)dest.x, (int) dest.y, 
                (int) actualPos.x, (int) actualPos.y,
                stepX, stepY);
    wp.x = (double) stepX;
    wp.y = (double) stepY;
    nb_operations = 20;
    return nb_operations / 1000000;
}

/* Computes the position of the closest station */ 
/* Returns the simulated time of execution */
double getClosestStation(coord_t actualPos, coord_t &stationPos)
{
    mexPrintf("data_ptr->actualPos.x: %f\n", actualPos.x);
    mexPrintf("data_ptr->actualPos.y: %f\n", actualPos.y);
    
    int nb_operations = 0;
    float randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    
    stationPos.x = actualPos.x + STATION_MAX_DIST_M*randVal/PX_TO_M/3;
    randVal = static_cast<float> (rand()) / static_cast<float>(RAND_MAX);
    stationPos.y = actualPos.y + STATION_MAX_DIST_M*randVal/PX_TO_M/3;
    nb_operations = 24;
    mexPrintf("Closest station: (%f, %f)\n", stationPos.x, stationPos.y);
    return nb_operations / 1000000;
}

/********************* A COMPLETER ************************************/

// Data structure used for the task data

struct TaskData 
{
    coord_t *stationPos;
    coord_t *dest;
    coord_t *currPos;
    coord_t *nextStep;
    bool needToCharge;
};

double alerte10_code(int seg, void * m_data) {
    TaskData * data = (TaskData *)m_data;
    switch(seg) {
    case 1:
        data->needToCharge = true;
        return T_INSTRUCTION;
     default:
         return FINISHED;
    }
            
}

double alerte80_code(int seg, void * m_data) {
    TaskData * data = (TaskData *)m_data;
    switch(seg) {
    case 1:
        data->needToCharge = false;
        return T_INSTRUCTION;
     default:
         return FINISHED;
    }
            
}

double getAngle(coord_t currPos, coord_t nextPos) {
      return (atan((nextPos.x - currPos.x)/(nextPos.y - currPos.y)) * (180/PI));
}

double nav_code(int seg , void* data) {
    TaskData* d = (TaskData*) data;
    double t;
    switch (d->needToCharge) {
        case false:
            // TODO : nav de la voiture normal
            switch(seg) {
                case 1:
                    ttAnalogOut(4, 50);
                    d->currPos->x = ttAnalogIn(3); // recup de la pos
                    d->currPos->y = ttAnalogIn(4);
                    return 2 * T_READ + 2 * T_INSTRUCTION;
                case 2:
                    int32_t stepX;
                    int32_t stepY;
                    t = getNextStep((int)d->dest->x, (int)d->dest->y,
                                    (int)d->currPos->x, (int)d->currPos->y,
                                    stepX, stepY);
                    d->nextStep->x = stepX;
                    d->nextStep->y = stepY;
                    return 4 * T_INSTRUCTION + t;
                case 3:
                    ttAnalogOut(3, getAngle(*d->currPos, *d->nextStep));
                    return T_INSTRUCTION;
                default:
                    return FINISHED;
            }
            

        case true:
            switch (seg) {
                case 1:
                    ttAnalogOut(4, 30); // vitesse a 30
                    d->currPos->x = ttAnalogIn(3); // recup de la pos
                    d->currPos->y = ttAnalogIn(4);
                    return 2 * T_READ + 2 * T_INSTRUCTION;
                case 2:
                    // get the closest station and store it in data's stationPos
                    t += getClosestStation(*d->currPos, *d->stationPos);
                    return t;
                case 3:
                    int32_t stepX;
                    int32_t stepY;
                    t = getNextStep((int)d->stationPos->x, (int)d->stationPos->y,
                                    (int)d->currPos->x, (int)d->currPos->y,
                                    stepX, stepY);
                    d->nextStep->x = stepX;
                    d->nextStep->y = stepY;
                    return 4 * T_INSTRUCTION + t;
                case 4:
                    ttAnalogOut(3, getAngle(*d->currPos, *d->nextStep));
                    return T_INSTRUCTION;
                default:
                    return FINISHED;
                    
        }
    }
}

double camera_code(int seg) {
    switch (seg) {
        case 1:
            ttAnalogOut(2, 1);
            return T_INSTRUCTION;
        case default:
            return FINISHED;
    }
}




// Kernel init function    
void init()
{
    TaskData *data = new TaskData;
    
    ttSetUserData(data);
    
    data->needToCharge = false;

    ttInitKernel(prioFP);

    ttCreateHandler("alerte10_handler", 1, alerte10_code, (void *)data);
    //activate battery save
    ttCreateHandler("alerte80_handler", 1, alerte80_code, (void *)data);
    //exit charge station

    ttAttachTriggerHandler(1, "alerte10_handler");
    ttAttachTriggerHandler(2, "alerte80_handler");
    
    ttCreatePeriodicTask("nav", 0, 0.1, nav_code, (void *)data);
    ttCreatePeriodicTask("camera", 0, 10, camera_code, (void *)data);

}

// Kernel cleanup function
void cleanup() 
{
    delete (TaskData *)ttGetUserData();
}

#define S_FUNCTION_NAME ctrlauto
#include "ttkernel.cpp"
#include "mex.h"

#include <time.h>
#include <math.h>
#include <string.h>

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
    coord_t stationPos;
    coord_t dest;
    coord_t currPos;
    coord_t nextStep;
    coord_t lastCamPos;
    bool    needToCharge;
    bool    beginCycle;
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
    if (nextPos.y - currPos.y > 0) {
        return (atan((nextPos.x - currPos.x)/(nextPos.y - currPos.y)) * (180/PI));
    } else if (nextPos.y - currPos.y < 0) {
        return (atan((nextPos.x - currPos.x)/(nextPos.y - currPos.y)) * (180/PI) + 180);
    } else {
        if (nextPos.x - currPos.x > 0) {
            return 90;
        } else {
            return -90;
        }
    }
}

double dest_code(int seg, void * data) {
    
    TaskData * d = (TaskData *)data;
    double t;

    switch (seg) {
        case 1:
            ttWait("gendest");
            t = 0;
            d->currPos.x = ttAnalogIn(5); d->currPos.y = ttAnalogIn(6);
            int32_t stepX;
            int32_t stepY;
            if (d->needToCharge) {
                t += getClosestStation(d->currPos, d->stationPos);
                d->nextStep.x = d->stationPos.x;
                d->nextStep.y = d->stationPos.y;
            } else {
                t += gen_dest(d->currPos, d->dest);
                t += getNextStep((int)d->dest.x, (int)d->dest.y,
                                (int)d->currPos.x, (int)d->currPos.y,
                                stepX, stepY);
                d->nextStep.x = (double)stepX;
                d->nextStep.y = (double)stepY;
            }
            
            d->beginCycle = false;
            return (t + 6) * T_INSTRUCTION + 2 * T_READ;
        default:
            ttSetNextSegment(1);
    }
}

double nav_code(int seg , void* data) {

    TaskData* d = (TaskData*) data;
    double t;
    float deltaX, deltaY;

    switch(seg) {
        case 1:
            if (d->needToCharge) {
                ttAnalogOut(4, 30);
            } else {
                ttAnalogOut(4, 50);
            }
            return 1 * T_INSTRUCTION;
        case 2:
            if (d->beginCycle) {
                ttNotify("gendest");
            }
            return 2 * T_INSTRUCTION;
        case 3:
            // écriture de l'angle
            ttAnalogOut(3, getAngle(d->currPos, d->nextStep));
            return T_INSTRUCTION;
        default:
            return FINISHED;
    }
}

double camera_code(int seg, void * data) {

    double delta;
    TaskData* d = (TaskData*) data;

    switch (seg) {
        case 1:
            d->lastCamPos.x = ttAnalogIn(5);
            d->lastCamPos.y = ttAnalogIn(6);
            ttAnalogOut(2, 1);
            return 3 * T_INSTRUCTION;
        case 2:
            double done;
            done = ttAnalogIn(4);
            double t_compt;
            t_compt = 0;
            while (done == 0) {
                done = ttAnalogIn(4);
                t_compt += 1;
            }
            ttAnalogOut(2, 0);
            return (t_compt + 3) * T_INSTRUCTION + t_compt * T_READ;
        case 3:
            double t_compt2;
            t_compt2 = 0;
            delta = 0;
            while (delta < 10) {
                delta = sqrt(pow((ttAnalogIn(5) - d->lastCamPos.x), 2) 
                           + pow((ttAnalogIn(6) - d->lastCamPos.y), 2));
                        t_compt2 += 2;
            }
            return t_compt2 * T_INSTRUCTION;
        default:
            return FINISHED;
    }
}

// Kernel init function    
void init()
{
    TaskData *data = new TaskData;
    
    ttSetUserData(data);
    
    data->needToCharge = false;
    data->beginCycle = true;

    ttInitKernel(prioFP);

    ttCreateEvent("gendest");

    ttCreateHandler("alerte10_handler", 1, alerte10_code, (void *)data);
    ttCreateHandler("alerte80_handler", 1, alerte80_code, (void *)data);

    ttAttachTriggerHandler(1, "alerte10_handler");
    ttAttachTriggerHandler(2, "alerte80_handler");
    
    ttCreatePeriodicTask("nav", 0, 0.1, nav_code, (void *)data);
    ttCreatePeriodicTask("camera", 0, 0.1, camera_code, (void *)data);
    ttCreateJob("camera");
    ttCreateTask("dest", 0, dest_code, (void *)data);

}

// Kernel cleanup function
void cleanup() 
{
    delete (TaskData *)ttGetUserData();
}

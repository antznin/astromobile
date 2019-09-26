#define S_FUNCTION_NAME ctrlauto
#include "ttkernel.cpp"
#include "mex.h"

#include <time.h>

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
    coord_t * stationPos;
    coord_t * dest;
};

double alerte10_code(int seg, void * data) {
    
    switch(seg) {
    case 1:
        ttAnalogOut(4, 30); // consigne vitesse a 30
        return T_INSTRUCTION;
    case 2:
        coord_t currPos;
        currPos.x = ttAnalogIn(3);
        currPos.y = ttAnalogIn(4);
        return 2 * T_READ + T_INSTRUCTION;
    case 3:
        double nb_op = 0;
        // get the closest station and store it in data's stationPos
        nb_op += getClosestStation(currPos, data->stationPos);
        return (nb_op) * T_INSTRUCTION;
        return t_exec;
     default:
         return FINISHED;
    }
            
}

double alerte80_code(int seg, void * data) {
    
//     switch seg {
//     case 1:
//         ttAnalogOut(4, 30); // consigne vitesse a 30
//         return T_INSTRUCTION;
//     case 2:
//         double nb_op = 0;
//         coord_t currPos;
//         currPos.x = ttAnalogIn(3);
//         currPos.y = ttAnalogIn(4);
//         // get the closest station and store it in data's stationPos
//         nb_op = getClosestStation(currPos, data->stationPos);
//         return (nb_op + 2) * T_INSTRUCTION + 2 * T_READ;
//     case 3:
//         return t_exec;
//      default:
         return FINISHED;
    }
            
}

double nav_code(...

// Kernel init function    
void init()
{
    TaskData *data = new TaskData;
    
    ttSetUserData(data);
    
    ttCreateHandler("alerte10_handler", 1, alerte10_code, data);
    //activate battery save
    ttCreateHandler("alerte80_handler", 1, alerte80_code, data);
    //exit charge station

    ttAttachTriggerHandler(1, "alerte10_handler");
    ttAttachTriggerHandler(2, "alerte80_handler");
    
    ttCreatePeriodicTask("nav", 0, 0.1, nav_code, data);
}

// Kernel cleanup function
void cleanup() 
{
    delete (TaskData *)ttGetUserData();
}

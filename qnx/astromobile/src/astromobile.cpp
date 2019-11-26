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
#include <sys/syspage.h>
#include <sys/stat.h>
#include <sys/netmgr.h>   /* ND_LOCAL_NODE */
#include <sys/neutrino.h> /* ChannelCreate ConnectAttach MsgReceive */
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <iomanip>  // needed to use manipulators with parameters (precision, width)
#include <cmath>
#include <ctime>
#include <chrono>
#include "genMap.h"
#include "astromobile.h"
#include "utils.h"
using namespace std;

/*********************
* Variables globales *
*********************/

// Carte principale
PathMap pm;

// Données physiques de la voiture
struct physicsData myData;

// État de la machine à état (navControl_worker)
enum carState state = GOTO_DEST; // état initial à GOTO_DEST
// États de la vitesse
enum speeds speedState;

// Angle commandé par la partie contrôle
double orderedAngle;

coord_t nextStep;   // La prochaine étape
coord_t dest;		// La prochaine destination
coord_t stationPos; // La prochaine station
coord_t obstacle;   // Coordonnées de l'obstacle

bool inCharge;    // Booléen indiquant si la voiture est en charge ou non 
bool destReached; // Booléen indiquant si la destination est atteinte ou non

bool lowBat;  // Booléen indiquant si la batterie est faible
bool highBat; // Booléen indiquant si la batterie est chargée

// Nombre d'étapes atteintes entre deux destinations
unsigned int nb_stepReached; 
// Nombre de destinations atteintes
unsigned int nb_destReached;

// Sémaphores de synchronisation des tâches 
sem_t taskCamera_sync;   //  camera_worker <--> cameraControl_worker
sem_t taskBattlow_sync;  // battery_worker <--> battLow_worker 
sem_t taskBatthigh_sync; // battery_worker <--> battHigh_worker 

// Fichier d'écriture des données de la voiture
FILE *trace_data;


/**********************
* Entrée du programme *
**********************/
int main() {

	init();

	return 0;
}

/*****************************************************
* Initialisation des variables, création des threads *
*****************************************************/
void init()
{

	// Création du fichier
	if ((trace_data = fopen("./data.csv", "w+")) == NULL) {
		perror("Error while opening file");
		return;
	}

	// Valeurs initiales des variables globales
	speedState       = VIT0;
	orderedAngle     = 0;
	myData.battLevel = 12; // Batterie faible à l'initialisation pour tester la recherche de station
	myData.currPos.x = 0;
	myData.currPos.y = 0;
	destReached      = true; // pour generer un destination initiale
	stationPos.x     = 0;
	stationPos.y     = 0;
	nb_stepReached   = 0;
	nb_destReached   = -1; // car on l'incrémente a l'initialisation
	obstacle.x       = 0;
	obstacle.y       = 0;

	// Initialisation des sémaphores de synchronisation
	if(0 != sem_init(&taskCamera_sync, 0, 0)) {
		cout << "Could not init semaphore taskCamera_sync:" <<  errno << endl;
		return;
	}
	if(0 != sem_init(&taskBattlow_sync, 0, 0)) {
		printf("Could not init semaphore taskBattlow_sync: %d\n", errno);
		return;
	}
	if(0 != sem_init(&taskBatthigh_sync, 0, 0)) {
		printf("Could not init semaphore taskBatthigh_sync: %d\n", errno);
		return;
	}

	/* Time manager */
	struct timespec tp;
	/* Get the start time */
	if(0 != clock_gettime(CLOCK_REALTIME, &tp)) {
		printf("Could not get start time: %d\n", errno);
		return;
	}

	// Initialisation des variables propres au threads et timers
	sem_t * sem_syncs               = (sem_t *)calloc(THREAD_NUM, sizeof(sem_t)); // timer sync semaphores
	pthread_t * pulseHandlers       = (pthread_t *)calloc(THREAD_NUM, sizeof(pthread_t)); // pulse handlers
	struct sigevent * sigevents     = (struct sigevent *)calloc(THREAD_NUM, sizeof(struct sigevent));
	struct itimerspec * itimerspecs = (struct itimerspec *)calloc(THREAD_NUM, sizeof(struct itimerspec));
	timer_t * timers                = (timer_t *)calloc(THREAD_NUM, sizeof(timer_t));
	thread_args_t * task_args       = (thread_args_t *)calloc(THREAD_NUM, sizeof(thread_args_t));
	pthread_t * tid                 = (pthread_t *)calloc(THREAD_NUM, sizeof(pthread_t)); // Tableau des tâches
	pthread_attr_t attrib; // Attributs des tâches (réglage de priorités)
	struct sched_param mySchedParam; 
	pthread_attr_init (&attrib);
	pthread_attr_setinheritsched (&attrib, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setschedpolicy (&attrib, SCHED_FIFO);

	// Périodes des tâches en ms
	uint32_t periods[THREAD_NUM] = {2,    // cameraControl_worker
							   	    2,    // camera_worker
									2,    // battery_worker
									2,    // battLow_worker
									2,    // battHigh_worker
									2,    // angle_worker
									2,    // speed_worker
									2,    // currPos_worker
									2,    // navControl_worker
									2,    // destControl_worker
									1000, // display_worker
									10    // trace_worker
	};

	// Priorités des tâches. Grande priorité aux tâches continues
	int prios[THREAD_NUM] = {2, // cameraControl_worker
							 4, // camera_worker
							 4, // battery_worker
							 3, // battLow_worker
							 3, // battHigh_worker
							 4, // angle_worker
							 4, // speed_worker
							 3, // currPos_worker
							 3, // navControl_worker
							 3, // destControl_worker
							 1, // display_worker
							 1	// trace_worker
	};

	int i;
	for (i = 0; i < THREAD_NUM; ++i) {

		// Initialisation des sémaphores de synchronisation
		if (sem_init(&sem_syncs[i], 0, 0) != 0) {
			printf("Could not get init semaphore %d: %d\n", i, errno);
			return;
		}

		// Initialisation des arguments de chaque thread
		task_args[i].id        = i;                // id du thread
		task_args[i].semaphore = &sem_syncs[i];    // sémaphore de sync
		task_args[i].starttime = tp.tv_sec;        // temps de départ
		task_args[i].chid      = ChannelCreate(0); // Channel de communication
		task_args[i].period    = periods[i]; 	   // Période de la tâche

		if(-1 == task_args[i].chid) {
			printf("Could not create channel: %d\n", errno);
			return;
		}

		// Création de la tâche
		mySchedParam.sched_priority = prios[i]; // priorité de la tâche
		pthread_attr_setschedparam(&attrib, &mySchedParam);
		if (pthread_create(&tid[i], &attrib, main_worker, (void *)&task_args[i]) < 0) {
			cout << "Thread " << i << " creation failed" << endl;
			return;
		}

		// Création du timer associé
		mySchedParam.sched_priority = 5; // priorité max pour les timers
		pthread_attr_setschedparam(&attrib, &mySchedParam);
		if(0 != pthread_create(&pulseHandlers[i], &attrib,
					           task_pulse_handler, (void *)&task_args[i])) {
			printf("Could not create pulse thread: %d\n", errno);
			return;
		}
		if(0 != init_timer(&sigevents[i], &itimerspecs[i], &timers[i],
						   task_args[i].chid, periods[i])) {
			printf("Could not create timer: %d\n", errno);
			return;
		}
	}

	// Paramètres des partitions
	unsigned int sched_pol = SCHED_APS_SCHEDPOL_LIMIT_CPU_USAGE;
	sched_aps_create_parms sched_partitions[2]; // Partition pour continu et discret
	sched_aps_parms        sched_params[2];
	sched_aps_join_parms   sched_join_param[THREAD_NUM];

	// Création des partitions
	if(0 != create_partitions(sched_partitions, sched_params, &sched_pol)) {
		return;
	}
	// Assignation des tâches aux partitions
	if(0 != assign_partitions(sched_join_param, sched_partitions, tid, THREAD_NUM)) {
		return;
	}

	sleep(SIMU_TIME); // On endort le thread principal le temps de la simulation
	pm.dumpImage("./map.bmp"); // Chemin de la voiture

	// Cancel des timers et des tâches
	for (i = 0; i < THREAD_NUM; ++i) {
		pthread_cancel(tid[i]);
		pthread_cancel(pulseHandlers[i]);
	}

	// Fermeture fichier de données
	fclose(trace_data);

	free(sem_syncs);
	free(pulseHandlers);
	free(sigevents);
	free(itimerspecs);
	free(timers);
	free(task_args);
	free(tid);

	return;
}

/************************************************************
* Cette tâche lance correctement chaque tâche selon son id. *
* Nécessaire afin de pouvoir lancer les threads dans une    *
* boucle.                                                   *
************************************************************/
void * main_worker(void * data) {

	int task_id = ((thread_args_t *)data)->id;

	switch(task_id) {
		case 0:  cameraControl_worker(data); break;
		case 1:  camera_worker(data);        break;
		case 2:  battery_worker(data);       break;
		case 3:  battLow_worker(data);       break;
		case 4:  battHigh_worker(data);      break;
		case 5:  angle_worker(data);         break;
		case 6:  speed_worker(data);         break;
		case 7:  currPos_worker(data);       break;
		case 8:  navControl_worker(data);    break;
		case 9:  destControl_worker(data);   break;
		case 10: display_worker(data);       break;
		case 11: trace_worker(data);         break;
		default: break;
	}
	return NULL;

}

/*******************************************************************
* Type de tâche : contrôle 												   *
* Cette tâche envoie une commande à la tâche continue de la caméra *
* pour prendre une photo 										   *
*******************************************************************/
void cameraControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

    double delta = 0, x, y;

	while(1) {
		if (sem_wait(sync_sem) == 0) {
			// Lecture de la position
			pthread_mutex_lock(&mutDataCurrPos);
			x = myData.currPos.x;
			y = myData.currPos.y;
			pthread_mutex_unlock(&mutDataCurrPos);
			// Condition à 10 mètres
			while (delta < 10) {
				// Lecture de la position courante puis calcul de la différence
				pthread_mutex_lock(&mutDataCurrPos);
				delta = sqrt(pow((myData.currPos.x - x), 2) + pow((myData.currPos.y - y), 2));
				pthread_mutex_unlock(&mutDataCurrPos);
			}
			// On relâche la sémaphore et on réveille camera_worker
			if(0 != sem_post(&taskCamera_sync)) {
				printf("Could not post semaphore: %d\n", errno);
			}
			delta = 0;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}

/******************************************************************
* Type de tâche : continue
* Cette tâche représente la partie continue de la caméra et prend *
* une photo lorsque la tâche de contrôle relache la sémaphore     *
******************************************************************/
void camera_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	rgb_t image;
	while(1) {
		if (sem_wait(sync_sem) == 0) {
			// Sémaphore relâchée quand delta > 10 dans cameraControl_worker
			if(0 == sem_wait(&taskCamera_sync)) {
				pthread_mutex_lock(&mutDataCurrPos);
				image = pm.takePhoto(myData.currPos);
				pthread_mutex_unlock(&mutDataCurrPos);
			} else {
				printf("Task camera_worker could not get time: %d\n", errno);
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}


/**************************************************************
* Type de tâche : continue 
* Cette tâche représente la charge ou décharge de la batterie *
* Elle fait décroître la batterie en fonction de la vitesse   *
* de la voiture. Si la partie contrôle signale que la voiture *
* est en charge alors la tâche recharge la batterie.          *
* Cette tâche détecte également les niveaux de batterie et    *
* réveille les thread battLow_worker et battHigh_worker qui   *
* signalent la partie contrôle.                               *
**************************************************************/
void battery_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id, period;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;
	period    = ((thread_args_t*)data)->period;

	float speed_local, batt_local;

	while(1) {
		if (sem_wait(sync_sem) == 0) {
			if (inCharge)	{
				// Recharge de 5% par seconde
				pthread_mutex_lock(&mutDataBattLevel);
				myData.battLevel += (float)CONST_CHARGE * period / 1000 ;
				pthread_mutex_unlock(&mutDataBattLevel);
			}
			// Lecture de la vitesse
			pthread_mutex_lock(&mutDataSpeed);
			speed_local = myData.speed;
			pthread_mutex_unlock(&mutDataSpeed);
			// La batterie décharge même si la vitesse est nulle, car la voiture est
			// toujours en marche, donc se décharge
			pthread_mutex_lock(&mutDataBattLevel);
			myData.battLevel -= SIMU_ACCEL * ((float)COEFF_DECHARGE * speed_local + (float)CONST_DECHARGE) * ((float)period/1000);
			batt_local = myData.battLevel;
			pthread_mutex_unlock(&mutDataBattLevel);
			// Débloquer les sémaphores selon le niveau de batterie
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
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

/***************************************************************************
* Type de tâche : contrôle
* Cette tâche a pour unique rôle de changer la valeur de deux booléens     *
* représentant la batterie faible (lowBat) et la batterie élevée (highBat) *
***************************************************************************/
void battLow_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			if(0 == sem_wait(&taskBattlow_sync)) {
				lowBat  = true;
				highBat = false;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}	

/***************************************************************************
* Type de tâche : contrôle
* Cette tâche a pour unique rôle de changer la valeur de deux booléens     *
* représentant la batterie faible (lowBat) et la batterie élevée (highBat) *
***************************************************************************/
void battHigh_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			if(0 == sem_wait(&taskBatthigh_sync)) {
				highBat = true;
				lowBat  = false;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return;
}

/*************************************************************
* Type de tâche : continue                                            *
* Tâche  représentant la partie continue qui modifie l'angle *
* Pas forcement necessaire dans le programme mais c'est une  *
* representation plus correcte du reel. Elle va assigner     *
* l'angle commandé par la partie contrôle à l'angle réel     *
*************************************************************/
void angle_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataAngle);
			myData.angle = orderedAngle;
			pthread_mutex_unlock(&mutDataAngle);
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
}

/*************************************************************
* Type de tâche : continue                                            *
* Thread representant la partie continue qui modifie la      *
* vitesse en fonction de la commande de vitesse de la partie *
* contrôle                                                   *
*************************************************************/
void speed_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
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
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
}

/******************************************************
* Type de tâche : continue                                     *
* Cette tâche met à jour la position en fonction      *
* de la vitesse et de l'angle					      *
******************************************************/
void currPos_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id, period;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;
	period    = ((thread_args_t*)data)->period;

	float deltaX, deltaY, dist;

	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataSpeed);
			// Distance parcourue en fonction de la vitesse et la période
			// Prend en compte l'accélération de la simulation
		 // m                          km/h           ms
			dist = SIMU_ACCEL * 1000 * myData.speed * period / (1000 * 3600) ;
			pthread_mutex_unlock(&mutDataSpeed);
			pthread_mutex_lock(&mutDataAngle);
			// Calcul du delta à ajouter aux coordonnées
			deltaX = dist * cos(myData.angle * PI / 180);
			deltaY = dist * sin(myData.angle * PI / 180);
			pthread_mutex_unlock(&mutDataAngle);
			pthread_mutex_lock(&mutDataCurrPos);
			// Mise à jour
			myData.currPos.x += deltaX;
			myData.currPos.y += deltaY;
			pthread_mutex_unlock(&mutDataCurrPos);
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}	

/************************************************************************
* Cette tâche renvoie true si la distance entre currPos et nextStep est *
* inférieure à dist                                                     *
************************************************************************/
bool closeTo(coord_t currPos, coord_t nextStep, int dist) {
	return (sqrt(pow((nextStep.x - currPos.x),2) + pow((nextStep.y - currPos.y),2)) <= dist);
}

/***************************************************************************
* Cette fonction renvoie l'angle auquel la voiture doit s'orienter afin de *
* se diriger de currPos à nestPos                                          *
***************************************************************************/
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

/**************************************************************************
* Type de tâche : contrôle                                                         *
* Cette tâche représente la partie navigation de la voiture.              *
* C'est une machine à état où les états représentent l'état  courant de   *
* la voiture :                                                            *
*  - GOTO_DEST : la voiture se dirige vers la prochaine étape (à 50km/h)  *
*  - PRE_BATT_LOW : la voiture est en batterie faible, commande la partie *
*    continue de se mettre à 30 km/h et trouve la station la plus proche  *
*  - BATT_LOW : la voiture commande un angle pour se diriger vers la      *
*    station                                                              *
*  - CHARGING : la voiture est à la station, à 0km/h et se recharge       *
*  - PRE_OBSTACLE : la voiture a détecté un obstacle et s'arrête. On      *
*    démarre un timer                                                     *
*  - OBSTACLE : Lorsque le timer est écoulé la voiture redémarre en       *
*    de l'état précédent (BATT_LOW ou GOTO_DEST)                          *
**************************************************************************/
void navControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	coord_t currPos_local;
	auto start = std::chrono::system_clock::now();
	// has_obstacle est à true si pm.genObstacle() renvoie true
	bool has_obstacle = false;

	while(1) {
		if (sem_wait(sync_sem) == 0) {
			// Lecture de la position
			pthread_mutex_lock(&mutDataCurrPos);
			currPos_local = myData.currPos;
			pthread_mutex_unlock(&mutDataCurrPos);
			switch(state) {
				case GOTO_DEST:
					speedState = VIT50; // commande à 50km/h
					// Critique donc on teste en premier si la voiture est face à un
					// obstacle
					if (has_obstacle && closeTo(currPos_local, obstacle, DIST_OBSTACLE)) {
						state = PRE_OBSTACLE;
						break;
					}
					// Test si la batterie est faible
					if (lowBat) {
						state = PRE_BATT_LOW;
						break;
					} else {
						// Test si la destination est atteinte
						if (destReached) {
							// Destination atteinte donc on en génère une nouvelle
							nb_destReached += 1;
							pm.genDest(currPos_local, dest);
							pm.genWp(currPos_local, dest, nextStep);
							nb_stepReached = 0; 
							destReached = false;
						} else {
							// Étape atteinte ?
							if (closeTo(currPos_local, nextStep, DIST_STEP)) {
								nb_stepReached += 1;
								// On génère la prochaine étape
								pm.genWp(currPos_local, dest, nextStep);
								// Generation aléatoire d'obstacle
								has_obstacle = pm.genObstacle(currPos_local,
															  nextStep,
															  obstacle);
							} else {
								// Sinon on met à jour l'angle
								orderedAngle = getAngle(currPos_local, nextStep);
							}
						}
					}
					break;
				case PRE_BATT_LOW:
					speedState = VIT30; // Commande à 30 km/h
					pm.getClosestStation(currPos_local, stationPos);
					// Il peut également y avoir un obstacle sur le chemin de la station
					has_obstacle = pm.genObstacle(currPos_local, stationPos, obstacle);
					state = BATT_LOW;
					break;
				case BATT_LOW:
					// Mise à jour de l'angle
					orderedAngle = getAngle(currPos_local, stationPos);
					// Détection d'obstacle
					if (has_obstacle && closeTo(currPos_local, obstacle, DIST_OBSTACLE)) {
						state = PRE_OBSTACLE;
						break;
					}
					if (closeTo(currPos_local, stationPos, DIST_STEP))
						state = CHARGING;
					break;
				case CHARGING:
					speedState = VIT0; // Commande à 0km/h
					inCharge = true;
					if (highBat) {
						// On genere une nouvelle etape a partir de la station
						// avant de retourner à l'état GOTO_DEST
						pm.genWp(currPos_local, dest, nextStep);
						inCharge = false;
						state = GOTO_DEST;
					}
					break;
				case PRE_OBSTACLE:
					speedState = VIT0; // Commande à 0km/h
					// Démarrage du timer
					start = std::chrono::system_clock::now();
					state = OBSTACLE;
					break;
				case OBSTACLE:
					auto end = std::chrono::system_clock::now();
					std::chrono::duration<double> elapsed_time = end - start;
					// délai de 5 secondes avant de repartir
					if (elapsed_time.count() > 1) {
						if (lowBat)
							state = BATT_LOW;
						else
							state = GOTO_DEST;
						has_obstacle = false;
					}
					break;
			}
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

/**********************************************************************
* Type de tâche : contrôle 
* Cette tâche détermine quand la voiture est arrivée à la destination *
**********************************************************************/
void destControl_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	coord_t currPos_local;
	while (1) {
		if (sem_wait(sync_sem) == 0) {
			pthread_mutex_lock(&mutDataCurrPos);
			currPos_local = myData.currPos;
			pthread_mutex_unlock(&mutDataCurrPos);
			if (closeTo(currPos_local, dest, DIST_STEP))
				destReached = true;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}

/**************************************************************
* Fonction servant à l'affichage de l'état sur display_worker *
**************************************************************/
char * getCarState(enum carState state) {
	switch (state) {
	  case GOTO_DEST:    return (char *)"GOTO_DEST";
	  case PRE_BATT_LOW: return (char *)"PRE_BATT_LOW";
	  case BATT_LOW:     return (char *)"BATT_LOW";
	  case CHARGING:     return (char *)"CHARGING";
	  case PRE_OBSTACLE: return (char *)"PRE_OBSTACLE";
	  case OBSTACLE:     return (char *)"OBSTACLE";
	  default:           return (char *)"UNKNOWN";
   }
}

/***************************************************************************
* Type de tâche : autre                                                             * 
* Cette tâche sert à afficher les information de la voiture sur la console *
***************************************************************************/
void display_worker(void * data) {

	sem_t* sync_sem;
	uint32_t task_id;
	sync_sem  = ((thread_args_t*)data)->semaphore;
	task_id   = ((thread_args_t*)data)->id;

	float bat, speed_local, angle_local;
	double x, y;

	while (1)	{
		if (sem_wait(sync_sem) == 0) {

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
				 << "  currPos.x: "  << setw(9) << x              << "   currPos.y: "  << y              << "\n"
				 << " nextStep.x: "  << setw(9) << nextStep.x     << "  nextStep.y: "  << nextStep.y     << "\n"
				 << "     dest.x: "  << setw(9) << dest.x         << "      dest.y: "  << dest.y         << "\n"
				 << "  station.x: "  << setw(9) << stationPos.x   << "   station.y: "  << stationPos.y   << "\n"
				 << " obstacle.x: "  << setw(9) << obstacle.x     << "  obstacle.y: "  << obstacle.y     << "\n"
				 << "    nb step: "  << setw(9) << nb_stepReached << "     nb dest: "  << nb_destReached << "\n"
				 << "*******************************************" << endl;
		} else {
			printf("Task %d could not wait semaphore: %d\n", task_id, errno);
		}
	}
	return; 
}	

/**************************************************************************
* Type de tâche : autre
* Cette tâche sert à la trace de données (historique) de la voiture. Elle *
* écrit dans un fichier de type csv.                                      *
**************************************************************************/
void trace_worker(void * data) {

	sem_t* sync_sem;
	sync_sem  = ((thread_args_t*)data)->semaphore;

	auto start = std::chrono::system_clock::now();

	while (1)	{
		if (sem_wait(sync_sem) == 0) {
			auto end = std::chrono::system_clock::now();
			std::chrono::duration<double> elapsed_time = end - start;
			fprintf(trace_data,
					"%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%d,%d\n",
					elapsed_time.count() * SIMU_ACCEL,
					myData.battLevel,
					myData.speed,
					myData.angle,
					myData.currPos.x,
					myData.currPos.y,
					nextStep.x,
					nextStep.y,
					dest.x,
					dest.y,
					obstacle.x,
					obstacle.y,
					stationPos.x,
					stationPos.y,
					nb_stepReached,
					nb_destReached);
		}
	}

	return;
}

/* vim: set ts=4 sw=4 tw=90 noet :*/

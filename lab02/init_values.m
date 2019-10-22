%Variables initiales (modifiables)
INIT_NIV_BAT = 100;
DISTANCE_MAX_LAT = 2000;
DISTANCE_MAX_LON = 6000;
TIME_TO_CHARGE_M = 20;
VITESSE_MAX_KMH = 50;
BAT_AUTONOMY_KM = 50;


%Variables calculées à partir des précédentes (non modifiables)
%INIT_PHASE_SOL = pi*(4*INIT_AVANCEMENT_JOUR - 1)/2;
%FREQ_SOL = 1/(PERIODE_SOL_H*3600);
TIME_TO_CHARGE_S = TIME_TO_CHARGE_M*60;
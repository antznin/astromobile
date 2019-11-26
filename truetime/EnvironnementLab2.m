clear all
close all
clc

cd ..
cd Documentation
cd truetime-2.0
run('init_truetime.m')  %pour initialiser les variables truetime
cd ..
cd ..
cd Controleur


mex -setup:'C:\Program Files\MATLAB\R2019a\bin\win64\mexopts\mingw64_g++.xml' C++

%make_truetime          %pour la compilation de la librairie truetime
ttmex ctrlauto.cpp     %pour la compilation du code du fichier c++

%truetime

run('init_values.m')    %pour initialiser les valeurs du système
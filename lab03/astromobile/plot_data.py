import matplotlib.pyplot as plt
import numpy as np

# Lecture des données et stockage dans data
data_f = open("./data.csv", "r")
first = data_f.readline().replace("\n", "").split(",")
data = []
[data.append([float(el)]) for el in first]

length = len(first)

for line in data_f:
    line_list = line.replace("\n", "").split(",")
    [data[i].append(float(line_list[i])) for i in range(length)]

data_f.close()

# Les données d'étapes etc son rendues uniques
def unique(seq1, seq2):
   checked1 = []
   checked2 = []
   last_el = None
   for i in range(len(seq1)):
       if seq1[i] != last_el:
           checked1.append(seq1[i])
           checked2.append(seq2[i])
           last_el = seq1[i]
   return checked1, checked2

for i in range(6,14,2):
    data[i], data[i+1] = unique(data[i], data[i+1])

print(data[12])

# Plot de la carte
plt.figure(1)
plt.scatter(data[4], data[5],c=data[1], s=4)
cbar = plt.colorbar()
cbar.set_label('Niveau de la batterie')
plt.plot(data[6][:-1], data[7][:-1], 's', c='fuchsia', markersize=4, label="Étapes")
plt.plot(data[10][1:], data[11][1:], 'go', markersize=4, label="Obstacles")
plt.plot(data[12][1:], data[13][1:], 'co', markersize=4, label="Stations")
plt.plot(data[8][:-1], data[9][:-1], 'star', c=yellow, markersize=5, label="Destinations")
plt.xlabel("$x$")
plt.ylabel("$y$")
plt.xticks(fontsize=8)
plt.yticks(fontsize=8)
plt.legend()
plt.grid()
plt.show()

# Plot de la vitesse
plt.figure(2)
plt.plot(data[0], data[2])
plt.grid()
plt.xlabel("Temps (s)")
plt.ylabel("Vitesse (km/h)")
plt.show()

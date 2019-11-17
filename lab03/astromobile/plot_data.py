import matplotlib.pyplot as plt

data_f = open("./data.csv", "r")
first = data_f.readline().replace("\n", "").split(",")
data = []
[data.append([float(el)]) for el in first]

length = len(first)

for line in data_f:
    line_list = line.replace("\n", "").split(",")
    [data[i].append(float(line_list[i])) for i in range(length)]

data_f.close()

print(data[0][0:50])
print(data[1][0:50])

#f, (ax1, ax2) = plt.subplots(1, 2)
plt.plot(data[0], data[1])
plt.xlabel("$t$ ($s$)")
plt.ylabel("Niveau de batterie (%)")
plt.grid()
plt.legend()
plt.savefig("./batt.jpg")
plt.show()

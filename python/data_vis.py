import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
from scipy.optimize import curve_fit

with open("../outputs/output.txt") as f:
    raw_data = f.readlines()

data = []
for d in raw_data:
    if "FALSE" in d:
        print(d)
        continue

    x = d.split(" ")
    x = x[1:]
    x[-1] = x[-1].replace("\n","")
    data_x = np.ones(10, dtype=float)*30
    for t in range(len(x)):
        if ("out" in x[t]):
            data_x[t] = 30
        else:
            data_x[t] = float(x[t])
    data.append(data_x)



plt.rcParams.update({'font.size': 15})
plt.subplots_adjust(bottom=0.15)
fig, ax = plt.subplots(figsize = (7, 7))
data = np.array(data)
x = data[:,2]
y = data[:,5]
# ax.scatter(x, y, s=20, alpha=0.3)
# def func(x):
#     return 0.5363*(x**0.9678)
# ax.plot(x, func(x), 'k-', label="Fitted Curve", linewidth=4)
# ax.text(0.002, 25, r"$y=0.5363x^{0.9678}$", size=15)

# plt.xlabel('ITA-CBS origin total time')
# plt.ylabel('ITA-CBS remake total time')
# plt.xscale('log')
# plt.yscale('log')
# plt.show()

x = data[:,2]
y = data[:,5]
idx = ~((x >= 30) + (y >= 30))
y1 = data[idx, 3]
y2 = data[idx, 4]
y3 = data[idx, 5]
for a,b,c in zip(y1, y2, y3):
    print(a,",",b, ",",c)

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

dict_BBmap = {}
list_BBname = []

df = pd.read_excel("/home/mpl/Downloads/NGS_analysis_FCsort_20221118(1).xlsx", sheet_name="Sheet3", header=None)
df_matrix = df.to_numpy()

rowN, columnN = df_matrix.shape

for j in range(0, columnN, 2):
    BBnames = df_matrix[:, j]
    BBcounts = df_matrix[:, j+1]
    for i in range(len(BBnames)):
        BBname = BBnames[i]
        if (np.isnan(BBname)):
            continue

        BBcount = BBcounts[i]
        if (np.isnan(BBcount)):
            BBcount = 0

        if (BBname not in dict_BBmap.keys()):
            dict_BBmap[BBname] = [0, 0, 0]

        dict_BBmap[BBname][int(j / 2)] += BBcount


keys = list(dict_BBmap.keys())
keys.sort()

plotData1, plotData2, plotData3 = [], [], []

index = 0
xlabel = []
for i in range(len(keys)):
    BBcounts = dict_BBmap[keys[i]]
    
    skip = True
    for count in BBcounts:
        if (count > 3):
            skip = False
    
    if (skip):
        continue

    total = sum(BBcounts)
    plotData1.append(BBcounts[0] / total)
    plotData2.append(BBcounts[1] / total)
    plotData3.append(BBcounts[2] / total)

    if (plotData1[index] != 0):
        plt.text(index, plotData1[index] / 2, str(round(plotData1[index] * 100, 1)) + "%", ha='center', fontsize = 'small')
    if (plotData2[index] != 0):
        plt.text(index, plotData2[index] / 2 + plotData1[index], str(round(plotData2[index] * 100, 1)) + "%", ha='center', fontsize = 'small')
    if (plotData3[index] != 0):
        plt.text(index, plotData3[index] / 2 + plotData1[index] + plotData2[index], str(round(plotData3[index] * 100, 1)) + "%", ha='center', fontsize = 'small')
    index += 1
    xlabel.append(int(keys[i]))


p1 = plt.bar(np.arange(index), plotData1, color ='#d62728')
p2 = plt.bar(np.arange(index), plotData2, bottom=plotData1)
p3 = plt.bar(np.arange(index), plotData3, bottom=[plotData1[i]+plotData2[i] for i in range(len(plotData1))] )


 
plt.ylabel('Count ratio')
plt.title('BB3 count ratio')
plt.xticks(np.arange(index), xlabel)
# plt.yticks(np.arange(0, 81, 20))
plt.legend((p3[0], p2[0], p1[0]), ('40S', '39H', '25C'))
plt.show()


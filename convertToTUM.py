import pandas as pd
import sys
import numpy as np

path = sys.argv[1]

table = pd.read_csv(path, skiprows=7, header=None)
table_np = table.to_numpy()
table_np = table_np[:, [1, 6, 7, 8, 2, 3, 4, 5]]

cur_time = 0.0
for i in range(table_np.shape[0]):
    table_np[i, 0] = cur_time
    cur_time += 0.008333

np.savetxt("gt.txt", table_np, delimiter=' ')
print(table_np)
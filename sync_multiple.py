import pandas as pd
import sys
import numpy as np
from scipy.spatial.transform import Rotation as sciRt

ref_table = np.loadtxt(sys.argv[1])

sync_tables = []
for i in range(2, len(sys.argv)):
    sync_tables.append(np.loadtxt(sys.argv[i]))

sync_file_num = len(sys.argv) - 2
cur_idxes = [0] * sync_file_num


result_table = np.zeros(ref_table.shape)
ref_counter = 0

for i in range(ref_table.shape[0]):
    ref_time = ref_table[i, 0]
    added = False
    for j in range(len(sync_tables)):
        cur_idx = cur_idxes[j]
        cur_table = sync_tables[j]

        if (cur_idx >= cur_table.shape[0]):
            continue
        cur_time = cur_table[cur_idx, 0]
        
        if (not added and abs(ref_time - cur_time) < 0.03):
            added = True
            result_table[ref_counter,:] = ref_table[i, :]
            ref_counter += 1
    print(i)
    for j in range(len(sync_tables)):
        cur_table = sync_tables[j]
        if (cur_idxes[j] >= cur_table.shape[0]):
            continue
        while(ref_time - cur_table[cur_idxes[j], 0] > 0):
            cur_idxes[j] += 1
            if (cur_idxes[j] >= cur_table.shape[0]):
                break
    
print(ref_counter)
np.savetxt("odom_sync.txt", result_table[:ref_counter, :], delimiter=' ')

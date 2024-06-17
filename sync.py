import pandas as pd
import sys
import numpy as np
from scipy.spatial.transform import Rotation as sciRt

gt_path = sys.argv[1]

gt_table = pd.read_csv(gt_path, skiprows=7, header=None)
gt_table_np = gt_table.to_numpy()
gt_table_np = gt_table_np[:, [1, 6, 7, 8, 2, 3, 4, 5]]

gt_active = 0
gt_prev = gt_table_np[0,:]
for i in range(1, gt_table_np.shape[0]):
    gt_prev_t = gt_prev[1:4].reshape(3, 1)
    gt_prev_q = sciRt.from_quat(gt_prev[4:8])
    gt_cur_t = gt_table_np[i, :][1:4].reshape(3, 1)
    gt_cur_q = sciRt.from_quat(gt_table_np[i, :][4:8])

    gt_rel_t = gt_prev_q.inv().as_matrix() @ (gt_cur_t - gt_prev_t)
    gt_prev_t = gt_cur_t
    gt_prev_q = gt_cur_q

    if (np.linalg.norm(gt_rel_t) > 10e-3):
        gt_active = i
        break


    # print(gt_table_np[i,0], gt_rel_t.reshape(1,3))


odom_path = sys.argv[2]
odom_table_np = np.loadtxt(odom_path)
odom_active = 0
odom_prev = odom_table_np[0,:]
for i in range(1, odom_table_np.shape[0]):
    odom_prev_t = odom_prev[1:4].reshape(3, 1)
    odom_prev_q = sciRt.from_quat(odom_prev[4:8])
    odom_cur_t = odom_table_np[i, :][1:4].reshape(3, 1)
    odom_cur_q = sciRt.from_quat(odom_table_np[i, :][4:8])

    odom_rel_t = odom_prev_q.inv().as_matrix() @ (odom_cur_t - odom_prev_t)
    odom_prev_t = odom_cur_t
    odom_prev_q = odom_cur_q
    if (np.linalg.norm(odom_rel_t) > 10e-3):
        odom_active = i
        break
    # print(odom_table_np[i,0], odom_rel_t.reshape(1,3))


add_num = 30
odom_active_time = odom_table_np[odom_active, 0]
odom_start_idx = odom_active + add_num
odom_start_time = odom_table_np[odom_start_idx, 0]
odom_rel_time = odom_active_time - odom_start_time

gt_active_time = gt_table_np[gt_active, 0]
gt_start_time = gt_active_time - odom_rel_time


odom_sync = odom_table_np[odom_start_idx:, :]
gt_sync = np.zeros((0, 8))

print("odom_start_time:", odom_start_time)
print("odom_active_time:", odom_active_time)
print("gt_start_time:", gt_start_time)
print("gt_active_time:", gt_active_time)

count = 0

fix_r = sciRt.from_matrix(np.array([[0,-1,0],[1,0,0],[0, 0, 1]])) * sciRt.from_matrix(np.array([[0,-1,0],[0,0,1],[-1, 0, 0]]))
for i in range(0, gt_table_np.shape[0]):
    cur_time = gt_table_np[i, 0]
    if (cur_time > gt_start_time) and (not np.isnan(gt_table_np[i, 1])):
        sync_time = odom_start_time + cur_time - gt_start_time
        cur_q = sciRt.from_quat(gt_table_np[i, 4:8])
        cur_t = gt_table_np[i, 1:4].reshape(3,1)
        cur_q = fix_r * cur_q * fix_r.inv()
        cur_t = fix_r.as_matrix() @ cur_t
        cur_element = np.append(cur_t.flatten(), cur_q.as_quat())
        cur_element = np.append(sync_time, cur_element).reshape(1, -1)
        print(i, "/", gt_table_np.shape[0])
        gt_sync = np.concatenate((gt_sync,cur_element))
        count+=1


np.savetxt("gt_sync.txt", gt_sync, delimiter=' ')
np.savetxt("odom_sync.txt", odom_sync, delimiter=' ')

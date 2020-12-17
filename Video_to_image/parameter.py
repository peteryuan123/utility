import yaml
import os
import numpy as np
import math
np.set_printoptions(suppress=True)
def get_yaml_data(yaml_file):

    file = open(yaml_file, 'r', encoding="utf-8")
    file_data = file.read()
    file.close()
    
    data = yaml.load(file_data)

    return data

current_path = os.path.abspath(".")
yaml_path1 = os.path.join(current_path, "calib_result/flip/0_1_2_3_4_5/camchain-output.yaml")

current_path = os.path.abspath(".")
yaml_path2 = os.path.join(current_path, "calib_result/flip/below_0/camchain-output.yaml")
cameras1 = get_yaml_data(yaml_path1)
cameras2 = get_yaml_data(yaml_path2)
file_name = "parameter.txt"
m_file = open(file_name,"w+")

for i in range(6):
    cam = "cam"+str(i)
    intri = np.zeros([3,3])
    xi = cameras1[cam]['intrinsics'][0]
    fu = cameras1[cam]['intrinsics'][1]
    fv = cameras1[cam]['intrinsics'][2]
    pu = cameras1[cam]['intrinsics'][3]
    pv = cameras1[cam]['intrinsics'][4]
    intri[0,0] = fu
    intri[0,1] = math.cos(xi)*fu
    intri[0,2] = pu
    intri[1,1] = fv
    intri[1,2] = pv
    intri[2,2] = 1
    m_file.write(cam + " intrinsic:\n")
    m_file.write(str(intri))
    m_file.write("\n")

for i in range(2):
    cam = "cam"+str(i)
    if (i==0):
        fu = cameras2[cam]['intrinsics'][0]
        fv = cameras2[cam]['intrinsics'][1]
        pu = cameras2[cam]['intrinsics'][2]
        pv = cameras2[cam]['intrinsics'][3]
        intri = np.zeros([3,3])
        intri[0,0] = fu
        intri[0,2] = pu
        intri[1,1] = fv
        intri[1,2] = pv
        intri[2,2] = 1
        m_file.write("below intrinsics:\n")
        m_file.write(str(intri))
        m_file.write("\n")
    else:
        m_file.write("transformation from 0 to below:\n")
        m_file.write(str(cameras2[cam]["T_cn_cnm1"]))
        m_file.write("\n")

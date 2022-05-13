from cmath import pi
from numpy import double, float64
import numpy as np
from scipy.spatial.transform import Rotation as sciRt
import readline
import sys

def main():
    while(True):
        q0_array = input("q0:").split()
        q0_array = list(map(double, q0_array))
        print(q0_array)
        q1_array = input("q1:").split()
        q1_array = list(map(double, q1_array))
        print(q1_array)

        q0 = sciRt.from_quat(q0_array)
        q1 = sciRt.from_quat(q1_array)

        result = q0*q1

        print("quat:", result.as_quat().transpose())
        print("euler:", result.as_euler("XYZ").transpose()*180/pi)
        print("rotv:", result.as_rotvec().transpose())
        print("norm:", np.linalg.norm(result.as_rotvec())*180/pi, "axis:", result.as_rotvec().transpose()/np.linalg.norm(result.as_rotvec()))
        print("\n")
        print("------------------")
        print("\n")






if __name__ == "__main__":
    main()
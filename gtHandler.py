import sys
import pandas as pd
from scipy.spatial.transform import Rotation as sciRt
import numpy as np
import PySimpleGUI as sg
import os.path

def main():

    
    file_list_column = [
        [
            sg.Text("GT File"),
            sg.In(size=(25, 1), enable_events=True, key="-FILE-"),
            sg.FileBrowse(initial_folder=sys.argv[1]),
        ],
        [
            sg.Listbox(
                values=[], enable_events=True, size=(40, 20), key="-STAMP LIST-"
            )
        ],
    ]

    image_viewer_column = [
        [sg.Text("Fixed Pose:")],
        [sg.Text(size=(100, 1), key="-fixed rotation-")],
        [sg.Text(size=(100, 1), key="-fixed translation-")],
        [sg.Text("Global Pose:")],
        [sg.Text(size=(100, 1), key="-global rotation-")],
        [sg.Text(size=(100, 1), key="-global translation-")],
        [sg.Button("Fixed")],
    ]
    
    layout = [
        [
            sg.Column(file_list_column),
            sg.VSeperator(),
            sg.Column(image_viewer_column),
        ]
    ]

    Qbi = sciRt.from_quat([1, 0, 0, 0])
    tbi = np.array([0.038, -0.008, 0.065]).reshape(3,1)

    print(Qbi.as_matrix())
    fixed_pose = [sciRt.from_quat([0, 0, 0, 1]), np.array([0, 0, 0]).reshape(3, 1)]
    # print(fixed_pose)

    gt_map = {}
    window = sg.Window("GT Viewer", layout)
    while True:
        event, values = window.read()
        if event == "Exit" or event == sg.WIN_CLOSED:
            break
        if event == "-FILE-":
            gt_path = values["-FILE-"]
            try:
                gt_table = pd.read_csv(gt_path)
                for i in range(len(gt_table)):
                    sec = str(int(gt_table.iloc[i]["#sec"]))
                    nsec = str(int(gt_table.iloc[i]["nsec"]))
                    stamp = sec + "." + nsec
                    t = np.array([gt_table.iloc[i]["x"], gt_table.iloc[i]["y"], gt_table.iloc[i]["z"]]).reshape(3,1)
                    R = sciRt.from_quat([gt_table.iloc[i]["qx"], gt_table.iloc[i]["qy"], gt_table.iloc[i]["qz"], gt_table.iloc[i]["qw"]])
                    gt_map[stamp] = (R, t)
                stamp_list = gt_map.keys()
            except:
                stamp_list = []

            window["-STAMP LIST-"].update(stamp_list)
        elif event == "Fixed":
            Qwb = gt_map[values["-STAMP LIST-"][0]][0]
            twb = gt_map[values["-STAMP LIST-"][0]][1]
            Qwi = Qwb*Qbi
            twi = Qwb.as_matrix()@tbi + twb
            fixed_pose[0], fixed_pose[1] = Qwi, twi
            cur_q = fixed_pose[0].as_quat()
            cur_t = fixed_pose[1]
            info_q = "Q(x, y, z, w): " + format(cur_q[0], ".7f") + " ," + format(cur_q[1], ".7f")  + " ," + format(cur_q[2], ".7f")  + " ," + format(cur_q[3], ".7f") 
            info_t = "t(x, y, z): " + format(cur_t[0][0], ".7f") + "," + format(cur_t[1][0], ".7f") + "," + format(cur_t[2][0], ".7f")
            window["-fixed rotation-"].update(info_q)
            window["-fixed translation-"].update(info_t)

        elif event == "-STAMP LIST-":  # A file was chosen from the listbox
            Qwb = gt_map[values["-STAMP LIST-"][0]][0]
            twb = gt_map[values["-STAMP LIST-"][0]][1]
            Qwi = Qwb*Qbi
            twi = Qwb.as_matrix()@tbi + twb
            rel_R = fixed_pose[0].inv()*Qwi
            rel_t = fixed_pose[0].inv().as_matrix()@(twi - fixed_pose[1])
            rel_R = rel_R.as_quat()
            print(rel_R)
            print(rel_t.T)
            print("-------")
            # print(rel_R)
            # print(rel_t)

            info_q = "Q(x, y, z, w): " + format(rel_R[0], ".7f") + " ," + format(rel_R[1], ".7f")  + " ," + format(rel_R[2], ".7f")  + " ," + format(rel_R[3], ".7f")
            info_t = "t(x, y, z): " + format(rel_t[0][0], ".7f") + " ," + format(rel_t[1][0], ".7f") + " ," + format(rel_t[2][0], ".7f")
            window["-global rotation-"].update(info_q)
            window["-global translation-"].update(info_t)
  

    window.close()

if __name__ == "__main__":
    main()
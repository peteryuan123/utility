import cv2
import glob
import numpy as np
import sys


def main():
    if (len(sys.argv) != 2):
        print("usage python3 img2video.py [img_dir]")
        return
    
    img_dir = glob.glob(sys.argv[1]+"/*")
    img_dir.sort()

    video_name = "output.mp4"
    fourcc = cv2.VideoWriter_fourcc('D', 'I', 'V', 'X')
    out_video = cv2.VideoWriter(video_name, fourcc, 30, (752,480))

    for name in img_dir:

        img = cv2.imread (name)
        out_video.write (img)

    out_video.release()

if __name__ == '__main__':
    main()
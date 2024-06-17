from distutils import extension
import glob
import sys

from matplotlib import image

if (len(sys.argv) != 2):
    print("usage python3 getAllTimes.py [image_folder_path + extension] eg. xx/xx/*.jpg")

print(sys.argv[1])
image_folder_path = sys.argv[1]
images_name = glob.glob(image_folder_path)
images_name.sort()

with open("times.txt", mode="w") as file:
    for name in images_name:
        name = name.split("/")[-1]
        time = name.split(".")[0]
        print(time, file=file)

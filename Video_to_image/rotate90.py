import numpy as np
import cv2
import os


path = 'image/not_flip/below_0_5/cam5'
file_names = []
img_list = []
for filename in os.listdir(path):
    img = cv2.imread(os.path.join(path,filename))
    print(os.path.join(path,filename))
    img_list.append(img)
    file_names.append(filename)

os.mkdir("cam5")

for i in range(len(img_list)):
    img = img_list[i]
    img = cv2.transpose(img)
    img = cv2.flip(img,0)
    cv2.imwrite("cam5/"+file_names[i],img)

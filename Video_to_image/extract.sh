#!/bin/bash
# mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image
# mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam0
#mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam1
#mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam2
#mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam3
#mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam4
#mkdir /home/peteryuan/Desktop/calib/video_to_image/build/image/cam5

num=1520526024123132022;
num1=0
for file in /home/peteryuan/Desktop/calib/video_to_image/build/output/cam0/*
do
    mv /home/peteryuan/Desktop/calib/video_to_image/build/output/cam0/$num1.png /home/peteryuan/Desktop/calib/video_to_image/build/output/cam0/$num.png
    mv /home/peteryuan/Desktop/calib/video_to_image/build/output/cam1/$num1.png /home/peteryuan/Desktop/calib/video_to_image/build/output/cam1/$num.png
    mv /home/peteryuan/Desktop/calib/video_to_image/build/output/cam2/$num1.png /home/peteryuan/Desktop/calib/video_to_image/build/output/cam2/$num.png
    # cp $file/origin_1.jpg /home/peteryuan/Desktop/calib/video_to_image/build/image/cam1/$num.jpg
    # cp $file/origin_2.jpg /home/peteryuan/Desktop/calib/video_to_image/build/image/cam2/$num.jpg
    # cp $file/origin_3.jpg /home/peteryuan/Desktop/calib/video_to_image/build/image/cam3/$num.jpg
    # cp $file/origin_4.jpg /home/peteryuan/Desktop/calib/video_to_image/build/image/cam4/$num.jpg
    # cp $file/origin_5.jpg /home/peteryuan/Desktop/calib/video_to_image/build/image/cam5/$num.jpg
    num=$(($num+250008000))
    num1=$(($num1+1))
done
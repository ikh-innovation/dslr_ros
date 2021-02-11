#!/bin/sh
echo "Broadcast DSLR rawvideo on /dev/vide0"
gphoto2 --stdout --capture-movie | ffmpeg -i - -vcodec rawvideo -pix_fmt yuyv422 -threads 12 -f v4l2 /dev/video0

#!/bin/bash

dir_img="mapimages_sim"

#temp=$$
#mkdir $temp
#files=$(ls -1 $dir_img/*.png | cut -d '/' -f 2 | sort -n)
#
#ii=0
#for i in $files; do 
#	index=$(printf "%05d" $ii)
#	convert $dir_img/$i $temp/$index.gif; 
#	ii=$((ii+1))
#done
#
#convert -delay 10 -loop 0 $temp/*.gif anim.gif

convert -delay 10 -loop 0 $dir_img/*.png anim.gif

rm -rf $temp

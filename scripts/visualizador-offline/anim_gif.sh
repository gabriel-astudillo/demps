#!/bin/bash

# Crea un archivo GIF animado a partir de las imágenes PNG 
# de la simulación, generados por el script "make_maps.pl"

# DEPENDENCIAS:
#    comando "convert" : ImageMagick
#    comando "gifsicle": https://www.lcdf.org/gifsicle/, 
#                        debian: apt-get install gifsicle


dir_img="mapimages_sim"

echo "Creando gif"
convert -delay 10 -loop 0 $dir_img/*.png anim.gif

echo "Optimizando gif"
gifsicle -i anim.gif -O3 --colors 128 -o anim.gif 

#rm -rf $temp

#!/bin/bash

# Crea un archivo GIF animado a partir de las imágenes PNG 
# de la simulación, generados por el script "make_maps.pl"

# DEPENDENCIAS:
#    comando "convert" : ImageMagick
#    comando "gifsicle": https://www.lcdf.org/gifsicle/, 
#                        debian: apt-get install gifsicle

#BASEDIR=$(dirname "$0")

BASEDIR=$(readlink -f $0)
BASEDIR=$(dirname $BASEDIR)

dir_img="mapimages_sim"

echo "Creando gif"
convert -delay 10 -loop 0 $BASEDIR/$dir_img/*.png $BASEDIR/anim.gif

echo "Optimizando gif"
gifsicle -i $BASEDIR/anim.gif -O3 --colors 128 -o $BASEDIR/anim.gif 

#rm -rf $temp

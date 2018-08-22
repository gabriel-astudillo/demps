# Scripts para visualizar los archivos log de DEMPS*

Lenguaje: PERL
Dependencias: 

* GD
* Tk-MultiMediaControls
* Script ```plot_latlong.pl``` ubicado en el directorio superior

## Procedimiento

1) Crear los archivos imagen:
```
./make_maps.pl -r ../../results
```
donde ```../../results``` es el directorio de los logs de DEMPS.
Los archivos PNG resultantes se almacenan en el directorio ```mapimages_sim``` (si no existe, el script lo crea). Se recomienda eliminar los archivos existentes si este directorio existe.

2) Para obtener una visualización temporal, ejecutar:
```
./anim.pl
```
Los archivos los carga desde el directorio ```mapimages_sim```. El archivo ```global.pm``` tiene la configuración global utilizada por ambos scripts.

Tambien se puede utilizar el script:
```
./anim_gif.sh
```
Este script utiliza el comando ```convert``` de la librería ImageMagick para crear un gif animado con la secuencia de imagénes del directorio ```mapimages_sim```.

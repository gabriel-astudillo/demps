# Dependencias

## Geographic lib

1) Bajar de https://geographiclib.sourceforge.io, descomprimir y entrar al directorio.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
../configure
make
make install
```
3) Incluir la biblioteca ```<GeographicLib/LocalCartesian.hpp>```. En el linker, agregar ```-lGeographic```

## CGAL

https://www.cgal.org: The Computational Geometry Algorithms Library

Versión: ≥ 4.10

* Linux: ```apt-get install libcgal-dev```
* Mac: ```port install cgal```

## OSRM backend


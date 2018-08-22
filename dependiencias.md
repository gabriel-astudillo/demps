# Dependencias

## Geographic lib

1) Bajar de https://geographiclib.sourceforge.io, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
../configure
make
make install (!)
```
(!) Necesita permisos de superusario

3) Incluir la biblioteca ```<GeographicLib/LocalCartesian.hpp>```. En el linker, agregar ```-lGeographic```

## CGAL

https://www.cgal.org: The Computational Geometry Algorithms Library

Versión: ≥ 4.10

* Linux: ```apt-get install libcgal-dev```
* Mac: ```port install cgal```

## OSRM backend

1) Bajar de https://github.com/Project-OSRM/osrm-backend/releases, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
cmake --build . --target intall  (!)
```
(!) Necesita permisos de superusario

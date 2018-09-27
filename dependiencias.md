# Dependencias

## JSON

JSON for Modern C++: https://github.com/nlohmann/json

1) Bajar la biblioteca desde https://github.com/nlohmann/json/blob/develop/single_include/nlohmann/json.hpp
2) Incluirla como:
```
#include "json.hpp"
using json = nlohmann::json;
```


## Geographic lib

"GeographicLib is a small set of C++ classes for performing conversions between geographic, UTM, UPS, MGRS, geocentric, and local cartesian coordinates", https://geographiclib.sourceforge.io

1) Bajar de https://geographiclib.sourceforge.io, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar la librería en el directorio por omisión ```/usr/local/```:
```
mkdir build
cd build
../configure
make
make install (!)
```
(!) Necesita permisos de superusario.

3) Incluir la biblioteca ```<GeographicLib/LocalCartesian.hpp>```. En el linker, agregar ```-lGeographic```

## CGAL

The Computational Geometry Algorithms Library, https://www.cgal.org

Versión: ≥ 4.10

* Linux: ```apt-get install libcgal-dev```
* Mac: ```port install cgal```

En el linker, agregar ```-lCGAL```.

## OSRM backend

Open Source Routing Machine: The OpenStreetMap Data Routing Engine, http://project-osrm.org

1) Bajar de https://github.com/Project-OSRM/osrm-backend/releases, descomprimir y entrar al directorio respectivo.
2) Compilar e instalar. Por omisión, ```--prefix=/usr/local```:
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release (!)
cmake --build .
cmake --build . --target intall  (!!)
```
(!) Este proceso necesita al menos 3GB en RAM.

(!!) Necesita permisos de superusario.

En el linker, agregar ```-losrm```.

## Observaciones

Esta versión no depende de la librería kdtree




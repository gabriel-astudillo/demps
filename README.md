# DEMPS
**D**isaster **E**vacuation and **M**obility **P**atterns **S**imulator

Este simulador implementa movimientos de personas dentro de una ciudad costera, en el caso de una evacuación de los habitantes hacia las zonas seguras que pueda tener dicho lugar.

**Estructura de directorios**

* ```sim/```: directorio donde se realiza la simulación.
* ```sim/ciudadX.config``` : Archivo JSON de configuración para la simulación de la *ciudadX*.
* ```sim/ejemplos/``` : ejemplo de archivos de visualización de una simulación.
* ```sim/input/``` : Directorio que contiene los archivos de mapas de las ciudades a simular. Cada ciudad es un subdirectorio, que contiene los archivos:
* ```sim/input/ciudadX/``` : Directorio de los archivos de mapas (GeoJSON) de la *ciudadX*. 
* ```sim/input/ciudadX/area.geojson``` : zona geográfica a simular
* ```sim/input/ciudadX/initial_zones.geojson``` : zonas iniciales donde los habitantes se crean
* ```sim/input/ciudadX/reference_zones.geojson``` : zonas de encuentro. Son los lugares donde las personas deben llegar en caso de evacuación.

# Cómo usar

Para utilizar el simulador, se puede ocupar el ejecutable estático disponible dentro del directorio ```sim```. Hay que renombrarlo a ```demps```y ejecutar el script ```run.sh``` para invocarlo correctamente.

```
git clone https://github.com/gabriel-astudillo/demps.git
cd demps
cd sim
mv demps.static demps
chmod +x demps
chmod +x run.sh
./run.sh
```

Otra forma es proceder a compilarlo. Para esto, hay que realizar los pasos de la siguiente sección.

# Compilación e instalación
1) Verificar que su sistema tiene las [dependencias](https://github.com/gabriel-astudillo/demps/blob/master/dependiencias.md) necesarias para la compilación.

2) Clonar este repositorio 

```
git clone https://github.com/gabriel-astudillo/demps.git
cd demps
make
```

El ejecutable se deja en el directorio ```sim```. Para realizar una simulación, utilizar el script ```run.sh```.

```
cd sim
chmod +x run.sh
./run.sh
```


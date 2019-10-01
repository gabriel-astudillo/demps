# DEMPS
Disaster Evacuation and Mobility Patterns Simulator

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
1) Verificar que su sistema tiene las dependencias necesarias para la compilación. Ver [aqui](https://github.com/gabriel-astudillo/demps/blob/master/dependiencias.md)

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


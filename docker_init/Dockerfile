# syntax=docker/dockerfile:1
FROM ubuntu:20.04
MAINTAINER Gabriel Astudillo Muñoz "gabriel.astudillo@gmail.com"


ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Santiago


RUN apt update && apt -y upgrade
RUN apt install -y sudo
RUN apt install -y apache2

ADD ["image_dir/etc/apache2/sites-available/000-default.conf", "/etc/apache2/sites-available/" ]

RUN apt install -y vim

RUN adduser --gecos "" --disabled-password --ingroup www-data demps-user
RUN adduser demps-user sudo
ADD ["image_dir/etc/sudoers", "/etc/sudoers"]

RUN apt install -y make g++ cmake
#RUN apt install -y libboost-all-dev
RUN apt install -y libboost-filesystem1.71-dev libboost-iostreams1.71-dev libboost-thread-dev
RUN apt install -y uuid-dev
RUN apt install -y jq
RUN apt install -y osmctools
RUN apt install -y libcgal-dev
RUN apt install -y libcurlpp-dev
RUN apt install -y libcurl4-openssl-dev
RUN apt install -y libtbb-dev libbz2-dev liblua5.3-dev libexpat1-dev
RUN apt install -y git
RUN apt install -y bc libspatialindex-dev unar gdal-bin
RUN apt install -y pip

RUN apt install -y supervisor
ADD ["image_dir/etc/supervisord.conf", "/etc/supervisord.conf"]

RUN apt clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install lazy bottle rtree gunicorn

USER demps-user
RUN cd /home/demps-user && git clone https://github.com/gabriel-astudillo/demps.git

# compile GeographicLib
RUN cd /home/demps-user/demps/dependencias && tar xzf GeographicLib-1.49.tar.gz
RUN mkdir /home/demps-user/demps/dependencias/GeographicLib-1.49/build
RUN cd /home/demps-user/demps/dependencias/GeographicLib-1.49/build && ../configure && make
RUN cd /home/demps-user/demps/dependencias/GeographicLib-1.49/build && sudo make install

# compile restclient-cpp
RUN cd /home/demps-user/demps/dependencias && git clone https://github.com/mrtazz/restclient-cpp.git
RUN cd /home/demps-user/demps/dependencias/restclient-cpp && ./autogen.sh && ./configure --prefix=/usr/local/restclient-cpp
RUN cd /home/demps-user/demps/dependencias/restclient-cpp && sudo make install

#compile osrm-backend
RUN cd /home/demps-user/demps/dependencias && tar xzf osrm-backend-5.18.0-patched.tgz
RUN mkdir /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build
RUN cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && cmake .. -DCMAKE_BUILD_TYPE=Release
RUN cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && cmake --build .
RUN cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && sudo cmake --build . --target install

RUN sudo rm -rf /home/demps-user/demps/dependencias

#compile demps
RUN cd /home/demps-user/demps && make -j && make install
RUN chmod +x /home/demps-user/demps/sim/run.sh

#create tiles files for openelevation server
#RUN ln -s /home/demps-user/demps/open.elevation.server /home/demps-user/open.elevation.server
RUN chmod +x /home/demps-user/open.elevation.server/make-srtm-data.sh
RUN chmod +x /home/demps-user/open.elevation.server/openElevationServer.py

RUN /home/demps-user/open.elevation.server/make-srtm-data.sh

USER root
RUN ldconfig


EXPOSE 80
#ENTRYPOINT ["/usr/sbin/apache2ctl","-D","FOREGROUND"]
ENTRYPOINT ["/usr/bin/supervisord", "-c", "/etc/supervisord.conf"]



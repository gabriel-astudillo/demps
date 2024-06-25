# syntax=docker/dockerfile:1
FROM ubuntu:20.04
MAINTAINER Gabriel Astudillo Muñoz "gabriel.astudillo@gmail.com"


ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Santiago


RUN apt update && apt -y upgrade
#RUN apt install -y libboost-all-dev
RUN apt install -y sudo apache2                 \
					make g++ cmake              \
					vim                         \
					libboost-filesystem1.71-dev \
					libboost-iostreams1.71-dev  \
					libboost-thread-dev         \
					libboost-test1.71-dev       \
					uuid-dev                    \
					jq                          \
					osmctools                   \
					libcgal-dev                 \
					libcurlpp-dev               \
					libcurl4-openssl-dev        \
					libtbb-dev libbz2-dev       \
					liblua5.3-dev               \
					libexpat1-dev               \
					bc                          \
					libspatialindex-dev         \
					unar                        \
					gdal-bin                    \
					pip                         \
					git                         \
					supervisor


RUN apt clean && rm -rf /var/lib/apt/lists/*
RUN pip3 install lazy bottle rtree gunicorn
					
RUN adduser --gecos "" --disabled-password --ingroup www-data demps-user
RUN adduser demps-user sudo

ADD ["image_dir/etc/sudoers", "/etc/sudoers"]
ADD ["image_dir/etc/apache2/sites-available/000-default.conf", "/etc/apache2/sites-available/" ]
ADD ["image_dir/etc/supervisord.conf", "/etc/supervisord.conf"]



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
RUN cd /home/demps-user/demps && make && make install
RUN chmod +x /home/demps-user/demps/sim/run.sh

#create tiles files for openelevation server
#RUN ln -s /home/demps-user/demps/open.elevation.server /home/demps-user/open.elevation.server
RUN chmod +x /home/demps-user/open.elevation.server/make-srtm-data.sh
RUN chmod +x /home/demps-user/open.elevation.server/openElevationServer.py

RUN /home/demps-user/open.elevation.server/make-srtm-data.sh

USER root
RUN ldconfig


EXPOSE 80
ENTRYPOINT ["/usr/bin/supervisord", "-c", "/etc/supervisord.conf"]



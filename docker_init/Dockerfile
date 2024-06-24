# syntax=docker/dockerfile:1
FROM ubuntu:20.04
MAINTAINER Gabriel Astudillo Muñoz "gabriel.astudillo@gmail.com"


ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Santiago

RUN ldconfig

RUN apt update && apt -y upgrade
RUN apt install -y sudo
RUN apt install -y apache2

ADD ["000-default.conf", "/etc/apache2/sites-enabled/" ]

RUN apt install -y vim

RUN adduser --gecos "" --disabled-password --ingroup www-data demps-user
RUN adduser demps-user sudo
ADD ["sudoers", "/etc/sudoers"]

RUN apt install -y make g++ cmake
RUN apt install -y libboost-all-dev
RUN apt install -y uuid-dev
RUN apt install -y jq
RUN apt install -y osmctools
RUN apt install -y libcgal-dev
RUN apt install -y libcurlpp-dev
RUN apt install -y libcurl4-openssl-dev
RUN apt install -y libtbb-dev libbz2-dev liblua5.3-dev libexpat1-dev
RUN apt install -y git
RUN apt clean && rm -rf /var/lib/apt/lists/*

#USER demps-user

#RUN su demps-user
RUN su demps-user -c "cd /home/demps-user && git clone https://github.com/gabriel-astudillo/demps.git"

# compile GeographicLib
RUN su demps-user -c "cd /home/demps-user/demps/dependencias && tar xzf GeographicLib-1.49.tar.gz"
RUN su demps-user -c "mkdir /home/demps-user/demps/dependencias/GeographicLib-1.49/build"
RUN su demps-user -c "cd /home/demps-user/demps/dependencias/GeographicLib-1.49/build && ../configure && make"
RUN cd /home/demps-user/demps/dependencias/GeographicLib-1.49/build && make install

# compile restclient-cpp
RUN su demps-user -c "cd /home/demps-user/demps/dependencias && git clone https://github.com/mrtazz/restclient-cpp.git"
RUN su demps-user -c "cd /home/demps-user/demps/dependencias/restclient-cpp && ./autogen.sh && ./configure --prefix=/usr/local/restclient-cpp"
RUN cd /home/demps-user/demps/dependencias/restclient-cpp && make install

#compile osrm-backend
RUN su demps-user -c "cd /home/demps-user/demps/dependencias && tar xzf osrm-backend-5.18.0-patched.tgz && cd osrm-backend-5.18.0-patched"
RUN su demps-user -c "mkdir /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build"
RUN su demps-user -c "cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && cmake .. -DCMAKE_BUILD_TYPE=Release"
RUN su demps-user -c "cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && cmake --build ."
RUN cd /home/demps-user/demps/dependencias/osrm-backend-5.18.0-patched/build && cmake --build . --target install

#compile demps
RUN su demps-user -c "cd /home/demps-user/demps && make -j && make install"
RUN chmod +x /home/demps-user/demps/sim/run.sh



EXPOSE 80
ENTRYPOINT ["/usr/sbin/apache2ctl","-D","FOREGROUND"]


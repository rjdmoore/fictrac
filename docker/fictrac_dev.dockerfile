## To build, execute from current directory:
# docker build -t fictrac_dev -f .\fictrac_dev.dockerfile .
## To enable volume sharing between host/container, execute in elevated powershell:
# Set-NetConnectionProfile -InterfaceAlias "vEthernet (DockerNAT)" -NetworkCategory Private
# (restart docker for windows)
## To launch container:
# docker run -v c:\Users\richardm:/home --name fictrac_dev -ti fictrac_dev /sbin/my_init -- bash -l
## To exit/stop container:
# exit
## To relaunch container:
# docker start -i fictrac_dev

FROM phusion/baseimage

# Use baseimage-docker's init system.
CMD ["/sbin/my_init"]

ENV DISPLAY=10.0.75.1:0
#RUN rm -f /etc/service/sshd/down
#RUN sed -i 's/.*PermitRootLogin.*/PermitRootLogin yes/' /etc/ssh/sshd_config

# Install prerequisities
RUN apt-get update
RUN apt-get -y install make pkg-config libopencv-dev libboost-all-dev libnlopt-dev libcairomm-1.0 gedit git gitk git-gui

# Clean up APT when done.
RUN apt-get clean && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

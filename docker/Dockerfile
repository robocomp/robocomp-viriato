#Download base image robocomp agn 18.04
FROM robocomp/robocomp:bionic_agm

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal"

    
# Install AGM dependencies
USER robolab
RUN cd robocomp/components/ && git clone --progress https://github.com/robocomp/robocomp-viriato.git
WORKDIR "robocomp/components/robocomp-viriato"
USER root
RUN cp interfaces/IDSLs/* /opt/robocomp/interfaces/IDSLs/
USER robolab
RUN cd components && mkdir build && cd build && cmake .. && make

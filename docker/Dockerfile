#Download base image robocomp agm 18.04
FROM robocomp/robocomp:bionic_agm

# LABEL about the custom image
LABEL maintainer="emartinena@unex.es"
LABEL version="0.1"
LABEL description="This is custom Docker Image for Robocomp in Ubuntu Focal"

# Update Ubuntu Software repository
RUN sudo apt update 

# Install AGM dependencies
USER robolab
RUN cd robocomp/components/ && git clone --progress https://github.com/robocomp/robocomp-viriato.git
WORKDIR "robocomp/components/robocomp-viriato"
USER root
RUN cp interfaces/IDSLs/* /opt/robocomp/interfaces/IDSLs/
RUN cp interfaces/*.ice /opt/robocomp/interfaces/

RUN apt install -y gcc libpq-dev python-dev  python-pip python3-dev python3-pip python3-venv python3-wheel portaudio19-dev python-pyaudio festival festvox-kallpc16k python-zeroc-ice
USER robolab
RUN python3 -m pip install --upgrade pip
RUN python3 -m pip install wheel PyAudio rasa matplotlib
RUN python3 -m pip install -U spacy
RUN python3 -m spacy download en_core_web_md
RUN python3 -m spacy link en_core_web_md en
USER root
RUN cd /usr/local/include/  && sudo git clone https://github.com/ryanhaining/cppitertools && cd cppitertools && sed -i "1s/.*/cmake_minimum_required(VERSION 3.10)/" CMakeLists.txt && head CMakeLists.txt && cmake . && sudo make install
RUN cd components && mkdir build && cd build && cmake .. && make


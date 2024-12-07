# this file contains a script to load all dependencies, which are not already installed in the docker container
# later on we should adapt the dockerfile to add those

sudo apt update
sudo apt install ros-noetic-mir-robot -y
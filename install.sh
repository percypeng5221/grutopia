#!/bin/bash

eval "$(conda shell.bash hook)"
source "$CONDA_PREFIX/etc/profile.d/mamba.sh"

ENV_NAME='grutopia'

mamba activate $ENV_NAME

# !!! Ensure correct environment is actually activated !!! #
if [ "$CONDA_DEFAULT_ENV" = "$ENV_NAME" ]; then
    echo "CONDA_DEFAULT_ENV is $ENV_NAME"
else
    echo "Error: CONDA_DEFAULT_ENV is not $ENV_NAME"
    exit 1
fi

wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
unzip eigen-3.4.0.zip -d thirdparty
rm eigen-3.4.0.zip

pip install -e .
pip install wget

# Pangolin Viewer
./Pangolin/scripts/install_prerequisites.sh recommended
mkdir Pangolin/build
cd Pangolin/build
pwd
cmake ..
make -j8
sudo make install
cd ../..



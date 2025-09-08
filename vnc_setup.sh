#!/bin/bash
# This script sets up a GCP VM with ROS 2, Jupyter, and other

set -e # Exit immediately if a command exits with a non-zero status

# Install basic tools
echo "Installing basic tools..."
sudo apt update -q && sudo apt install -y \
    nano \
    htop \
    software-properties-common \
    gnupg2 \
    curl \
    wget \
    vim \
    git \
    byobu \
    net-tools \
    ca-certificates \
    apt-transport-https \
    build-essential \
    locales \
    lsb-release

# Set locale
echo "Setting up locale..."
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# vnc prereqs
sudo apt-get -y -qq install \
        dbus-x11 \
        mesa-utils \
        xclip \
        xfce4 \
        xfce4-panel \
        xfce4-session \
        xfce4-settings \
        xfce4-goodies \
        xorg \
        xubuntu-icon-theme \
        fonts-dejavu
# Disable the automatic screenlock since the account password is unknown
sudo apt-get -y -qq remove xfce4-screensaver

# Install TurboVNC
echo "Installing TurboVNC..."
wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/TurboVNC.gpg
sudo wget -O /etc/apt/sources.list.d/TurboVNC.list https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list
sudo apt-get -y -qq update
sudo apt-get -y -qq install turbovnc

# Add TurboVNC to PATH
echo 'export PATH=/opt/TurboVNC/bin:$PATH' >> ~/.bashrc

# Configure VNC server
echo "Configuring VNC server..."
mkdir -p ~/.vnc

# Set VNC password
echo "Setting up VNC password..."
/opt/TurboVNC/bin/vncpasswd -f <<< "vnc123" > ~/.vnc/passwd
chmod 600 ~/.vnc/passwd

# Install ROS 2 Jazzy
echo "Installing ROS 2 Jazzy..."

sudo apt install software-properties-common
sudo add-apt-repository universe -y

sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" # If using Ubuntu derivates use $UBUNTU_CODENAME
sudo dpkg -i /tmp/ros2-apt-source.deb

sudo apt update && sudo apt install ros-dev-tools -y

sudo apt install ros-jazzy-desktop -y
sudo apt install ros-jazzy-ros-gz -y

source /opt/ros/jazzy/setup.bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

# Create conda environment
# Install our public GPG key to trusted store
curl https://repo.anaconda.com/pkgs/misc/gpgkeys/anaconda.asc | gpg --dearmor > conda.gpg
sudo install -o root -g root -m 644 conda.gpg /usr/share/keyrings/conda-archive-keyring.gpg

# Check whether fingerprint is correct (will output an error message otherwise)
sudo gpg --keyring /usr/share/keyrings/conda-archive-keyring.gpg --no-default-keyring --fingerprint 34161F5BF5EB1D4BFBBB8F0A8AEB4F8B29D82806

# Add our Debian repo
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/conda-archive-keyring.gpg] https://repo.anaconda.com/pkgs/misc/debrepo/conda stable main" | sudo tee -a /etc/apt/sources.list.d/conda.list

sudo apt update
sudo apt install conda

source /opt/conda/etc/profile.d/conda.sh
echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc

echo "Creating conda environment..."
conda create -n ros2 python=3.12 -y

# Activate conda environment
conda activate ros2

# Add conda-forge channel
conda config --add channels conda-forge

# Install mamba and websockify
conda install -y mamba
mamba install -y websockify \
    'jupyterhub-singleuser' \
    'jupyterlab' \
    'notebook'

    
jupyter server --generate-config && \
    mamba clean --all -f -y && \
    jupyter lab clean


# Install Python packages
echo "Installing Python packages..."
pip install --upgrade \
    ipywidgets \
    jupyter-resource-usage \
    jupyter-server-proxy \
    jupyterlab-git \
    jupyter-remote-desktop-proxy \
    jupyter_offlinenotebook \
    Pillow \
    rosdep \
    sidecar \
    lark \
    catkin_tools \
    colcon-common-extensions

pip cache purge

sudo rosdep init
rosdep update

# # Install VSCode server (UNUSED)
# echo "Installing VSCode server..."
# curl -fsSL https://code-server.dev/install.sh | sh -s

# # Add code alias
# echo 'alias code="$(which code-server)"' >> ~/.bashrc

# # Install VSCode extensions
# code-server --install-extension ms-python.python
# code-server --install-extension ms-toolsai.jupyter

# pip install jupyter-code-server

# --- Jupyter Configuration ---
echo "Configuring Jupyter..."
mkdir -p ~/.jupyter/certificates
openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout ~/.jupyter/certificates/mykey.key \
    -out ~/.jupyter/certificates/mycert.pem \
    -subj "/CN=localhost"

# Configure Jupyter for remote desktop
echo "Configuring Jupyter for remote desktop..."
# Generate password hash
PASSWORD="robotics2025"
HASHED_PASSWORD=$(python -c "from jupyter_server.auth import passwd; print(passwd('$PASSWORD'))")

cat > ~/.jupyter/jupyter_server_config.py << EOF
c.ServerApp.allow_origin = '*'
c.ServerApp.allow_remote_access = True
c.ServerApp.ip = '0.0.0.0'
c.ServerApp.port = 8888
c.ServerApp.open_browser = False
c.ServerApp.certfile = '/home/$USER/.jupyter/certificates/mycert.pem'
c.ServerApp.keyfile = '/home/$USER/.jupyter/certificates/mykey.key'
c.ServerApp.password = u'$HASHED_PASSWORD'
EOF

jupyter server extension enable --py jupyter_server_proxy --sys-prefix
jupyter server extension enable --py jupyter_remote_desktop_proxy --sys-prefix

# Install franka ros2 packages
echo "Installing libfranka dependencies..."

# libfranka dependency
sudo apt-get update
sudo apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev

sudo apt-get install -y lsb-release curl
sudo mkdir -p /etc/apt/keyrings
curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | sudo tee /etc/apt/keyrings/robotpkg.asc

echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | sudo tee /etc/apt/sources.list.d/robotpkg.list

sudo apt-get update
sudo apt-get install -y robotpkg-pinocchio

echo "Installing franka_ros2 packages..."
mkdir -p ~/franka_ros2_ws/src 
cd ~/franka_ros2_ws  # not into src

# clone branch jazzy
git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src
vcs import src < src/franka.repos --recursive --skip-existing

rosdep install --from-paths src --ignore-src --rosdistro jazzy -y

# use the --symlinks option to reduce disk usage, and facilitate development.
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Install Firefox
echo "Installing Firefox..."
sudo install -d -m 0755 /etc/apt/keyrings
wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O- | sudo tee /etc/apt/keyrings/packages.mozilla.org.asc > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main" | sudo tee -a /etc/apt/sources.list.d/mozilla.list > /dev/null
echo '
Package: *
Pin: origin packages.mozilla.org
Pin-Priority: 1000
' | sudo tee /etc/apt/preferences.d/mozilla
sudo apt-get update &&  sudo apt-get install -y firefox

echo 'Finished setup!'
echo 'Rebooting in 10 seconds...'
sleep 10
sudo reboot now

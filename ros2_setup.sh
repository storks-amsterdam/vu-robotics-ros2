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
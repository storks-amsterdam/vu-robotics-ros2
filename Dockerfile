FROM quay.io/jupyter/minimal-notebook:ubuntu-24.04

# --- Define Environment Variables--- #
ENV ROS_DISTRO=jazzy
ARG ROS_PKG=desktop
LABEL version="ROS-${ROS_DISTRO}-${ROS_PKG}"

ENV ROS_PATH=/opt/ros/${ROS_DISTRO}
ENV ROS_ROOT=${ROS_PATH}/share/ros
ENV ROS_WS=${HOME}/ros2_ws

# --- Install basic tools --- #
USER root
RUN  apt update -q && apt install -y \
        software-properties-common \
        gnupg2 \
        curl \
        wget \
        vim \
        git \
        byobu \
        net-tools\
        ca-certificates \
        apt-transport-https \
        build-essential \
        locales \
        lsb-release \
        nano \
        htop

# --- Grant sudo privileges to NB_USER ---
RUN apt-get update && apt-get install -y sudo && \
    echo "${NB_USER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${NB_USER} && \
    chmod 0440 /etc/sudoers.d/${NB_USER}

# Set locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# --- Install ROS2 --- #
USER root
RUN add-apt-repository universe -y
RUN apt update && apt install curl -y
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $UBUNTU_CODENAME)_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb
RUN apt update && \
    apt install -y \
        ros-dev-tools \
        ros-${ROS_DISTRO}-${ROS_PKG} \
        ros-${ROS_DISTRO}-ros-gz && \
    apt clean && \
    echo "source ${ROS_PATH}/setup.bash" >> /root/.bashrc && \
    echo "source ${ROS_PATH}/setup.bash" >> /home/${NB_USER}/.bashrc

# --- rosdep init --- #
RUN rosdep init && \
    rosdep update && \
    rosdep fix-permissions

# --- Install VNC server and XFCE desktop environment --- #
USER root
RUN apt-get -y -qq update \
 && apt-get -y -qq install \
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
        fonts-dejavu \
    # Disable the automatic screenlock since the account password is unknown
 && apt-get -y -qq remove xfce4-screensaver \
 && mkdir -p /opt/install \
 && chown -R $NB_UID:$NB_GID $HOME /opt/install

# Install a VNC server, (TurboVNC)
ENV PATH=/opt/TurboVNC/bin:$PATH
RUN echo "Installing TurboVNC"; \
    # Install instructions from https://turbovnc.org/Downloads/YUM
    wget -q -O- https://packagecloud.io/dcommander/turbovnc/gpgkey | \
    gpg --dearmor >/etc/apt/trusted.gpg.d/TurboVNC.gpg; \
    wget -O /etc/apt/sources.list.d/TurboVNC.list https://raw.githubusercontent.com/TurboVNC/repo/main/TurboVNC.list; \
    apt-get -y -qq update; \
    apt-get -y -qq install \
        turbovnc \
    ; \
    rm -rf /var/lib/apt/lists/*;

# Install VNC jupyterlab extension
USER ${NB_USER}
RUN mamba install -y websockify
ENV DISPLAY=:1

# Configure VNC server
USER ${NB_USER}
RUN mkdir -p /home/${NB_USER}/.vnc && \
    /opt/TurboVNC/bin/vncpasswd -f <<< "vnc123" > /home/${NB_USER}/.vnc/passwd && \
    chmod 600 /home/${NB_USER}/.vnc/passwd

# Fix permissions
USER root
RUN chown -R ${NB_UID}:${NB_GID} ${HOME}

# --- Install python packages --- #
USER ${NB_USER}
RUN pip install --upgrade \
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
        colcon-common-extensions \
        && pip cache purge

# --- Jupyter Configuration ---
USER ${NB_USER}
RUN mkdir -p /home/${NB_USER}/.jupyter/certificates && \
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout /home/${NB_USER}/.jupyter/certificates/mykey.key \
    -out /home/${NB_USER}/.jupyter/certificates/mycert.pem \
    -subj "/CN=localhost"

RUN HASHED_PASSWORD=$(python -c "from jupyter_server.auth import passwd; print(passwd('robotics2025'))") && \
    jupyter server --generate-config -y && \
    echo "c.ServerApp.allow_origin = '*'" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.allow_remote_access = True" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.ip = '0.0.0.0'" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.port = 8888" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.open_browser = False" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.certfile = '/home/${NB_USER}/.jupyter/certificates/mycert.pem'" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.keyfile = '/home/${NB_USER}/.jupyter/certificates/mykey.key'" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.password = u'${HASHED_PASSWORD}'" >> /home/${NB_USER}/.jupyter/jupyter_server_config.py

RUN jupyter server extension enable --py jupyter_server_proxy --sys-prefix && \
    jupyter server extension enable --py jupyter_remote_desktop_proxy --sys-prefix

# # --- Install VSCode server --- # --- UNUSED ---
# USER ${NB_USER}
# RUN curl -fsSL https://code-server.dev/install.sh | sh
# RUN echo 'alias code="$(which code-server)"' >> ~/.bashrc
# RUN code-server --install-extension ms-python.python \
#   && code-server --install-extension ms-toolsai.jupyter
# RUN pip install jupyter-code-server
# ENV CODE_WORKING_DIRECTORY=${HOME}

# --- Install franka_ros2 --- #
USER root
RUN apt-get update && \
    apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev lsb-release curl && \
    mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y robotpkg-pinocchio

USER ${NB_USER}
RUN mkdir -p ${HOME}/franka_ros2_ws/src
WORKDIR ${HOME}/franka_ros2_ws
RUN git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ${HOME}/franka_ros2_ws/install/setup.bash" >> /home/${NB_USER}/.bashrc

# Install Firefox
USER root
RUN install -d -m 0755 /etc/apt/keyrings && \
    wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O- | sudo tee /etc/apt/keyrings/packages.mozilla.org.asc > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main" | sudo tee -a /etc/apt/sources.list.d/mozilla.list > /dev/null && \
    printf 'Package: *\nPin: origin packages.mozilla.org\nPin-Priority: 1000\n' | sudo tee /etc/apt/preferences.d/mozilla > /dev/null && \
    apt-get update && \
    apt-get install -y firefox

# --- Entrypoint --- #
USER ${NB_USER}
COPY --chown=${NB_USER}:users entrypoint.sh /
RUN chmod +x /entrypoint.sh
WORKDIR ${HOME}
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "start-notebook.sh" ]
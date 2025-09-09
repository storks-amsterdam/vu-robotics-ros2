FROM osrf/ros:jazzy-desktop-noble

ARG USERNAME="ros2"
ARG USER_UID=1000
ARG USER_GID=1000

ENV NB_USER=$USERNAME
ENV NB_UID=$USER_UID
ENV NB_GID=$USER_GID
ENV HOME=/home/${NB_USER}

# Create user, and grant sudo privileges.
USER root
RUN apt-get update && apt-get install -y sudo && \
    if id -u $NB_UID >/dev/null 2>&1; then userdel --remove `id -un $NB_UID`; fi && \
    groupadd --gid $NB_GID $NB_USER && \
    useradd --uid $NB_UID --gid $NB_GID -m -s /bin/bash $NB_USER && \
    echo $NB_USER ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$NB_USER && \
    chmod 0440 /etc/sudoers.d/$NB_USER && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

SHELL ["/bin/bash", "-c"]

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

# --- Install Miniconda --- #
USER root
RUN curl -fsSL https://repo.anaconda.com/pkgs/misc/gpgkeys/anaconda.asc | gpg --dearmor > conda.gpg && \
    install -o root -g root -m 644 conda.gpg /usr/share/keyrings/conda-archive-keyring.gpg && \
    gpg --keyring /usr/share/keyrings/conda-archive-keyring.gpg --no-default-keyring --fingerprint 34161F5BF5EB1D4BFBBB8F0A8AEB4F8B29D82806 && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/conda-archive-keyring.gpg] https://repo.anaconda.com/pkgs/misc/debrepo/conda stable main" | tee -a /etc/apt/sources.list.d/conda.list && \
    apt-get -y -qq update && \
    apt-get -y -qq install conda && \
    echo 'source /opt/conda/etc/profile.d/conda.sh' >> ${HOME}/.bashrc

# Create conda environment
USER ${NB_USER}
RUN source /opt/conda/etc/profile.d/conda.sh && \
    conda config --add channels conda-forge && \
    conda config --set channel_priority strict && \
    conda create -n ros2 python=3.12 mamba -y && \
    echo "conda activate ros2" >> ${HOME}/.bashrc && \
    conda run -n ros2 mamba install -y websockify \
        'jupyterhub-singleuser' \
        'jupyterlab' \
        'notebook'
ENV DISPLAY=:1

# Configure VNC server
USER ${NB_USER}
RUN mkdir -p ${HOME}/.vnc && \
    /opt/TurboVNC/bin/vncpasswd -f <<< "vnc123" > ${HOME}/.vnc/passwd && \
    chmod 600 ${HOME}/.vnc/passwd

# Fix permissions
USER root
RUN chown -R ${NB_UID}:${NB_GID} ${HOME}

# --- Install python packages --- #
USER ${NB_USER}
RUN source /opt/conda/etc/profile.d/conda.sh && \
    conda run -n ros2 pip install --upgrade \
        ipywidgets \
        jupyter-resource-usage \
        jupyter-server-proxy \
        jupyterlab-git \
        jupyter-remote-desktop-proxy \
        jupyter_offlinenotebook \
        Pillow \
        && conda run -n ros2 pip cache purge

# --- Jupyter Configuration ---
USER ${NB_USER}
RUN mkdir -p ${HOME}/.jupyter/certificates && \
    openssl req -x509 -nodes -days 365 -newkey rsa:2048 \
    -keyout ${HOME}/.jupyter/certificates/mykey.key \
    -out ${HOME}/.jupyter/certificates/mycert.pem \
    -subj "/CN=localhost"

RUN source /opt/conda/etc/profile.d/conda.sh && \
    conda activate ros2 && \
    HASHED_PASSWORD=$(python -c "from jupyter_server.auth import passwd; print(passwd('robotics2025'))") && \
    jupyter server --generate-config -y && \
    echo "c.ServerApp.allow_origin = '*'" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.allow_remote_access = True" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.ip = '0.0.0.0'" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.port = 8888" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.open_browser = False" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.terminado_settings = {'shell_command': ['/bin/bash']}" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.certfile = '${HOME}/.jupyter/certificates/mycert.pem'" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.keyfile = '${HOME}/.jupyter/certificates/mykey.key'" >> ${HOME}/.jupyter/jupyter_server_config.py && \
    echo "c.ServerApp.password = u'${HASHED_PASSWORD}'" >> ${HOME}/.jupyter/jupyter_server_config.py

RUN source /opt/conda/etc/profile.d/conda.sh && \
    conda activate ros2 && \
    jupyter server extension enable --py jupyter_server_proxy --sys-prefix && \
    jupyter server extension enable --py jupyter_remote_desktop_proxy --sys-prefix

# # --- Install VSCode server --- # --- UNUSED ---
# USER ${NB_USER}
# RUN curl -fsSL https://code-server.dev/install.sh | sh
# RUN echo 'alias code="$(which code-server)"' >> ~/.bashrc
# RUN code-server --install-extension ms-python.python \
#   && code-server --install-extension ms-toolsai.jupyter
# RUN pip install jupyter-code-server
# ENV CODE_WORKING_DIRECTORY=${HOME}

# Install Firefox
USER root
RUN install -d -m 0755 /etc/apt/keyrings && \
    wget -q https://packages.mozilla.org/apt/repo-signing-key.gpg -O- | sudo tee /etc/apt/keyrings/packages.mozilla.org.asc > /dev/null && \
    echo "deb [signed-by=/etc/apt/keyrings/packages.mozilla.org.asc] https://packages.mozilla.org/apt mozilla main" | sudo tee -a /etc/apt/sources.list.d/mozilla.list > /dev/null && \
    printf 'Package: *\nPin: origin packages.mozilla.org\nPin-Priority: 1000\n' | sudo tee /etc/apt/preferences.d/mozilla > /dev/null && \
    apt-get update && \
    apt-get install -y firefox

# Install Gazebo
USER root
RUN apt install -y \
    ros-${ROS_DISTRO}-ros-gz

# # --- Install franka_ros2 --- #
# USER root
# RUN apt-get update && \
#     apt-get install -y build-essential cmake git libpoco-dev libeigen3-dev libfmt-dev lsb-release curl && \
#     mkdir -p /etc/apt/keyrings && \
#     curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
#     echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" | tee /etc/apt/sources.list.d/robotpkg.list && \
#     apt-get update && \
#     apt-get install -y robotpkg-pinocchio

USER ${NB_USER}
RUN mkdir -p ${HOME}/franka_ros2_ws/src
WORKDIR ${HOME}/franka_ros2_ws
RUN git clone -b jazzy https://github.com/frankarobotics/franka_ros2.git src && \
    vcs import src < src/franka.repos --recursive --skip-existing && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
RUN echo "source ${HOME}/franka_ros2_ws/install/setup.bash" >> ${HOME}/.bashrc

# Change shell to bash for the user
USER root
RUN chsh -s /bin/bash ${NB_USER}

# --- Entrypoint --- #
USER ${NB_USER}
COPY --chown=${NB_UID}:${NB_GID} entrypoint.sh /
RUN chmod +x /entrypoint.sh
WORKDIR ${HOME}
ENTRYPOINT ["/entrypoint.sh"]
CMD [ "jupyter", "server" ]
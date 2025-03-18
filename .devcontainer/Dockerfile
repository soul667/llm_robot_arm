# 带有ROS2 和 gazebo的Dockerbase
FROM osrf/icra2023_ros2_gz_tutorial:roscon2024_tutorial_nvidia
LABEL maintainer="2194521087@qq.com"

# Set environment variables
ARG DEBIAN_FRONTEND=noninteractive
ENV USER_NAME=developer
ARG USER_PASSWORD=0000
ENV HOME=/home/${USER_NAME}

# Improve apt-get reliability
RUN sudo apt-get update --fix-missing 
RUN sudo apt-get install -y --no-install-recommends \
    zsh \
    wget \
    python3-venv \
    python3-pip \
    python3-full \
    git && \
    sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

RUN wget https://github.com/robbyrussell/oh-my-zsh/raw/master/tools/install.sh -O - | zsh || true && \ 
    git clone https://github.com/valentinocossar/vscode.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/vscode && \
    git clone https://github.com/zsh-users/zsh-autosuggestions ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-autosuggestions && \
    git clone https://github.com/zsh-users/zsh-syntax-highlighting.git ${ZSH_CUSTOM:-~/.oh-my-zsh/custom}/plugins/zsh-syntax-highlighting

ADD .zshrc $HOME

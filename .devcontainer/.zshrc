# Path to your oh-my-zsh installation.
export ZSH="$HOME/.oh-my-zsh"

# Set name of the theme to load
ZSH_THEME="robbyrussell"

# Enable command auto-correction
ENABLE_CORRECTION="true"

# Display red dots whilst waiting for completion
COMPLETION_WAITING_DOTS="true"

# Which plugins would you like to load?
plugins=(
    git 
    zsh-syntax-highlighting 
    zsh-autosuggestions 
    vscode
)

#source $ZSH/oh-my-zsh.sh

# User configuration
export LANG=en_US.UTF-8

# ROS2 setup
source /opt/ros/jazzy/setup.zsh 

# Aliases
alias ll='ls -alF'
alias la='ls -A'
alias l='ls -CF'

[ ! -f "$HOME/.x-cmd.root/X" ] || . "$HOME/.x-cmd.root/X" # boot up x-cmd.
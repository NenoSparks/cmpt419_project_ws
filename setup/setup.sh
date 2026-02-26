#!/bin/bash
# Flags
#     --no_import: Do not import from .repos 
#     --host: Install on host (skip tmux settings and fzf)
#     --no_build: Do not build ROS packages
import=true
container=true
build=true

while [[ $# -gt 0 ]]; do
    case "$1" in
    --no_import)
        import=false
        shift
        ;;
    --host)
        container=false
        shift
        ;;      
    --no_build)
        build=false
        shift
        ;;            
    *)
        echo "Unknown option: $arg"
        exit 1
        ;;
    esac
done


set -e
mkdir -p src
mkdir -p py


# ======================
# Clone non-ROS packages
# ======================
# tmux config
if [[ $container == true ]]; then
    if [ ! -f ~/.tmux.conf ]; then
        touch ~/.tmux.conf
        echo "set -g history-limit 10000" >> ~/.tmux.conf
        echo "set -g mouse on" >> ~/.tmux.conf
    fi
    
    # fzf
    if [ ! -d ~/.fzf/ ]; then
        git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
        ~/.fzf/install --key-bindings --completion --update-rc
    fi
fi

pip install --user matplotlib

# ============
# ROS packages
# ============
# Import repos
if [[ $import == true ]]; then
    vcs import src < setup/src.repos --skip-existing -recursive

    # Clone MoveIt tutorial
    if [ ! -d src/moveit2_tutorials ]; then
        git clone -b humble https://github.com/moveit/moveit2_tutorials.git src/moveit2_tutorials
    fi
fi

if [[ $build == true ]]; then
    # Resolve dependencies
    sudo apt-get update
    rosdep update
    rosdep install -i --from-paths src --ignore-src --rosdistro humble -y
    colcon build --symlink-install
fi


echo "Setup complete!"
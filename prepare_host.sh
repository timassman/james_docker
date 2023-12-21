#!/bin/bash

# ********************************************************
# * Install xbox driver                                  *
# ********************************************************

# https://askubuntu.com/questions/695069/xbox-controller-blinking/1438948#1438948

# Install xboxdrv
sudo apt-add-repository ppa:rael-gc/ubuntu-xboxdrv
sudo apt-get update
sudo apt-get install ubuntu-xboxdrv

# Add xpad in the blacklist
echo "blacklist xpad" | sudo tee -a /etc/modprobe.d/blacklist.conf

# Unload the module if it's already loaded
sudo rmmod xpad

# Remove jstest-gtk to set xboxdrv as default
sudo apt-get purge jstest-gtk
sudo apt-get install xboxdrv
sudo apt-get install jstest-gtk #Reinstalling it without affecting xboxdrv

# Copy the settings file. Creating a symlink can cause issues for systemctl
sudo cp xboxdrv/xboxdrv.settings /etc/default/xboxdrv

# https://askubuntu.com/questions/1112920/xboxdrv-not-able-to-start/1438836#1438836

# Copy the service file. Creating a symlink can cause issues for systemctl
sudo cp xboxdrv/xboxdrv.service /etc/systemd/system

# Enable the service
sudo systemctl enable xboxdrv.service

# Make sure the Xbox controller is connected before continuing the next steps
# Start the service
sudo systemctl start xboxdrv.service

# Allow user to read/write to xbox controller
# Permissions will be given on to docker
sudo cp 10-xbox-controller.rules /etc/udev/rules.d/

# ********************************************************
# * Install docker                                       *
# ********************************************************

# https://docs.docker.com/engine/install/ubuntu/

# Run the following command to uninstall all conflicting packages
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Manage docker as non-root user
# https://docs.docker.com/engine/install/linux-postinstall/

# Create the docker group
sudo groupadd docker

# Add your user to the docker group
sudo usermod -aG docker $USER

# Activate the changes to groups
newgrp docker

# ********************************************************
# * Install other packages                               *
# ********************************************************

# Install text editor nano
sudo apt install nano

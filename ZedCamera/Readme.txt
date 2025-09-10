# The image
The original image is based on the image from ZED, with their Zed wrapper. (https://github.com/stereolabs/zed-ros2-wrapper/tree/master/docker)

# Nvidia Runtime

Follow the turtorial guide [here](https://www.stereolabs.com/docs/docker/install-guide-linux/#nvidia-docker)

```bash
    curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
    curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

    sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit
    sudo systemctl daemon-reload
    sudo systemctl restart docker
```

# Start the container
To start the container use for 2 cameras:
```bash
    sudo NUM_CAMERAS=2 docker compose 
```
Note that this only works with the following cameras: 
zedm with serialnumber: 17640832
zed2 with serialnumber:  27432249


  
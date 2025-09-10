# 🖐️ A Modular Handheld Proxy Device for Learning from Demonstration in Robotics

<img width="920" height="199" alt="image" src="https://github.com/user-attachments/assets/8075ab14-466f-439d-97f5-15bcd033ad6c" />

**Handheld_lfd** is a ROS2-based module for the designed **modular handheld device**.  
It provides a flexible setup to add additional sensors and stream their data into a ROS2 bag for further processing.


---

## ✨ Features
- 📡 Base module collects position and orientation data from external camera (Zed2)  
- 🧩 Modular design to add new sensor inputs  
- 🎥 Includes example ROS2 nodes for quick integration  (Zed cam & Sensoeone force sensor) 
- 📦 Built on **ROS2 (via Docker)** for easy deployment  
- 📊 Example scripts for processing orientation/position data  

---

## 🚀 Getting Started

### Prerequisites
- [Docker](https://docs.docker.com/get-docker/) installed
- [ROS2](https://docs.ros.org/en/foxy/Installation.html) knowledge recommended

### Installation
Clone the repository:
```bash
git clone https://github.com/JelmerHofstra/Handheld_lfd.git
cd Handheld_lfd
```

### Run
Follow instructions given in terminal by running:
````bash
sudo python3 Master.py
````


import subprocess
import os
import signal
import keyboard
import time

# Basic
ext_camera = True
 
# Additional sensors
force_sensor = False
int_camera   = False

#Bagfile ? 
recording = True
filename = 'Square40cm_1m_away_45degv7'

topics = [
    "/Senseone_eth/imu",
    "/Senseone_eth/wrench",
    "/cam_ext/zed/point_cloud/cloud_registered",
    "/cam_ext/zed/imu/data",
    "/cam_ext/zed/left/image_rect_color",
    "/cam_int/zed/point_cloud/cloud_registered",
    "/cam_int/zed/imu/data",
    "/cam_int/zed/left/image_rect_color"
]

topic_str = " ".join(topics)  # This creates: "/Senseone_eth/imu /Senseone_eth/wrench ..."




#running 
if ext_camera:
    subprocess.run(
        ["sudo", "docker", "compose", "-f", "docker-compose.yml", "up", "-d"],
        cwd="./ZedCamera/Zed_ext",
        check=True,
    )

if int_camera:
    time.sleep(5)
    subprocess.run(
        ["sudo", "docker", "compose", "-f", "docker-compose.yml", "up", "-d"],
        cwd="./ZedCamera/Zed_int",
        check=True,
    )

if force_sensor:
    subprocess.run(
        ["sudo", "docker", "compose", "-f", "docker-compose.yml", "up", "-d"],
        cwd="./Force_sensor",
        check=True,
    )

if recording :
    subprocess.run(
        ["sudo", "docker", "compose", "-f", "docker-compose.yml", "up", "-d"],
        cwd="./Data_collector",
        check=True,
    )




print(' For exiting the system press: q ')

if recording :
    print(' To start and recording press spacebar')


while True :

    if keyboard.is_pressed('q'):

        if ext_camera:
            subprocess.run(["sudo", "docker", "compose", "down"],cwd="./ZedCamera/Zed_ext", check=True)

        if int_camera : 
             subprocess.run(["sudo", "docker", "compose", "down"],cwd="./ZedCamera/Zed_int", check=True)

        if force_sensor:
            subprocess.run(["sudo", "docker", "compose", "down"],cwd="./Force_sensor",check=True)

        if recording :
            subprocess.run(["sudo", "docker", "compose", "down"],cwd="./Data_collector",check=True)

        


        break
     


     

    if keyboard.is_pressed('space') :
        process = subprocess.Popen(["sudo", "docker", "exec", "Data_collector", "bash", "-c",
            f"source /opt/ros/humble/setup.bash && ros2 bag record -o /root/Bagfiles/{filename} {topic_str}"])

        try:
            print("Recording started. Press Ctrl+C to stop...")
            process.wait()  # Wait for process to finish (or get interrupted)
        except KeyboardInterrupt:
            print("Stopping recording...")
            process.send_signal(signal.SIGINT)  # Send SIGINT to stop rosbag gracefully
            process.wait()
            print("Recording stopped cleanly.")




    # if keyboard.is_pressed('space') :
    #     command = [
    #         "sudo", "docker", "exec", "Data_collector",
    #         "bash", "-c",
    #         f"source /opt/ros/humble/setup.bash && ros2 bag record -o /root/Bagfiles/{filename} {topic_str}"]
        
    #     subprocess.run(command, check=True)
        
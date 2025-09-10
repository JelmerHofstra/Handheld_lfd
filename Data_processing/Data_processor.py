import cv2 
from tqdm import tqdm
import openpyxl
import numpy as np
import os
from scipy.spatial.transform import Rotation as R
from Functions import * 
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection




name_bag_file = 'Supertestv2'




# --- Config Orientation/Position processing---
bag_path = f'./Bagfiles/{name_bag_file}/{name_bag_file}_0.db3'
topic_image = '/cam_ext/zed/left/image_rect_color'
topic_pointcloud = '/cam_ext/zed/point_cloud/cloud_registered'
imu_topic = '/cam_int/zed/imu/data'
wrench_topic = '/Senseone_eth/wrench'
image_int_topic = '/cam_int/zed/left/image_rect_color'

# -- Camera/Marker info -- 
CameraMatrix = np.array([
    [532, 0, 474],
    [0, 532, 264],
    [0,   0,   1]
    ], dtype=np.float32)
DistCoeffs = np.zeros((5,), dtype=np.float32)

MarkerSize = 0.049  # meters
distance2markers=  0.056 # meters 
object_points = np.array([
    [-MarkerSize / 2,  MarkerSize / 2, 0],  # top-left
    [ MarkerSize / 2,  MarkerSize / 2, 0],  # top-right
    [ MarkerSize / 2, -MarkerSize / 2, 0],  # bottom-right
    [-MarkerSize / 2, -MarkerSize / 2, 0]   # bottom-left
], dtype=np.float32)

#Check if video and excel file already exist , otherwise make them.
if os.path.exists(f'./Data_processing/Recordings/{name_bag_file}/video.mp4') and os.path.exists(f'./Data_processing/Recordings/{name_bag_file}/Data.xlsx') and os.path.exists(f'./Data_processing/Recordings/{name_bag_file}/configs.txt'):
    pass
else :
    print(" Making data files ...")
    Pointcloud_aviable = Makeanalyse_files(bag_path,topic_image,name_bag_file,topic_pointcloud, imu_topic,wrench_topic, image_int_topic)

Pointcloud_aviable = True


AllData = Pose_PositionData(CameraMatrix,DistCoeffs,object_points, MarkerSize,distance2markers,name_bag_file,Pointcloud_aviable,bag_path,topic_pointcloud)


idx = 0
outerloopbreak = False
allframes_accepted = False
config = 'Orientation'
while not allframes_accepted:

    while True:
        frame = AllData.Data[idx]["frame"].copy()
        tvec =  AllData.Data[idx]["CentreCube"]
        rvec = AllData.Data[idx]["Rvec_cube"]

        cv2.drawFrameAxes(frame, CameraMatrix , DistCoeffs, rvec, tvec, MarkerSize)
        cv2.putText(frame, f'Frame {idx+1}/{len(AllData.Data)}', (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f'{config}', (760, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow('Pose Viewer', frame)

        key = cv2.waitKey(0) & 0xFF  # Wait indefinitely for key press

        if key in [27, ord('q')]:  # ESC or q to quit
            cv2.destroyAllWindows()
            outerloopbreak = True
            break
        elif key in [81, ord('a')]:  # Move one back
            if idx > 0 : 
                idx -=1 
        elif key in [83, ord('d')]:  # Move one front
            if idx < len(AllData.Data)-1 : 
                idx +=1 


        elif key == ord('p') : # show other possability 
            if config == 'Orientation' :
                AllData.Data[idx]["Rvec_cube"],AllData.Data[idx]["Rvec_cube_alt"] = AllData.Data[idx]["Rvec_cube_alt"],AllData.Data[idx]["Rvec_cube"]
            else :
                AllData.Data[idx]["CentreCube"],AllData.Data[idx]["CentreCube_alt"] = AllData.Data[idx]["CentreCube_alt"],AllData.Data[idx]["CentreCube"]


        elif key == ord('o') : # Visualize plot
            if config == 'Orientation' :
                rvecs = [item["Rvec_cube"] for item in AllData.Data]
                AllData.plot_quats()
            else :
                Positions = [item["CentreCube"] for item in AllData.Data]
                AllData.plot_positions()

        elif key == ord('l') :
            if config == 'Orientation' :
                config = 'Position'
            else : 
                config = 'Orientation'

                
        elif key == ord('v') : #Delete image
            AllData.Data.pop(idx)
            AllData.skips.append(idx)
            idx -=1
            cv2.destroyAllWindows()
            break       
        
        elif key == ord('c') :
            if idx > 0 : 
                AllData.Data = AllData.Recalculatecpositions_and_orientations(idx)

        elif key == ord('z') :
            answer1 = input('What is your begin frame?')
            answer2 = input('What is your end frame?')
            idx = 0
            AllData.Define_Begin_End(int(answer1),int(answer2))
            
        elif key == ord('y')  :
            allframes_accepted =True
            AllData.Savedata()
            cv2.destroyAllWindows()
            break

    if outerloopbreak :
        break




print(AllData.Cam2world)
rotm_cube,_ = cv2.Rodrigues(AllData.Data[0]["Rvec_cube"])
for i, item in enumerate(AllData.Data):
    if "CentreCube" in item:
        AllData.Data[i]["CentreCube"] = AllData.Cam2world @ item["CentreCube"]
        # AllData.Data[i]["CentreCube"] = rotm_cube @ AllData.Data[i]["CentreCube"]

    if "CentreCube_alt" in item:
        AllData.Data[i]["CentreCube_alt"] = AllData.Cam2world @ item["CentreCube_alt"]
        # AllData.Data[i]["CentreCube_alt"] = rotm_cube @ AllData.Data[i]["CentreCube_alt"]







# --- Collect data ---
x_coords = []
y_coords = []
z_coords = []
x_coords_alt = []
y_coords_alt = []
z_coords_alt = []

for item in AllData.Data:
    if "CentreCube" in item:
        point = item["CentreCube"]
        x_coords.append(point[0])
        y_coords.append(point[1])
        z_coords.append(point[2])

    if "CentreCube_alt" in item:
        point_alt = item["CentreCube_alt"]
        x_coords_alt.append(point_alt[0])
        y_coords_alt.append(point_alt[1])
        z_coords_alt.append(point_alt[2])

# --- Align data ---
x_shifted = [x - x_coords[1] for x in x_coords]
y_shifted = [y - y_coords[1] for y in y_coords]
z_shifted = [z - z_coords[1] for z in z_coords] 
x_shifted_alt = [x - x_coords_alt[1] for x in x_coords_alt]
y_shifted_alt = [y - y_coords_alt[1] for y in y_coords_alt]
z_shifted_alt = [z - z_coords_alt[1] for z in z_coords_alt]

# --- Compare and collect best points ---
best_points_x = []
best_points_y = []
best_points_z =[]
best_distances = []


for i in range(len(x_shifted)):
    dist_main = distance_to_square_edge(x_shifted[i], z_shifted[i])
    dist_alt = distance_to_square_edge(x_shifted_alt[i], z_shifted_alt[i])
    
    if dist_main < dist_alt:
        best_points_x.append(x_shifted[i])
        best_points_y.append(y_shifted[i])
        best_points_z.append(z_shifted[i])
        best_distances.append(dist_main)
    else:
        best_points_x.append(x_shifted_alt[i])
        best_points_y.append(y_shifted_alt[i])
        best_points_z.append(z_shifted_alt[i])

X = np.array(best_points_x)
Y = np.array(best_points_y)
Z = np.array(best_points_z)  # Z is depth

# X = np.array(x_shifted)
# Y = np.array(y_shifted)
# Z = np.array(z_shifted)
# print(Z)

# --- Sanity check ---
assert len(X) == len(Y) == len(Z), "Array lengths don't match!"

# --- Create 3D plot ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')

# Scatter plot in a single color
ax.scatter(X, Y, Z, color='blue', marker='o')

# --- Axis labels ---
ax.set_xlabel('X axis')
ax.set_ylabel('Y axis')
ax.set_zlabel('Z axis (Depth)')
ax.set_title('3D Scatter Plot of Best Points')

# --- Set equal aspect ratio ---
max_range = np.array([
    X.max() - X.min(),
    Y.max() - Y.min(),
    Z.max() - Z.min()
]).max() / 2.0

mid_x = (X.max() + X.min()) / 2.0
mid_y = (Y.max() + Y.min()) / 2.0
mid_z = (Z.max() + Z.min()) / 2.0

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)


# Add square in X-Z plane at y = 0

# --- Add square in X-Z plane at Y=0 ---
square_x = [0, 0.4, 0.4, 0]
square_y = [0, 0, -0.4, -0.4]    # Y constant
square_z = [0,0,0,0]

verts = [list(zip(square_x, square_y, square_z))]

# Use a light facecolor with alpha (instead of 'none')
square = Poly3DCollection(
    verts,
    facecolor=(1, 0, 0, 0.1),     # Red, transparent
    edgecolor='red',
    linewidth=2
)
ax.add_collection3d(square)

# Mark the starting point (first best point)
ax.scatter(
    [X[0]],
    [Y[0]],
    [Z[0]],
    color='green',
    s=100,
    marker='*',
    label='Start point'
)

ax.legend()


plt.tight_layout()
plt.show()



# Create the subplots
fig, axes = plt.subplots(3, 1, figsize=(8, 6), sharex=True)

# Plot x values
axes[0].plot(x_coords, marker='o', color='red')
axes[0].set_ylabel('X')
axes[0].set_title('X over index')

# Plot y values
axes[1].plot(y_coords, marker='o', color='green')
axes[1].set_ylabel('Y') 
axes[1].set_title('Y over index')

# Plot z values
axes[2].plot(z_coords, marker='o', color='blue')
axes[2].set_ylabel('Z')
axes[2].set_title('Z over index')
axes[2].set_xlabel('Index')

# Improve layout
plt.tight_layout()
plt.show()




fig2 = plt.figure(figsize=(8, 6))
ax2 = fig2.add_subplot(111, projection='3d')
ax2.scatter(x_coords, y_coords, z_coords, color='blue', marker='o')
ax2.set_xlabel('X axis')
ax2.set_ylabel('Y axis')
ax2.set_zlabel('Z axis (Depth)')
ax2.set_title('3D Scatter Plot of Best Points')

max_range = np.array([
    x_coords.max() -x_coords.min(),
    y_coords.max() - y_coords.min(),
    z_coords.max() - z_coords.min()
]).max() / 2.0

mid_x = max_range[0]/ 2.0
mid_y = max_range[1]/ 2.0
mid_z = max_range[2]/ 2.0

ax2.set_xlim(mid_x - max_range, mid_x + max_range)
ax2.set_ylim(mid_y - max_range, mid_y + max_range)
ax2.set_zlim(mid_z - max_range, mid_z + max_range)



plt.tight_layout()
plt.show()



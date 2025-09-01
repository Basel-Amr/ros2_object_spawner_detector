# ⚡ ROS2 Gazebo Model Spawner Task
A ROS 2 Humble + Gazebo Classic project for spawning, deleting, visualizing, and detecting models in simulation.
This task demonstrates ROS 2 node development, Gazebo service interaction, RViz visualization, and camera frame saving using cv_bridge + OpenCV

---

# 📦 Features
*  **Model Spawner Node:**
    * Periodically spawns random battery models in Gazebo.
    * Deletes the previously spawned model before spawning a new one.
*  **Reusable Gazebo Client:**
    *  Service client for `/spawn_entity` and `/delete_entity`.
*  **Visualization Markers (RViz):**
    *  Publishes colored markers with labels for spawned models.
*  **Camera Subscriber:**
    *  Subscribes to the camera topic in Gazebo.
    *  Saves every incoming frame as a `.png` image with timestamp.
*  **Docker Support:**
    *  Portable environment with ROS 2 Humble + Gazebo Classic preinstalled.
---
# 🎥 Demo
[![Demo Video]](Assets\demo_video.webm)
---

# 🖥️ Dependencies (Native)
Make sure you have ROS 2 Humble and Gazebo Classic installed. Then install required packages:
```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins

sudo apt install \
  ros-humble-vision-msgs \
  ros-humble-cv-bridge \
  ros-humble-visualization-msgs
```
⚙️ Build the Workspace (Native)
```bash
mkdir -p ~/ros2_ws/src
# place this package inside ~/ros2_ws/src
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---
# 🚀 Quickstart
Open Terminal A (to start Gazebo world + camera):
```bash
ros2 launch ros2_eval_task gazebo.launch.py
```

Open Terminal B (to run the spawner + image saver):
```bash
ros2 run ros2_eval_task model_spawner_node
```

Excepted Output:
*  Gazebo world opening with a camera and ground plane.
*  Battery models spawning one at a time at random positions.
*  Saved images in the configured folder (e.g., ~/ros2_ws/captured_images).
*  Logs in the terminal showing spawned/deleted models + saved images.

---
# 🐳 Docker Usage

**1️⃣ Build the Image**
From the package root (where the Dockerfile is):
```bash
docker build -t ros2-gz-classic:humble .
```
**2️⃣ Run the Container (with GUI)**
```bash
docker run -it --name ros2_gz_gui \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
  --device /dev/dri:/dev/dri \
  -v ~/interview_ws:/home/dev/ros2_ws \
  -w /home/dev/ros2_ws \
  ros2-gz-classic:humble
```

**3️⃣ Inside the Container: Source, Build, and Launch**
```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Make Gazebo see your package models
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/dev/ros2_ws/src/ros2_eval_task/models

# Build
colcon build --symlink-install

# Overlay
source install/setup.bash

# Launch Gazebo
ros2 launch ros2_eval_task gazebo.launch.py
```

**4️⃣ Open Another Shell (Optional)**
```bash
docker exec -it ros2_gz_gui bash
source /opt/ros/humble/setup.bash
cd /home/dev/ros2_ws
source install/setup.bash
ros2 run ros2_eval_task model_spawner_node
```

---
# 📊 Expected Output
* **In Gazebo:**
    *  Only one battery model is visible at a time.
    *  The model changes every 5 seconds at a random position.
* **In Terminal:**
```bash
[INFO] [..] [model_spawner_node]: Spawning model: lipo_battery at (0.18, -0.09, 1.10)
[INFO] [..] [model_spawner_node]: Published marker for model: lipo_battery
✅ Image saved: /home/baselamr/ros2_ws/src/ros2_eval_task/captured_images/image_3_1726409312398471.png
```
---
# 🔮 Future Work
*  Add bounding boxes around detected models in images.
*  Run YOLO/OpenCV to classify the spawned object.
*  Implement automatic dataset generation from captured frames.

---
# Diagrames

🔹 1) System Architecture Diagram
```mermaid
flowchart TD
    A[Model Spawner Node] -->|Spawn/Delete Service| B[Gazebo Simulator]
    B -->|Camera Image| C[Model Spawner Node]
    C -->|Save Frames| D[Disk Storage (Assets/Images)]
    A -->|Markers| E[Rviz2 Visualization]
```
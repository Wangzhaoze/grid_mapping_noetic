# Grid Mapping with ROS Noetic (Docker)

This repository provides a **ROS Noetic**-based implementation for grid mapping using Docker. It includes scripts and tools for creating occupancy grid maps from ROS bag files.

## 📦 Setup Instructions

### 1. Allow X11 access for Docker
```bash
xhost +local:docker
```

### 2. Clone this repository
```bash
git clone git@github.com:Wangzhaoze/grid_mapping_noetic.git
cd grid_mapping_noetic
```

### 3. Pull ROS Noetic Docker image
```bash
docker pull ros:noetic
```

### 4. Run the Docker container
```bash
docker run -it \
    --name grid_mapping_noetic \
    --net=host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/grid_mapping_noetic/:/root/catkin_ws/src/grid_mapping_noetic/" \
    ros:noetic
```

---

## ⚙️ Inside the Container

### 1. Setup Catkin workspace
```bash
cd ~/catkin_ws/

rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Install dependencies
```bash
apt update
apt install -y python3-pip python3-opencv python3-matplotlib
pip3 install numpy
apt install ros-noetic-tf -y
```

### 3. Configure ROS environment
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 4. Build the workspace
```bash
catkin_make
source devel/setup.bash
```

### 5. Make scripts executable
```bash
chmod +x src/grid_mapping_noetic/scripts/create_from_rosbag.py
```

### 6. Run the mapping script
```bash
rosrun grid_mapping create_from_rosbag.py
```

---

## 📂 Repository Structure

```
grid_mapping_noetic/
│── bagfiles/             # Example ROS bag files
│── bagfiles.zip          # Compressed bag files
│── CMakeLists.txt        # Catkin build configuration
│── Dockerfile            # Docker build file (optional)
│── maps/                 # Generated occupancy grid maps
│── package.xml           # ROS package manifest
│── papers/               # Related research papers
│── README.txt            # Original notes
│── scripts/              # Python mapping scripts
│   ├── bresenham.py
│   ├── create_from_rosbag.py
│   ├── grid_map.py
│   ├── message_handler.py
│   ├── rtime_gmapping_node.py
│   ├── utils.py
│   └── __pycache__/
```

---

## 🖼️ Example Outputs

Example maps generated from ROS bag files:

- **House map**
  - `house.png`
  - `house_grid_map.png`
  - `house_grid_map_mle.png`
  - `house_compared.png`

- **Stage 4 map**
  - `stage_4.png`
  - `stage_4_grid_map.png`
  - `stage_4_grid_map_mle.png`
  - `stage_4_compared.png`

- **World map**
  - `world.png`
  - `world_grid_map.png`
  - `world_grid_map_mle.png`
  - `world_compared.png`

---

## 🚀 Usage Example

To process a bag file and generate a grid map:
```bash
rosrun grid_mapping create_from_rosbag.py --bag bagfiles/example.bag --output maps/
```

---

## 📖 References

- ROS Noetic documentation: [http://wiki.ros.org/noetic](http://wiki.ros.org/noetic)
- Occupancy grid mapping concepts
# F1tenth Control Part

## Ros2 humble, Ubuntu 22.04

### python  virtual space

```
python3 -m venv ~/f1racer-venv
source ~/f1racer-venv/bin/activate
python -m pip install --upgrade "pip==23.2.1" "setuptools==59.8.0" "wheel==0.38.4"
pip install pyyaml "flatbuffers>=2.0,<24" "catkin_pkg>=0.5" "transforms3d>=0.4" "empy>=3.3"
```

### F1TENTH Gym 설치

```
cd ~
git clone https://github.com/f1tenth/f1tenth_gym
cd ~/f1tenth_gym
pip install -e .
python -c "import gym; print('gym =', __import__('gym').__version__)"   # 0.21.0 출력이면 OK
```

### ROS 워크스페이스 & ros-racer 받기

```
source /opt/ros/humble/setup.bash        # ROS 환경
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/eclipse-sdv-blueprints/ros-racer.git
```

### sim.yaml 맵 경로 설정

```
nano ~/ros2_ws/src/ros-racer/src/f1tenth_gym_ros/config/sim.yaml
```

##### map_path: /home/ [yours] /ros2_ws/src/ros-racer/src/f1tenth_gym_ros/maps/underground
##### map_img_ext: png

### 빌드 (venu -> ROS -> 빌드)

```
# venv + ROS 로드
source ~/f1racer-venv/bin/activate
source /opt/ros/humble/setup.bash

cd ~/ros2_ws
rosdep install --from-path src --ignore-src -r -y --rosdistro ${ROS_DISTRO} --skip-keys=transforms3d
colcon build --symlink-install
source install/local_setup.bash

# (선택) 실행 스크립트가 venv 파이썬을 가리키는지 확인
head -1 install/f1tenth_gym_ros/lib/f1tenth_gym_ros/gym_bridge
# -> #!/home/hyeonwoo/f1racer-venv/bin/python  면 OK
```

### 실행

```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

#### 맵이 바로 안뜨면 다른 터미널에서 
```
source /opt/ros/humble/setup.bash
ros2 lifecycle set /map_server configure && ros2 lifecycle set /map_server activate
```

#### Fixed Frame 에러 보이면 정적 TF 추가
```
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```








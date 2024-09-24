依赖：

ROS; OpenCV; PCL; yaml-cpp
```angular2html
sudo apt-get install libyaml-cpp-dev
```

编译：
```angular2html
catkin build
```

运行：
```angular2html
source devel/setup.bash
rosrun sensor_simulator sensor_simulator
```

仿真位置发布：
```angular2html
cd src/sensor_simulator
python sim_odom.py

cd src/sensor_simulator
rviz -d rviz.rviz
```

demo:
![Demo GIF](demo.gif)

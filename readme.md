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

其他工具：
```
1. 点云裁剪：map_process.py
2. 点云扩充：map_expand.py
```

demo:
深度图0.02s, 点云0.01s

![Demo GIF](demo.gif)

TODO:

1. CUDA+栅格地图索引

2. 等改为栅格地图以后，取消对地图的复制，而是对栅格索引变到有效范围内，实现无限的地图
# 送分小队	ucar_ws

## 版本日志

### 2023 / 3 / 23

* 上传最初版本代码

### 2023 / 4 / 16

* 添加 `simple_navigation_goals`

### 2023 / 4 / 20

* 完成 `simple_navigation_goals` ，可以发送终点坐标

---

## 常用命令

```sh
# 1. turtlesim
# 打开三个终端，在三个终端中分别输入如下命令
roscore
rosrun turtlesim turtlesim_node
rosrun turtlesim turtle_teleop_key
```

```sh
# 启动模拟程序
roslaunch gazebo_nav dwa_race_demo_singlepath.launch
```

```sh
# 编译
# 进入工作空间，一般是 ~/ucar_ws 目录
catkin_make
```

```sh
# dwa 参数文件目录
~/ucar_ws/src/gazebo_nav/launch/config/move_base/dwa_local_planner_params.yaml
```

文档链接 [dwa_params](/src/gazebo_nav/launch/config/move_base/dwa_local_planner_params.yaml)

```sh
# 发送终点坐标
rosrun simple_navigation_goals simple_navigation_goals
# 终点大概位置参数 0 -5 180
# 测试用位置参数 5 0 0

# 从 rviz 获取终点坐标
# 打开 rviz ，用粉色箭头(2DNavGoal)点击地图
rostopic echo /move_base_simple/goal
```


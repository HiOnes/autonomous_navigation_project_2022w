# autonomous_navigation_project_2022w
BARN Challenge in ICRA 2022 as course project

## TODO
implement your own navigation algorithms in run.py

## Run Simulations
1. 测试导航模块
```
source devel/setup.sh
python3 src/scripts/run.py --gui --world_idx xxx
```
2. 单独测试A*路径规划模块
```
python3 src/scripts/Astar.py
```
3. 单独测试DWA速度规划模块(可能会发现小车到终点之后冲过头，但此模块仅为仿真，在实际gazebo中导航规划不会出现类似情况)
```
python3 src/scripts/DWA.py
```

# Acknowledgements
Code references

# Kill all process
```
bash src/clean.sh
```

[ros_jackal](https://github.com/Daffan/ros_jackal).

[nav-competition-icra2022](https://github.com/Daffan/nav-competition-icra2022).

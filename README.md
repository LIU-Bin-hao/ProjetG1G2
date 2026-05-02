# ProjetG1G2
当然可以。下面是一整份 README.md 文件内容，你可以从第一行开始全部复制，然后粘贴到 GitHub 仓库里的 README.md 中。

# ProjetG1G2

本项目为 `simpletest_ws` 工作区中的源码部分，主要用于管理 ROS/ROS2 项目代码。  
当前仓库建议只保存 `simpletest_ws/src` 目录下的内容，不上传编译生成文件。

---

## 1. 项目简介

本项目包含 ROS/ROS2 功能包源码，用于完成课程/实验/项目中的相关机器人功能开发。

项目代码位于：

```text
ProjetG1G2/simpletest_ws/src

该目录中通常包含一个或多个 ROS 功能包，每个功能包可以包括节点源码、启动文件、配置文件、消息定义文件等内容。

2. 项目结构

推荐的项目目录结构如下：

ProjetG1G2/
└── simpletest_ws/
    └── src/
        ├── README.md
        ├── package_1/
        │   ├── package.xml
        │   ├── CMakeLists.txt
        │   ├── src/
        │   │   └── node_file.cpp / node_file.py
        │   ├── include/
        │   ├── launch/
        │   │   └── launch_file.launch / launch_file.py
        │   ├── config/
        │   │   └── params.yaml
        │   ├── msg/
        │   ├── srv/
        │   └── scripts/
        │
        └── package_2/
            ├── package.xml
            ├── CMakeLists.txt
            ├── src/
            ├── launch/
            └── config/
3. 主要文件说明
文件或目录	说明
README.md	项目说明文件
package.xml	ROS 功能包信息与依赖声明
CMakeLists.txt	编译规则配置文件
src/	节点源代码目录，通常存放 C++ 或 Python 文件
include/	C++ 头文件目录
launch/	启动文件目录
config/	参数配置文件目录
scripts/	Python 脚本或辅助脚本目录
msg/	自定义消息文件目录
srv/	自定义服务文件目录
4. 环境要求

运行本项目之前，请确保已安装以下环境：

Ubuntu Linux
ROS1 或 ROS2
Git
Python3
CMake
colcon 或 catkin

如果使用 ROS1，常见版本为：

ROS Noetic

如果使用 ROS2，常见版本为：

ROS2 Humble
5. 获取项目代码
方法一：直接克隆仓库
git clone https://github.com/LIU-Bin-hao/ProjetG1G2.git

进入项目目录：

cd ProjetG1G2
方法二：克隆到 ROS 工作区的 src 目录

如果需要重新创建工作区，可以执行：

mkdir -p ~/simpletest_ws/src
cd ~/simpletest_ws/src
git clone https://github.com/LIU-Bin-hao/ProjetG1G2.git

如果仓库内容本身就是 src 下的功能包代码，则应保证功能包直接位于：

~/simpletest_ws/src/
6. 编译项目
6.1 ROS2 编译方法

进入工作区根目录：

cd ~/simpletest_ws

编译：

colcon build

编译完成后加载环境变量：

source install/setup.bash

如果希望每次打开终端自动加载，可以执行：

echo "source ~/simpletest_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
6.2 ROS1 编译方法

进入工作区根目录：

cd ~/simpletest_ws

编译：

catkin_make

编译完成后加载环境变量：

source devel/setup.bash

如果希望每次打开终端自动加载，可以执行：

echo "source ~/simpletest_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
7. 运行方法
7.1 查看功能包

ROS2：

ros2 pkg list

ROS1：

rospack list
7.2 运行单个节点

ROS2：

ros2 run <package_name> <node_name>

示例：

ros2 run my_package my_node

ROS1：

rosrun <package_name> <node_name>

示例：

rosrun my_package my_node
7.3 使用 launch 文件启动

ROS2：

ros2 launch <package_name> <launch_file.py>

示例：

ros2 launch my_package demo_launch.py

ROS1：

roslaunch <package_name> <launch_file.launch>

示例：

roslaunch my_package demo.launch
8. 常用开发命令
查看当前工作区状态
git status
添加修改文件
git add .
提交修改
git commit -m "update project code"
推送到 GitHub
git push
拉取远程更新
git pull
9. GitHub 管理说明

本项目使用 GitHub 进行代码版本管理。

仓库地址：

https://github.com/LIU-Bin-hao/ProjetG1G2.git

推荐只上传源码文件，不上传编译生成文件。

不建议上传的目录包括：

build/
install/
devel/
log/

这些目录可以通过重新编译生成，不需要放入 GitHub 仓库。

10. .gitignore 建议

建议在仓库根目录添加 .gitignore 文件，内容如下：

# ROS / ROS2 build files
build/
install/
devel/
log/

# Python cache
__pycache__/
*.pyc
*.pyo

# Editor files
.vscode/
.idea/

# System files
.DS_Store

# Temporary files
*.tmp
*.log
11. 常见问题
11.1 找不到功能包

如果运行时提示找不到 package，请确认已经在工作区根目录编译：

cd ~/simpletest_ws
colcon build

或：

cd ~/simpletest_ws
catkin_make

然后重新加载环境：

ROS2：

source install/setup.bash

ROS1：

source devel/setup.bash
11.2 push 到 GitHub 失败

如果 GitHub 提示不能使用密码登录，需要使用 Personal Access Token。

推送命令：

git push -u origin main

输入用户名：

LIU-Bin-hao

密码位置粘贴 GitHub Token，而不是 GitHub 登录密码。

11.3 push 时提示 non-fast-forward

说明远程仓库已有内容，本地分支落后于远程分支。

可以先执行：

git pull origin main --allow-unrelated-histories --no-rebase

然后再推送：

git push -u origin main
12. 项目维护流程

日常修改代码后，推荐按照以下顺序提交：

git status
git add .
git commit -m "update code"
git push

如果多人协作，修改代码前建议先拉取最新版本：

git pull
13. 作者

LIU-Bin-hao

14. 备注

本 README 用于说明项目结构、编译方式、运行方法以及 GitHub 管理流程。
如项目后续增加新的功能包、节点或 launch 文件，应及时更新本文档。

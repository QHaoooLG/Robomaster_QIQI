# 视觉避坑手册-RM-QIQI

---

## 配置工控环境
- ##### 基本配置

> 所需工具：
> 1. 空U盘(最好64G起步，烧镜像别买惠普u盘，推荐金士顿)

1. ubuntu官网下载所需版本的镜像，用镜像烧录工具，如：`SD Card Formatter`、`Fedora Media Writter`等，把下载的(.iso)文件烧录到空u盘中，制作系统盘
   > 烧录镜像前最好先把u盘格式化，推荐使用`Fedora Media Writter`中的格式化功能
2. 买来的主板装好内存条、固态硬盘，最好直接把主板装到保护壳内再进行配置，以防损坏
   > 工控电源一定要看好是否适配主板的电压电流，低了开不了机，高了会烧主板
3. 开机前先插系统盘再开机，然后看出不出BIOS选择系统选项进入的GUI界面，若没有就开机前按`F2`进入BIOS，在boot选项中把系统盘启动项换到首选项，然后保存并退出
4. 装`ubuntu 22.04 LTS`则在BIOS系统选项GUI界面中选择`Try to install ubuntu`，若是装`ubuntu 20.04 LTS`则选择`ubuntu`，这两项都能进入ubuntu安装界面
5. 安装时，语言选中文，正常安装且附带媒体播放器等第三方功能，正常安装`ubuntu 22.04 LTS`(若是重装则选择`删除当前的ubuntu 22.04 LTS并重新安装`)
6. 安装完毕，检查是否有wifi，没有就买个带天线免驱动安装的wifi接收器，附可购链接
   https://m.tb.cn/h.5E0tk1j5Z840KMd?tk=ENAKWm1A8We
   配好wifi，在`开始`界面中打开`软件更新器`进行更新，更新完毕后重启，新建终端，输入
   ```shell
   sudo apt-get update
   sudo apt-get upgrade
   ```  
   来检查更新
7. 依次完成以下基本配置：
   - 谷歌输入法 
   ```shell
   sudo apt install fcitx-googlepinyin
   ```
   - Git
   ```shell
   sudo apt install git
   ```
   - QQ 
      官网下载
   - Clash -> 加速访问github以及上传下载代码
      队内资源共享或github下载
   - steam++ (瓦特加速器)
      可加速github，涵盖windows和linux平台
   - vscode -> 查看和编辑代码
      官网下载
   - 小鱼一键安装ROS2
   ```shell
   wget http://fishros.com/install -O fishros && sudo bash fishros
   ```
   - 工业相机对应的SDK和驱动工具包
      去品牌官网下载页下载
   - edge浏览器
      推荐使用，在ubuntu上能用的浏览器里算是比较好用的，账号登陆后自动同步收藏夹很方便，教程随手收藏，系统崩了重装不丢失
8. 在主目录下新建文件夹`HOME`，用于存放安装的其他软件或功能包等，自瞄代码工作空间文件夹直接放到主目录下即可
9.  在主目录创建自瞄代码工作空间文件夹`RM-Vision-Main`，然后用github把自瞄代码源代码文件夹`src`放进去，整体呈现编译架构如下：
   ```
   RM-Vision-Main(workspace)
   |
   |——src
   |——log
   |——build
   |——install
   ```
   其中`log`,`install`,`build`是由`src`源码包编译得到
   > 下载安装包前需要终端输入`uname -m`或`sudo dpkg --print-architecture`查看系统架构，选择对应版本的安装包，后续如相机SDK工具包也要看好架构安装
10.   到自瞄代码工作空间`.../RM-Vision-Main`下，终端输入
      ```shell
      colcon build
      ```
   进行编译，把编译过程中的错误一一解决即可
   - 缺少`camera_info_manager.hpp`需要终端输入
      ```shell
      sudo apt-get install ros-humble-camera-info-manager
      ```
   - 运行陈君的自瞄代码需要看源码中每个功能包里的`README.md`装好相关依赖，如`serial_driver`等
      ```shell
      sudo apt install ros-humble-serial-driver
      ```
11.   直到自瞄代码的`src`能成功编译就算整个环境配置完毕了，建议再设置以下仿真工具`rqt`调出其图像显示和debug列表 
12.   编译后运行自瞄代码时可能会提示缺少xacro，需要安装下载对应版本的xacro
      ```shell
      sudo apt install ros-humble-xacro
      ```

- ##### VNC连接配置

1. 链接双方设置静态ip
2. 


---


## ROS2必备工具/包

- ROS2-humble
- concol
- python3
- camera-calibration
- 或小鱼一键安装ROS环境
  

---

## 编译/包

- 每次修改src中的源代码后一定要在/home/dev_ws中重新编译一遍，替换掉上次编译的代码，如此才能实现代码的更新
若只是修改src中某文件或代码中的参数，则直接在工作空间重新编译即可实现自动覆盖，若是更改幅度较大或上次编译中出现error，则需要将上次编译后的包全部删除重新编译
```shell
colcon build	//重新编译的指令
ros2 run	//该命令执行的是install文件夹中的文件，故需要更新其中文件才能在终端中运行新版代码
```

- 每个具体到节点的代码文件，在修改后都需要在对应父类文件夹的setup.py文件中进行相应的配置：
找到entry_points={...}
修改大括号内的内容即可（初次配置好后默认就已经是完整的，无需再配置）

- 在ubuntu上用命令行下载安装某些包时最好保证较好的网速，而且部分包在安装时会定位到外网，也就需要挂梯子，如在conda虚拟环境中用pip安装torch相关工具包
> ubuntu系统的梯子可以用Clash，免费注册一个，有免费的加速节点，第一次用之前还要手动配置一下网络代理IP，之后上外网就先切换为手动网络代理+开启Clash，如果是访问国内网站如csdn等就再切换回自动代理即可

---

## 相机

- 调用hik_camera前一定先source对应工作空间里的.bash/.sh文件(一般是用.bash文件)

- 每次进行完相机标定后需要将opt.yaml改名为camera_info.yaml(与包中的相机参数文件名保持一致)，然后在以下两个路径中替换掉原参数文件，分别是：`./RM-Vision-main/ws/src/ros2_hik_camera-main/config`,`/RM-Vision-main/ws/src/rm_vision-main/rm_vision-bringup/config`

> 安装camera_calibration包时不要盲目跟教程直接调用！！！
> ```shell
> sudo apt install ros-galactic-camera-calibration-parsers
> ```
> 上面这条命令是从csdn教程上抠的，它会在`/opt/ros/humble/`中下载并安装`camera-calibration-parsers`，而不是多数教程会用到的`camera-calibration`
所以一般会调用下面这条下载安装命令
> ```shell
> sudo apt install ros-humble-camera-calibration
> ```

- 工业相机标定流程
  1. 安装相机标定工具包
      ```shell
      sudo apt install ros-humble-camera-calibration-parsers
      sudo apt install ros-humble-camera-info-manager
      sudo apt install ros-humble-launch-testing-ament-cmake
      ```
  2. 官网下载对应品牌工业相机的驱动和SDK
      如海康机器人官网:
      https://www.hikrobotics.com/cn/machinevision/service/download?module=0
  3. 启动相机节点
      1. 先进入包含有相机启动文件的工作空间(事先编译过的)
      2. 手动包含ROS相关工具指令
         ```shell
         ./install/setup.bash
         ```
      3. 启动相机节点(这里以海康相机为例)
         ```shell
         ros2 launch hik_camera hik_camera.launch.py
         ```
  4. 相机标定
      ```shell
      ros2 run camera_calibration cameracalibrator --size 11x8 --square 0.50 image:=/camera/image_raw camera:=/camera
      ```
      `--size`后的参数参考实际使用的标定板的规格，同样`--square`后的参数也需根据实际进行调整，`image:=/`表示图像发布话题，`camera:=/`表示相机名称

      
---

## 仿真

- 调出`rqt`或`rviz2`后可以查看经过cv处理后的图像result_img，在陈君自瞄代码的实际运行情况中，仿真中查看图像的中心处有绘制一个红色小圆圈作为图像中心点，与中心点绘制相关的代码为`rm_auto_aim-main/armor_detector/src/detectoe.cpp`中的`cam_center`参数部分
  该中心点仅供参考图像中心位置，作为实际击打路径的比较点，也可以观察中心点是否在图像中心来判断源码中相机内参文件是否正确对应

- 推荐使用`rqt`进行调参，可在菜单栏中打开debug列表、话题实时发布列表、图像显示框等实用窗口

---

## 君佬部署自瞄代码-流程剖析
##### 全过程基于foxglove实现

- 君佬部署自瞄直播回放
   https://flowus.cn/lihanchen/share/8e866082-3758-447d-a034-aa1ab813a417

- 查看设备接口及其总线id等信息
  ```shell
   lsusb
  ```

- 安装温度传感器查看主板及其连接设备的温度
  ```shell
  sudo apt install lm-sensors
  sensors
  ```

- 计算`src/rm_vision_main/rm_vision_bringup/config/laungh_params.yaml`中的`r_xyz_factor`:
  观察`/tracker/measurement.x`的Plot图，得出x的范围值，如$[-1.88, -1.96]$，
  取差值$0.08，$
  $0.08/4=0.02，$
  $0.02^2=0.0004=4e-4，$
  $0.0004/1.88=2e-4量级 $

- 海康相机限帧率？

- 相机标定后，foxglove中观测`/detector/armors`中的数据，`/armors[]/pose[]/position`中的`norm`数据与相机到装甲板识别中心距离理应基本相等，单位(m)
  "5m内相差0.2m稳态误差大概为 $0.2/5=0.04$，略偏大，尽量控制在 2% 左右，相机FOV小(镜头焦距大)的情况下，稳态误差越大影响越大"

- ubuntu系统之间通过wifi直接终端传输文件，
  ```shell
  scp <document_name> <ubuntu_hostname>@<ubuntu_ip>:~/
  ```
   `document_name`为文件名，带后缀
   `ubuntu_hostname`为ubuntu系统用户名
   `ubuntu_ip`可在ubuntu系统联网后，在终端输入
   ```shell
   ifconfig
   ```
   来进行查看
   `~/`表示传输到目标ubuntu系统的主目录下

- 撒的

---

## 调车(与电控联调)
> 马哥时期调参方法，22级感觉不咋稳

##### 整车进行实战训练前的一次完整自瞄调试流程

> 视觉可进行改动的参数：
> - 相机位姿(xyz/rpy)
> - 曝光时间

> 电控可进行改动的参数：
> - 枪口前推距离
> - 枪口垂直距离
> - 机器人固有时间偏差
> - 视觉计算时间
> - 重力加速度系数
> - 空气阻力系数

#### 1. 确认弹速
弹速是后续调车准确性的保障！！！
正式开始调参前先以同一弹速进行多次发弹，在主控上查看每次发弹的实际弹速，使其稳定在一个较小范围内，然后将稳定后实际平均弹速填入预设的弹速(电控代码中防止读不到测速模块的弹速而设置的宏定义常数)

#### 2. 安装相机并修改其相对位姿参数(xyz/rpy)
自瞄调参前一定要给相机加装保护壳，规范安装到小车图纸的对应位置
该阶段需要根据机械组在图纸上测量imu到镜头表面的距离来修改位姿参数xyz
> imu指c板上相对较大的芯片，xyz坐标系是以imu为原点，右手大拇指朝前方作x轴，食指朝上作z轴，中指垂直于大拇指作y轴进行坐标系建立
> rpy(roll轴 pitch轴 yaw轴)分别是绕x轴、y轴和z轴进行转动的

#### 3. 修改枪口前推距离和枪口垂直距离
电控组在`vision_task.h`的宏定义常量中修改，单位为m
枪口前推距离指的是c板上imu到弹丸推出点的x轴距离
枪口垂直距离为imu到到弹丸推出点的z轴距离
> 以左手大拇指朝前作x轴建系

#### 4. 打靶测试
  1. 打1m靶 -> 此时可视作无空气阻力影响的理想情况，要保证击打位置在正中心
  2. 打3m靶 -> 有空气阻力影响，需要调空气阻力系数，使击打位置下坠程度在可控范围内
  3. 打5m靶 -> 空气阻力影响较大，继续调空气阻力系数，保证能打中装甲板即可
  4. 测试击打旋转/平移中的实车装甲板 -> 调机器人固有偏差(预测时间)，使旋转和平移两种情况下的最终击打效果均为良好即可
   
#### 注意事项

- 
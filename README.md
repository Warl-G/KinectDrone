# KinectDrone
![](https://github.com/guoru123/KinectDrone/raw/master/Mannual/KinectDrone.png)
![](https://github.com/guoru123/KinectDrone/raw/master/Mannual/DroneVideo.png)
Ar.drone飞行器Kinect文档
一、    开发环境
Visual Studio 2013及以上
二、    硬件需求
Kinect、Ar.Drone飞行器
三、    第三方工具
KinectSDK2.0
四、    指令
指令    人体动作    向前    右手向前
起飞/降落    左手举起    向后    右手向后
原地左转    左手向左    向左    右手向左
原地右转    左手向右    向右    右手向右
向上    右手向上    向下    右手向下
飞行动作    左手向上剪刀手（Lasso）    灯光效果    右手中间剪刀手
（Lasso）

五、    操作步骤
1.    进入目录，…\大创\文档、成果、工具汇总\大创开发文件
2.    安装KinectSDK-v2.0_1409-Setup.exe文件
3.    安装完成后将Kienct接入电脑
4.    无人机接入电池，置于开阔地
5.    电脑WiFi连接Ardrone字样的网络
6.    进入目录…\大创\文档、成果、工具汇总\飞行器\work\KinectDrone
7.    双击打开KinectDrone.sln文件
8.    点击上方工具栏start按钮，开始程序
9.    点击start，右侧出现无人机摄像头图像
10.    站于Kinect摄像头正前方1.5~3米，使左侧出现完整上半身骨骼图，开始手势操作
11.    紧急情况点击stop按钮
12.    若飞机无响应，点击emergency/reset emergency按钮

六、    Kinect与飞行器的链接与控制关系









\大创\文档、成果、工具汇总\飞行器\work\KinectDrone\KinectDrone\GestureDetection.cs
GestureDetection类识别人体关节位置，判断符合条件姿态，若符合发送至
\大创\文档、成果、工具汇总\飞行器\work\KinectDrone\KinectDrone\DroneController.cs
DroneController类负责连接与飞行器通讯，根据接收到的GestureDetection类识别结果，下达对应指令
七、    代码
1.    Kinect基本启动代码见官方示例与文档：KinectSDK
2.    \KinectDrone\KinectDrone\GestureDetection.cs
负责对Kinect采集骨骼数据进行分析
GestureDetection.FrameReady(body)

3.    \KinectDrone\KinectDrone\DroneController.cs
负责无人机操控
NavigationData类为飞行数据
可获得如飞行高度Altitude和电量Battery等数据

DroneClient 飞行控制器操控以下指令

pitch 前后 范围[-1,1]
yaw 左右转 范围[-1,1]
gaz 上下 范围[-1,1]
roll 向左右飞行 范围[-1,1]
Takeoff起飞
Land降落
Hover悬停

用法：
_client.Progress(FlightMode.Progressive, pitch: -0.1f);向前
_client.Progress(FlightMode.Progressive, pitch: 0.1f);向后

飞行特技和灯光效果为特殊指令使用Setting对象
settings.Leds.LedAnimation = new LedAnimation(ledAnimationType, 2.0f, 5);
settings.Control.FlightAnimation=new FlightAnimation(flightAnimationType);
_client.Send(settings);

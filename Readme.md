#Readme
###ROSControlPanel使用说明
####1、启动软件
在命令行中进入ROSControlPanel文件夹，输入./ControlPanel.py或python ControlPanel.py命令后回车后进入程序
####2、新建工作环境
在File菜单中点击New选项，在弹出窗口中选择launch文件和rviz文件路径，选择Continue按钮后，界面会启动一个rviz，同时后台会运行对应的launch文件
####3、小工具
在tools菜单中一共有两种小工具，一种是Arbotix GUI，是一种通过鼠标拖动控制小车运行方向的工具，在启动了rviz界面后可运行。另一个是配置ROS主从机工具。
###注意事项
本软件尚有未完善之处，偶有可能崩溃闪退的可能，所有均使用ROS自带的PyQt库编写。

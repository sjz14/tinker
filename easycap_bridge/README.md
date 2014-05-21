###双目采集、转换到ROS格式<br />
<br />
##使用方法<br />
使用此命令运行：<br />

    rosrun easycap_bridge easycap_bridge_node

使用此命令，在CV窗口上按键选择左眼：<br />

    rosrun easycap_bridge select_left_eye.py

<br />
##安装方法<br />
Ubuntu 12.04.3, 内核版本 3.8.0-29<br />
具体请参考 https://code.google.com/p/easycap-somagic-linux/wiki/GettingStarted ，此处只列出关键文件<br />
<br />
按教程安装需要的软件<br />
<br />
从win驱动光盘得到 SmiUsbGrabber3C.sys <br />
<br />
编译源代码下 kernel 文件夹，得到 smi2021\_bootloader.ko smi2021.ko<br />
用 modprobe 命令加载 .ko 文件，如果失败用 dmesg 查看依赖关系<br />
<br />
<br />

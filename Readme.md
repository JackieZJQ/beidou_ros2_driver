## 本文件包为北云惯导程序, 参考Apollo程序修改而来

## 1.serial.h与serial_stream.cc为串口打开, 读写文件, 其中stream.h为父类，serial_stream.cc为stream.h的子类,stream.h包含很多虚函数，具体实现在serial_stream.cc; raw_sream会构造stream类的指针, 同时使用     serial_stream.cc赋值构造. raw_stream调用stream读取串口流.

## 2.data_parser为ros发布类, novatel_parser为串口信息处理类, data_parser持有一个novatel_parser的指针, raw_stream为data_stream类的指针.

## 3.novatel_message为惯导设备数据结构.

## 4.可用以下命令设置usb串口别名
## 4.1打开文件
## sudo gedit /etc/udev/rules.d/beidou-usb-serial.rules

## 4.2增加如下命令, idVendor与idProduct可用lsusb命令查看, SYMLINK为设备别名, 最后完整设备别名为/dev/beidou
## KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", SYMLINK+="beidou"

## 4.3重启udev
## sudo udevadm control --reload-rules
## sudo service udev restart
## sudo udevadm trigger

## 4.4查看别名
## ll /dev/beidou

## 5.坐标系
## CORRIMUDATA：右手系，XYZ -> 前左上
## INSPVA: roll -> 前，pitch -> 右，azimuth -> 航向角


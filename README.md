这个代码是关于项目“基于计算机视觉技术的集装箱起吊安全监测”。
在港口货物运输中，需要起重机将集装箱吊起，来给卡车卸货或者装货，但是在集装箱与卡车是在四个角是锁住的，如果在起吊过程中集装箱与卡车没有全部分离，而且未被发现，就会发生安全事故，项目设计内容就是通过采集来的视频，利用计算机视觉算法、视频及图像处理算法来判断集装箱与卡车是否完全分离。目标就是识别起吊情况，防止卡车被吊起，起到安全监测的重大作用。下面将做详细的背景及技术介绍。

此算法主要包括以下七个步骤：

1.获取初始化数据； 

2. 计算卡车与集装箱交界位置的大致范围与直线检测范围；

3.对图像感兴趣区域执行LSD算法，检测直线线段；

4.计算集装箱与卡车较为精确的交界位置；

5.在交界位置进行标记，获得要跟踪的点集；

6.对标记点进行运动目标跟踪;

7.触发起吊信号，每一帧给出一个判断结果直到给出最终结果或触发结束信号。

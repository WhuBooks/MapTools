
MapMatch.cpp : 进行路网匹配，最少要有一个路口和两条道路。

LaneMatch.cpp : 进行多车道匹配，发送当前车身周围多车道轨迹信息，不存在路口，不存在全局规划，可接受毫米波雷达进行速度调控。将不同车道的轨迹文件放入程序所在文件夹下的Model文件夹内，自动抽稀与排序。

Common.h : LaneMatch与MapMatch共同使用的头文件，包括通讯模块的初始化。

LaneMessage.h : 车道信息，从左向右依次排序。

RoadMessage.h : 道路信息，包括路口外、入路口、路口中、出路口三种状态。

GlobalMessage.h : 全局信息，用于指导离开地图的车辆回到地图中

ReverseMessage.h : 逆向道路信息，用于路网中查询调头道路。

VelocityMessage.h : 根据车身位置和毫米波检测结果，对建议速度进行调控

SceneMessage.h : 匹配道路上的场景点，目前定义三种场景：1）减速点，只减速；2）路口点：先减速，再停车检测；3）让行点：提醒规划开启让行功能。场景文件格式：[id,roadid,x,y,yaw,type,length,width]
	

	
	
	
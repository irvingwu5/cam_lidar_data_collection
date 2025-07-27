# TanwayLidarSDK2[v2.0.5]

## 1 介绍

TanwayLidarSDK2是探维科技针对所售雷达产品开发的SDK开发包，主要实现激光雷达原始UDP数据的解析和计算，以获取三维点云坐标信息、强度信息等属性的功能，支持客户进行定制化的功能开发和集成，帮助用户在多种操作系统中快速构建基于激光雷达点云的应用程序。

## 2 SDK的下载

git下载链接如下：

```bash
git clone https://github.com/TanwayLab/TanwayLidarSDK.git
```

说明，TanwayLab/TanwayLidarSDK仓库更新频率相对较低，可通过技术支持人员获取下载最新版本，下载压缩包再解压到指定目录。

## 3 编译支持

- ##### Windows平台

  MSVC（VS2019已测试），将开发包的SDK文件夹直接拷贝至工程中，如果仅使用本SDK进行测试，可以直接将demo文件夹下的Demo_UseSDK.cpp文件导入到工程中，并作为程序入口（demo文件中含有main()入口），进行编译即可；集成到具体项目中使用可直接将文件夹TanwayLidarSDK下除demo文件夹外的源码拷贝至开发项目中，并添加所有.h文件即可使用。

  如果仅使用本SDK进行测试，可切换到demo所在文件夹执行命令 `cmake .. -G "Visual Studio 16 2019"`，通过VS2019打开生成的run_demo.sln工程，设置release  ×64模式，在项目run_demo上右键，“设为启动项目”后，即可编译运行。
- ##### Ubuntu平台

  g++ (ubuntu18.04,ubuntu20.04已测试) ，如果仅使用本SDK进行测试，可进入到下载完成后的TanwayLidarSDK目录下，执行cmake . && make命令，编译成功后执行./run_demo命令，即可运行示例程序；集成到具体项目中可直接使用除demo文件夹外的源码进行开发。

## 4 文件结构

![文件结构](./doc/pics/文件结构.png)

SDK中包含以下文件：

1. about：定义了常量字符串以表示不同雷达型号；定义是否采用调试模式、版本号；提供辅助函数用于将整型的雷达型号转换为对应字符串表示；

2. config：algo_table.json文件用于配置点云处理算法，如需更改算法参数请联系技术支持；

3. demo：示例代码，支持在线和回放模式；

4. doc：包含一系列说明文档：

   [参数详解](doc/参数详解.md)：详细介绍了include文件夹下`common.h`和`user_define.h`中定义的各参数含义；

   [接口说明](doc/接口说明.md) ：介绍了 `ILidarDevice.h` 中定义的纯虚函数接口类，解释接口的功能和用途；

   [状态码详解](doc/状态码详解.md)：提供了一系列表示错误码和提示代码的列表；

   [点云解算流程介绍](doc/点云解算流程介绍.md)：以 Tensor48 型号雷达为例，详细介绍 SDK 如何解析和发布雷达扫描数据。

5. include：定义点云类型结构、错误码信息等；

6. lib：算法链接库；

7. lidar：`LidarDevice`类继承于`ILidarDevice`接口类，并实现了该接口所要求的方法。`LidarDevice`类主要用于管理激光雷达设备的操作，如启动、停止、设置参数、解析点云数据等；

8. pics：保存README中图片；

9. utils：工具函数，如提供数据缓存、获取系统时间、进制转换等函数。

**如需详细参数介绍等，可参考doc文件下的说明文档。**

## 5 使用示例

### 5.1 连接实时的雷达设备

创建SDK实例对象，其中UserPoint为自定义的点结构体类型。结构体可以扩展的参数在include/user_define.h中有定义，可根据实际需要释放/注释所需的点属性，具体参数介绍请参考 [参数详解](doc/参数详解.md)。如果安装了PCL库或ROS系统，可直接定义宏USE_FOR_ROS基于PCL的扩展点。最后将雷达IP地址、本机IP、数据接收端口、回调类对象指针、雷达型号作为参数：

```cpp
auto lidar = ILidarDevice::Create("192.168.111.51", "192.168.111.204", 5600, 5700, &lidarObserver, LT_TempoA4);  //在线模式连接雷达
```

定义点云数据回调函数、异常信息回调函数，各回调函数均在子线程中运行，请避免在回调函数中进行耗时操作或直接操作UI对象。

启动实例，此时如果雷达正常连接将可以在lidarObserver回调函数中获取到相应的数据，输出结果参考 [5.4节](#5.4 **回调函数输出打印信息**)

```cpp
lidar->Start();
```

### 5.2 回放.pcap文件

创建SDK实例对象，仅参数与实时连接雷达时不同，其他一致。参数：pcap文件路径（绝对路径）、雷达的IP地址、数据接收端口、回调类对象指针、雷达型号。

注意：在Win环境下使用绝对路径要使用双斜杠，即`C:\Users\Downloads`应写为`C:\\Users\\Downloads`

输出结果参考 [5.4节](#5.4 **回调函数输出打印信息**)

```cpp
auto lidar = ILidarDevice::Create("test.pcap", "192.168.111.51", 5600, 5700, &lidarObserver, LT_TempoA4);
lidar->Start();
```

### 5.3 **算法的使用**

创建算法实例对象，参数：雷达型号，算法配置json文件的路径（绝对路径）

```cpp
auto algo = ILidarAlgo::Create(LT_Scope128, "algo_table.json");
lidar->SetLidarAlgo(algo.get());
```

如果不需要使用算法，仅使用SDK进行解析点云数据，则删除上述的两行代码，并在CmakeList中根据系统和平台，取消对算法库的依赖：

![取消算法依赖](./doc/pics/取消算法依赖.png)

### 5.4 **回调函数输出打印信息**

当点云数据触发回调时，会打印当前帧的点云信息，其中width对应点云数量，height在回调函数中设置固定为1，注意：第一帧数据会较少，因为雷达启动时不能保证从镜面0°开始扫描，所以第一帧数据会不完整

```cpp
virtual void OnPointCloud(const LidarInfo &lidarInfo, const UserPointCloud &pointCloud){
	/*
	*The point cloud struct uses a const reference. 
	*Please copy the point cloud data to another thread for use.
	*Avoid directly operating the UI in the callback function.
	*/
	std::cout << "width:" << pointCloud.width 
			  << " height:" << pointCloud.height 
			  << " point cloud size: " << pointCloud.points.size() << std::endl;
  }
```

**正常运行后输出的信息**

<img src="./doc/pics/正确运行demo的结果.jpg" alt="正确运行demo的结果" style="zoom:67%;" />

其中【algo】打印信息代表现在生效的算法，与雷达型号和algo_table.json文件中算法开关的配置有关，如果使用的雷达型号在json文件中没有开启任何算法，或者没有创建算法实例，则不会打印出【algo】的信息

### 5.5 **连接多台雷达**

1. 在Demo_UseSDK.cpp中创建多个雷达实例，最后一位是雷达ID，用于区分不同雷达实例

```cpp
int main()
{
    LidarObserver lidarObserver;
    // 此处演示离线模式
    auto lidar_0 = ILidarDevice::Create("test1.pcap", "192.168.111.51", 5600, 5700, &lidarObserver, LT_Scope128H, true,0);
    auto lidar_1 = ILidarDevice::Create("test2.pcap", "192.168.111.51", 5600, 5700, &lidarObserver, LT_Scope128, true,1);

    lidar_0->Start();
    lidar_1->Start();


    // 以下代码演示了每三秒进行一次雷达的开启和关闭操作。
    // 正常情况下，用户无需频繁执行此操作。调用 lidar->Start() 启动雷达后，根据需要调用 lidar->Stop() 停止雷达即可。
    bool run_t = false;
    while (true){
        std::this_thread::sleep_for(std::chrono::seconds(3));
        if (run_t){
            lidar_0->Start();
            lidar_1->Start();
            std::cout << "===========: start()" << std::endl;
            run_t = false;
        }
        else{
            lidar_0->Stop();
            lidar_1->Stop();
            std::cout << "===========: stop()" << std::endl;
            run_t = true;
        }
    }

    return 0;
}
```

2. 如果需要区分每个点所属雷达，需要在user_define.h中增加字段，根据自己的环境在相应的`UserPoint`下增加`int lidar_id=-1`，并在common.h中增加语句，用于将雷达id在回调时赋值给每个点

```cpp
MemberCheck(lidar_id);

template <typename PointT>
inline typename std::enable_if<!PointT_HasMember(PointT, lidar_id)>::type
setLidar_id(PointT &point, const unsigned int &value) {}

template <typename PointT>
inline typename std::enable_if<PointT_HasMember(PointT, lidar_id)>::type
setLidar_id(PointT &point, const unsigned int &value) {
  point.lidar_id = value;
}
```

3. 在lidar/LidarDevice.cpp文件中的回调函数内增加： `setLidar_id(basic_point, _lidarInfo.lidarID)`见下面第15行

~~~cpp
void LidarDevice::PointCloudCallback(TWPointCloud::Points &points,
                                     bool callback, uint64_t frameIndex) {
  if (_isLidarReady) {
    if (_lidarAlgo)
      _lidarAlgo->Process(points);

    int pointSize = points.size();

    for (int i = 0; i < pointSize; i++) {
      `````````````//省略此处代码
      setT_usec(basic_point, point.t_usec);
      setAPDTemp(basic_point, point.apd_temp);
      setPulseCodeInterval(basic_point, point.pulseCodeInterval);

      setLidar_id(basic_point, _lidarInfo.lidarID);
      
      _pointCloudPtr->push_back(std::move(basic_point));
    }

    if (callback) {
      ``````````````````````//省略此处代码
    }
  }
}
~~~

4. 即可通过访问`point.lidar_id`来判断每一个点所属雷达

5. 回放两个pcap包的运行结果：

<img src="./doc/pics/两台雷达pcap包运行结果.jpg" alt="两台雷达pcap包运行结果" style="zoom:67%;" />






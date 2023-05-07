## TF tools  
TF库的目的是实现系统中任一个点在所有坐标系之间的坐标变换，也就是说，只要给定一个坐标系下的一个点的坐标，就能获得这个点在其他坐标系下的坐标。  
### 监听tf变换  
接收并缓存系统中发布的所有参考系变换，并从中查询所需要的参考系变换。
1）lookupTransform();
定义监听器：tf::TransformListener listener
定义存放变换关系的变量：tf::StampedTransorm transform;

try{
      listener.lookupTransform("/map", "/base_link",
                               ros::Time(0), transform);
      //这里监听了base_link到map的坐标变换，并将参数存放到transform中。
}
catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
}

transform.getOrigin().getX()；
transform.getRotation().getX()；

由于tf的会把监听的内容存放到一个缓存中，然后再读取相关的内容，而这个过程可能会有几毫秒的延迟，也就是，tf的监听器并不能监听到“现在”的变换，所以如果不使用try,catch函数会导致报错：
“world” passed to lookupTransform argument target_frame does not exist. ”

注意到ros::Time(0)的使用是获取了某一个tf变换中最后一次的数据，注意是最后一次，不是当前时间的。所以对于某些tf时间性要求比较高的场合，比如物体时别。当我从某个位置时别到物体后，发布一个相对于物体的TF。然后我再次运动，时别物体，此时假设由于光线等因素影响其实可能我的算法没有时别出来物体，但是使用ros::Time(0)仍然是可以获取到相对变换的，这个变换就是最后一次时别到物体时的相对TF，显然这样子的结果可能会是错误的。

那如果我们需要得到当前时间戳下的TF而不是很早之前的TF应该怎么做？很简单的方式就是增加一个waitfortransform：

将
listener.lookupTransform("/map", "/base_link",ros::Time(0), transform);
改为
ros::Time now = ros::Time::now();
listener.waitForTransform("/turtle2", "/turtle1",now, ros::Duration(3.0));
listener.lookupTransform("/turtle2", "/turtle1",now, transform);

waitForTransform与ros::Time::now()连用可以获取当前最新时间戳下的TF变换，ros::Duration(3.0)代表你能接受的最大时间戳差异，3就是当前时间戳3秒内的数据。因为时间戳完全对齐基本是不可能的，所以一般这里会设置一下给定一个范围。如果获取到的TF不是当前时间戳的一定容许范围内的，数据会被舍弃，这样子下面的lookupTransform就不会执行。

2）transformPoint（）；
这个在传感器数据的坐标变换中使用的比较多，用来将一个传感器的数据从一个坐标系转换到另外一个坐标系下。
listener_.transformPoint(“map”,laser_pose,map_pose)；
1）其中laser_pose,world_pose的数据类型都是 geometry_msgs::PointStamped， 需要定义laser_pose.header.frame.id即该点所属的坐标系（比如激光坐标系）
2）”map“是指，我要将laser_pose转换到map坐标系下，map_pose是转换的结果。


geometry_msgs::PointStamped turtle1;
turtle1.header.stamp=ros::Time();
turtle1.header.frame_id="turtle1";
turtle1.point.x=1;
turtle1.point.y=2;
turtle1.point.z=3;
geometry_msgs::PointStamped turtle1_world;
        try{
listener_.transformPoint("PTAM_world",turtle1,turtle1_world);
         }
             catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
    }
这里将turtle1从turtle1坐标系转换到PTAM_world坐标系下，数据存储在turtle1_world下


3）transformLaserScanToPointCloud（）；
这个TF一般用于激光SLAM中对于激光点云转换到其他坐标系下，例如将来自激光的数据转换到map下，使用transformPoint也可以转但是会比较麻烦，使用transformLaserScanToPointCloud可以一次性转换。

tf::TransformListener tf_listener; 
laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud mapcloud; 
void XXX::ScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    if(!tf_listener.waitForTransform(
            scan_in->header.frame_id,
            "/map",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
           ros::Duration(1))){
        ROS_INFO("timestamp error");
        return;
    }   
    try
    {
        projector_.transformLaserScanToPointCloud("/map",*scan_in,mapcloud,tf_listener);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("%s", e.what());
    }	
	//点云处理：	    
}



### 广播tf变换  
向系统中广播参考系之间的坐标变换关系。系统中更可能会存在多个不同部分的tf变换广播，每个广播都可以直接将参考系变换关系直接插入tf树中，不需要进行同步。

TFBroadCaster::TFBroadCaster(std::string frame_id, std::string child_frame_id) {
    transform_.frame_id_ = frame_id;
    transform_.child_frame_id_ = child_frame_id;
}

void TFBroadCaster::SendTransform(Eigen::Matrix4f pose, double time) {
    Eigen::Quaternionf q(pose.block<3, 3>(0, 0));
    ros::Time ros_time(time);
    transform_.stamp_ = ros_time;
    transform_.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    transform_.setOrigin(tf::Vector3(pose(0, 3), pose(1, 3), pose(2, 3)));
    broadcaster_.sendTransform(transform_);
}


### 数据结构  
数据结构，基本的数据类型有：Quaternion, vector, point, pose, transform
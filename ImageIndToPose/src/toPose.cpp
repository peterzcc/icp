#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseArray.h>
// PCL specific includes
// #include <pcl/ros/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

using namespace std;
using namespace ros;

int width = 640;
ros::Publisher pub;

class app {
	vector<int> indices;
public:
	void initIndices();
	vector<int>  getInd() const { return indices; } 
	void setInd(vector<int> newInd) {indices = newInd; }
	void cloudCallback(const pcl::PCLPointCloud2ConstPtr& inputCloud);
	void indCallback(const std_msgs::Int32MultiArray& indices);
};

void app::cloudCallback(const pcl::PCLPointCloud2ConstPtr& inputCloud)
{
	if (indices.size()==0) return;
	
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	// cloud.width = 307200;
 //  	cloud.height = 1; 
  	
  	pcl::fromPCLPointCloud2(*inputCloud, cloud);
  	geometry_msgs::PoseArray newPoseArray;
  	newPoseArray.header.frame_id = "/camera_rgb_optical_frame";
    newPoseArray.header.stamp = ros::Time::now();
    newPoseArray.poses.resize(indices.size());

    for (int i=0;i<indices.size();++i){
    int index = indices[i];
	newPoseArray.poses[i].orientation.x = 0.0;
	newPoseArray.poses[i].orientation.y = 0.0;
	newPoseArray.poses[i].orientation.z = 0.0;
	newPoseArray.poses[i].orientation.w = 1.0;
    newPoseArray.poses[i].position.x = cloud.points[index].x;
    newPoseArray.poses[i].position.y = cloud.points[index].y;
    newPoseArray.poses[i].position.z = cloud.points[index].z;
    // ROS_INFO("Good: %d",cloud.points[0].r);
	}
	pub.publish(newPoseArray);
}

void app::initIndices()
{
	indices.resize(4);
	indices[0] = 0;
	indices[1] = (240-1)*640+0;
	indices[2] = (240-1)*640+320;
	indices[3] = (240-1)*640+630;
}

void app::indCallback(const std_msgs::Int32MultiArray& newInd)
{
// # A standard, 3-channel 640x480 image with interleaved color channels
// # would be specified as:
// #
// # dim[0].label  = "height"
// # dim[0].size   = 480
// # dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
	indices = newInd.data;
}

int main (int argc, char** argv)
{	

//   // Initialize ROS
	ros::init (argc, argv, "toPose");
	ros::NodeHandle nh;
	app appObj;
	appObj.initIndices();
    ros::Subscriber subToCloud = nh.subscribe("camera/depth/points", 1, &app::cloudCallback,&appObj);
    // ros::Subscriber subToInd = nh.subscribe ("indices", 1, &app::indCallback,&appObj);
    pub = nh.advertise<geometry_msgs::PoseArray> ("facePoses", 1);

//   // Spin
   ros::spin();
} 
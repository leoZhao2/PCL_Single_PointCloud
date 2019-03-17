#include <iostream> //标准输入输出流
#include <pcl/io/pcd_io.h> //PCL的PCD格式文件的输入输出头文件
#include <pcl/point_types.h> //PCL对各种格式的点的支持头文件
#include <pcl/visualization/cloud_viewer.h>
#include <string>
using namespace std;

// OpenCV 库
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// 定义点云类型
typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloud;

// 相机内参
const double camera_factor = 1000;
const double camera_cx = 633.0;
const double camera_cy = 404.0;
const double camera_fx = 682.0;
const double camera_fy = 682.0;

// 主函数 
int main(int argc, char** argv)
{
	cv::Mat rgb, depth;
	rgb = cv::imread("img.png");
	depth = cv::imread("depth.png", -1);
	// 使用智能指针，创建一个空点云。这种指针用完会自动释放。
	PointCloud::Ptr cloud(new PointCloud);
	// 遍历深度图
	for (int m = 0; m < depth.rows; m++)
		for (int n = 0; n < depth.cols; n++)
		{
			// 获取深度图中(m,n)处的值
			ushort d = depth.ptr<ushort>(m)[n];
			// d 可能没有值，若如此，跳过此点
			if (d == 0)
				continue;
			// d 存在值，则向点云增加一个点
			PointT p;

			// 计算这个点的空间坐标
			p.z = double(d) / camera_factor;
			p.x = (n - camera_cx) * p.z / camera_fx;
			p.y = (m - camera_cy) * p.z / camera_fy;

			// 从rgb图像中获取它的颜色
			// rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];
			cloud->points.push_back(p);
		}
	// 设置并保存点云
	cloud->height = 1;
	cloud->width = cloud->points.size();
	//cout << "point cloud size = " << cloud->points.size() << endl;
	cloud->is_dense = false;
	pcl::io::savePCDFile("Test0109.pcd", *cloud);
	pcl::visualization::CloudViewer viewer("pcd viewer");
	viewer.showCloud(cloud);
	// 清除数据并退出
	cloud->points.clear();
	cout << "Point cloud saved." << endl;
	system("pause");
	return (0);
}
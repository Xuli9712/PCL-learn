#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;


int main(int argc, char **argv)
{
    //创建一个PointCloud类的对象
    //通过<>内的数据类型pcl::PointXYZ创建pcl::PointCloud<pcl::PointXYZ>类，并创建对象cloud
    //pcl::PointXYZ是pcl库的基础数据类型之一， PointCloud是一个点云类
    pcl::PointCloud<pcl::PointXYZ> cloud;
    //fill in cloud data
    cloud.width = 5; 
    cloud.height = 1; 
    cloud.is_dense = false;
    //cloud.points是构建点云的点向量
    cloud.points.resize(cloud.width*cloud.height);
    //遍历cload中的点point
    for (auto &point:cloud)
    {
        point.x = 1024*rand()/(RAND_MAX + 1.0f);
        point.y = 1024*rand()/(RAND_MAX + 1.0f);
        point.z = 1024*rand()/(RAND_MAX + 1.0f);
    }

    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);
    cerr << "Saved" << cloud.size() << "data points to test_pcd.pcd" << endl;

    for (const auto &point:cloud)
    {
        cerr << " " << point.x << " " << point.y << " " << point.z << endl;
    }
    
    return 0;



}
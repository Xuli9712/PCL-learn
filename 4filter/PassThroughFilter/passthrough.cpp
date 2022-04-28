//直通滤波
//在指定轴(field)上根据范围(limit)进行滤波，如下方代码
//在z轴上(0.0 400.0)的范围进行过滤
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h> //直通滤波
#include <pcl/visualization/pcl_visualizer.h> //可视化

using namespace std;

int main(int argc, char **argv)
{
    //cloud before filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    //cloud after filtering
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    cloudPtr->width = 20;
    cloudPtr->height = 1; //无序点云
    cloudPtr->resize(cloudPtr->width*cloudPtr->height);

    for(size_t i=0; i<cloudPtr->size(); i++)
    {
        cloudPtr->points[i].x = 1024.0f*rand()/(RAND_MAX + 1.0f);
        cloudPtr->points[i].y = 1024.0f*rand()/(RAND_MAX + 1.0f);
        cloudPtr->points[i].z = 1024.0f*rand()/(RAND_MAX + 1.0f);
    }
    //输出滤波前的点云
    cout << "Cloud before filtering" << endl;
    for(size_t i=0; i<cloudPtr->size(); i++)
    {
        cout << cloudPtr->points[i].x << " "
             << cloudPtr->points[i].y << " "
             << cloudPtr->points[i].z << " " << endl;
    }
    //pass through核心代码
    //创建直通滤波对象 creat passthrough filter object
    pcl::PassThrough<pcl::PointXYZ> passthrough;
    //输入点云指针
    passthrough.setInputCloud(cloudPtr);
    //设置滤波作用域和范围，此处也就是滤掉范围在z坐标在(0,400)的点
    passthrough.setFilterFieldName("z");
    //设置滤波范围及是否获取范围之外的内容
    passthrough.setFilterLimits(0.0,400.0);
    passthrough.setFilterLimitsNegative(false);
    //得到滤波后点云
    passthrough.filter(*cloudPtr_filtered);

    //输出滤波后的点云
    cout << "Cloud after filtering" << endl;
    for(size_t i=0; i<cloudPtr_filtered->size(); i++)
    {
        cout << " " << cloudPtr_filtered->points[i].x
             << " " << cloudPtr_filtered->points[i].y
             << " " << cloudPtr_filtered->points[i].z << endl;
    }
    //可视化
    pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.4);
    viewer.addPointCloud(cloudPtr, "cloud1");
    viewer.addPointCloud(cloudPtr_filtered, "cloud2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

return 0;
}

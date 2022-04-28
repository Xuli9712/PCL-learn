//statistical outlier removal 统计学离群点移除
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h> //统计离群点移除SOR
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> //读写滤波前后的点云和pcd文件
#include <pcl/visualization/pcl_visualizer.h> //可视化

int main(int argc, char** argv)
{
    //create pointcloud pointer object and define cloud type 创建点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //create pcd reader object and read pcd file into cloud
    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd", *cloudPtr);

    //滤波核心代码
    //create SOR filter 
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> SOR;
    SOR.setInputCloud(cloudPtr);
    //设置平均距离估计的最近邻个数
    SOR.setMeanK(50);
    //设置标准差阈值
    SOR.setStddevMulThresh(1.0);
    //执行滤波filter
    SOR.filter(*cloudPtr_filtered);

    //滤波后点云写入PCD
    pcl::PCDWriter writer;
    writer.write("table_scene_lms400.pcd", *cloudPtr_filtered);
    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    viewer.addPointCloud(cloudPtr_filtered, "cloud");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }


    return 0;
}
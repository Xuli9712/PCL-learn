//采用voxel grid对点云数据进行下采样，在保持形状特征的前提下减少数据量
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    //滤波前的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    //绿波后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    //创建PCD Reader类的实例reader，从文件中读取点云
    pcl::PCDReader reader;
    reader.read("table_scene_lms400.pcd", *cloudPtr);

    //创建一个提速为边长0.01m的voxelGrid
    float leftSize = 0.01f;
    pcl::VoxelGrid<pcl::PointXYZ> voxelfilter;
    //输入点云
    voxelfilter.setInputCloud(cloudPtr);
    //set leaf size
    voxelfilter.setLeafSize(leftSize, leftSize, leftSize);
    //取得过滤后的点云
    voxelfilter.filter(*cloudPtr_filtered);

    //写入过滤前后的点云PCD
    pcl::PCDWriter writer;
    writer.write("table_scene_lms400_filtered.pcd", *cloudPtr_filtered);

    //可视化
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.0, 0.0, 0.5);
    //viewer.addPointCloud(cloudPtr, "cloud1");
    viewer.addPointCloud(cloudPtr_filtered, "cloud2");

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
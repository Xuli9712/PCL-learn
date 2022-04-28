//通过点云创建深度图(距离图像) PointCloud2RangeImage
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/range_image/range_image.h> //引入深度图头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h> //深度图可视化

using namespace std;
//RangeImage类继承于PointCloud类
int main(int argc, char** argv)
{
    //创建点云，new在堆中申请新的空间
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr (new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZ> &cloud = *cloudPtr;
    //读取pcd文件中的点云
    pcl::io::loadPCDFile("table_scene_lms400.pcd", *cloudPtr);

    //设置深度图像参数
    //角分辨率为弧度制1度
    float angularResolution = (float)(1.0f * (M_PI/180.0f));
    //水平方向最大采样角为弧度制360度
    float maxAngleWidth = (float)(360.0f * (M_PI/180.0f));
    //垂直方向最大采样角为弧度制180度
    float maxAngleHeight = (float)(180.0f * (M_PI/180.0f));
    //噪声程度，即周围点对深度值的影响程度
    float noiselevel = 0.00;

    //设置最小获取距离。若该值大于0，则小于获取距离为盲区
    float minRange = 0.0f;
    //获得深度图的边缘宽度
    int borderSize = 0;
    //Eigen库创建仿射变换
    //sensor pose定义传感器的姿态：roll, pitch, yaw,默认初始值军为0
    Eigen::Affine3f sensorPose;
    sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f); //采集传感器位姿
    //设置坐标轴
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;   //相机坐标


    //创建深度图对象
    boost::shared_ptr<pcl::RangeImage> rangeImagePtr (new pcl::RangeImage);
    //pcl::RangeImage rangeImage;
    pcl::RangeImage &rangeImage = *rangeImagePtr;

    //从点云创建深度图
    rangeImagePtr->createFromPointCloud(*cloudPtr, angularResolution, maxAngleWidth, maxAngleHeight, 
                                        sensorPose, coordinate_frame, noiselevel, minRange, borderSize);

    cout << rangeImagePtr << endl;

    //可视化
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.setBackgroundColor(1, 1, 1);
    
    
    //颜色设定
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(rangeImagePtr, 0.5, 0.0, 0.0);
    //渲染设置
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "range_image");
    //添加深度图点云
    viewer.addPointCloud(rangeImagePtr, range_image_color_handler, "rangeImage");
    
    //颜色设定
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloudPtr, 0.0, 0.0, 0.0);
    //渲染设置
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
    //添加原点云
    viewer.addPointCloud(cloudPtr, cloud_color_handler, "cloud");
    //vierwer主循环
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
    return 0;
}
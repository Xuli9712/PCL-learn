//RadiusOutlierRemoval半径离群值移除
//ConditionalRemoval条件滤波
#include <iostream>
#include <pcl/filters/radius_outlier_removal.h>//半径离群值移除
#include <pcl/filters/conditional_removal.h>//条件滤波
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h> //输入输出pcd点云文件
#include <pcl/visualization/pcl_visualizer.h> //可视化
using namespace std;

//设置点云的数据类型,简化代码
typedef pcl::PointXYZ xyz;

//可视化，定义点云显示函数 void showPointCloud(),输入两个创建的点云（滤波前后）
//指针传参
void showPointCloud(const pcl::PointCloud<xyz>::Ptr &cloud1, const pcl::PointCloud<xyz>::Ptr &cloud2)
{
    //创建可视化用的Viewer
    pcl::visualization::PCLVisualizer viewer("3D Viewer");
    viewer.addCoordinateSystem(1.0);
    viewer.setBackgroundColor(0.05, 0.05, 0.0);
    //创建第一个点云的颜色
    pcl::visualization::PointCloudColorHandlerCustom<xyz> color1(0, 255, 0);
    //添加第一个点云
    viewer.addPointCloud<xyz>(&cloud1, color1, "cloud1");
    //设置第一个点云渲染属性
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud1");
    //创建第二个颜色
    pcl::visualization::PointCloudColorHandlerCustom<xyz> color2(255, 0, 0);
    //添加第二个点云
    viewer.addPointCloud<xyz>(&cloud2, color2, "cloud2");
    //设置第二个点云渲染属性
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud2");
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }
}


int main(int argc, char **argv)
{
    //命令行需要选择半径离群值移除还是条件滤波，判断命令行内容个数
    if(argc != 2)
    {
        cerr << "Please specify command line argument '-r' or '-c'" << endl;
        exit(0);//退出当前运行函数
    }
    //创建滤波前后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    cloudPtr->width = 100;
    cloudPtr->height = 1;//无序点云
    cloudPtr->resize(cloudPtr->width*cloudPtr->height);
    for(size_t i = 0; i < cloudPtr->points.size(); i++)
    {
        cloudPtr->points[i].x = 1024.0f*rand()/(RAND_MAX+1.0f);
        cloudPtr->points[i].y = 1024.0f*rand()/(RAND_MAX+1.0f);
        cloudPtr->points[i].z = 1024.0f*rand()/(RAND_MAX+1.0f);
    }

    if(strcmp(argv[1], "-r") == 0) //选择半径离群点移除
    {
        pcl::RadiusOutlierRemoval<xyz> outrem;
        //输入点云
        outrem.setInputCloud(cloudPtr);
        //设置滤波参数
        outrem.setRadiusSearch(0.4);
        outrem.setMinNeighborsInRadius(2);
        //执行滤波
        outrem.filter(*cloudPtr_filtered);

    }
    else if (strcmp(argv[1], "-c") == 0) //选择条件滤波
    {
        //建立条件build the condition
        pcl::ConditionAnd<xyz>::Ptr range_cond(new pcl::ConditionAnd<xyz>);
        range_cond->addComparison(pcl::FieldComparison<xyz>::ConstPtr(new pcl::FieldComparison<xyz>("z", pcl::ComparisonOps::GT, 0.0)));
        range_cond->addComparison(pcl::FieldComparison<xyz>::ConstPtr(new pcl::FieldComparison<xyz>("z", pcl::ComparisonOps::LT, 0.0)));
        
        pcl::ConditionalRemoval<xyz> condrem;
        condrem.setCondition(range_cond);
        condrem.setInputCloud(cloudPtr);
        //设置条件滤波参数
        condrem.setKeepOrganized(true);
        condrem.filter(*cloudPtr_filtered);  
    }
    else 
    {
        cerr << "Please specify command line argument '-r' or '-c'" << endl;
        exit(0);
    }

    //输出滤波前后的cloud data
    cout << "Point Cloud before filtering" << endl;
    for (size_t i = 0; i < cloudPtr->size(); i++)
    {
        cout << " " << cloudPtr->points[i].x
             << " " << cloudPtr->points[i].y
             << " " << cloudPtr->points[i].z << endl;
    }
    cout << "Point Cloud after filtering" << endl;
    for (size_t i = 0; i < cloudPtr->size(); i++)
    {
        cout << " " << cloudPtr_filtered->points[i].x
             << " " << cloudPtr_filtered->points[i].y
             << " " << cloudPtr_filtered->points[i].z << endl;
    }
    //可视化
    showPointCloud(cloudPtr, cloudPtr_filtered);

    return 0;
}




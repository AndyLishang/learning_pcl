#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>


void showHelp(char *program_name)
{
    std::cout << std::endl;
    std::cout << "Usages: " << program_name << " cloud_filename.(PCD!)" << std::endl;
    std::cout << "-h: Show this help." << std::endl;
}


int main(int argc, char** argv)
{
    if (pcl::console::find_switch(argc, argv, "-h") || pcl::console::find_switch(argc, argv, "--help"))
    {
        showHelp(argv[0]);
        return 0;
    }

    std::vector<int> filenames;
    filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");

    if (filenames.size()!=1) return 0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云数据
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZ>(argv[filenames[0]], *cloud);
    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // 创建滤波器对象
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);       // 设置待滤波的点云
    sor.setMeanK(50);               // 设置在进行统计时考虑查询点邻居点数
    sor.setStddevMulThresh(1.0);    // 设置判断是否为离群点的阈值
    sor.filter(*cloud_filtered);    // 将滤波结果保存在cloud_filtered中

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("table_scene_lms400_inliers.pcd", *cloud_filtered, false);

    // 可视化
    pcl::visualization::PCLVisualizer viewer("statistic removal");
    int v1(1);
    int v2(2);
    viewer.createViewPort(0, 0, 0.5, 1, v1);
    viewer.createViewPort(0.5, 0, 1, 1, v2);
    viewer.addPointCloud(cloud, "cloud", v1);
    viewer.addPointCloud(cloud_filtered, "cloud_filtered", v2);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}
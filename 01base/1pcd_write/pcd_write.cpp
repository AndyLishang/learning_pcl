#include <iostream>
#include <pcl/io/pcd_io.h>      // pcd文件IO相关
#include <pcl/point_cloud.h>    // pcl中的类定义头文件
#include <pcl/point_types.h>    // pcl中的点类型头文件

int main(int argc, char **argv)
{
    // pcl的版本
    std::cout << "PCL version: " << PCL_VERSION_PRETTY << std::endl;
    
    // PCL中用于表示点云数据的模板类 <PCL 中定义的一种点类型>
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ p;

    // 填充点云数据信息
    cloud.width = 5;            // 点云每行5个点
    cloud.height = 2;           // 点云有2行
    cloud.is_dense = false;     // 非dense，即允许存在 NaN/Inf 等无效点
    cloud.points.resize(cloud.width * cloud.height);    // 申请10个 PointXYZ 的存储空间

    for (auto &point : cloud)       // 遍历每个点并填充随机坐标
    {
        point.x = 1024 * rand() / (RAND_MAX + 1.0f);    
        point.y = 1024 * rand() / (RAND_MAX + 1.0f);
        point.z = 1024 * rand() / (RAND_MAX + 1.0f);
        // std::cout << point.x << "  " << rand() << "  " << RAND_MAX << std::endl;
    }


    pcl::io::savePCDFileASCII("test_pcd.pcd", cloud);       // 保存为点云PCD文件
    std::cerr << "Saved " << cloud.size() << " data points to test_pcd.pcd." << std::endl;

    for (const auto &point : cloud)     // 打印保存的点云信息
        std::cerr << "    " << point.x << " " << point.y << " " << point.z << std::endl;

    return (0);
}
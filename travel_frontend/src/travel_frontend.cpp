#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_msgs/GridMap.h>

ros::Publisher pointCloudPub;

void gridMapCallback(const grid_map_msgs::GridMap& gridMapMsg) {
    grid_map::GridMap gridMap;
    grid_map::GridMapRosConverter::fromMessage(gridMapMsg, gridMap);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Iterate over the grid map and extract elevation points
    for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position position;
        gridMap.getPosition(*iterator, position);

        float elevation = gridMap.at("elevation", *iterator);  // grid map 에서 layer 선택
        pcl::PointXYZ point;
        point.x = position.x();
        point.y = position.y();
        point.z = elevation; // sub grid 안에 존재하는 값
        cloud->points.push_back(point);
    }
    
    sensor_msgs::PointCloud2 pointCloudMsg;
    pcl::toROSMsg(*cloud, pointCloudMsg);
    pointCloudMsg.header.frame_id = gridMapMsg.info.header.frame_id;
    pointCloudMsg.header.stamp = gridMapMsg.info.header.stamp;

    pointCloudPub.publish(pointCloudMsg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_to_point_cloud");
    ros::NodeHandle nh;

    ros::Subscriber gridMapSub = nh.subscribe("/elevation_mapping/elevation_map", 1, gridMapCallback);

    pointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("elevation_point_cloud", 1);

    ros::spin();
    return 0;
}

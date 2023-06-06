/* 3D Object Clustering and Segmentation

Reference: 
1. https://github.com/jupidity/PCL-ROS-cluster-Segmentation/blob/master/src/segmentation.cpp

Authors:
Nalin Das (nalindas9@gmail.com)
Graduate Student pursuing M.Eng. in Robotics,
University of Maryland, College Park
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <obj_recognition/SegmentedClustersArray.h>

ros::Publisher pub;

// Function prototypes
void pclCallback(const sensor_msgs::PointCloud2ConstPtr&);

// Main function
int main(int argc, char** argv){
    //Initialize ros node
    ros::init(argc, argv, "sawyer_segmentation");
    ros::NodeHandle nh;
    // Publish voxelized cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("/obj_recognition/voxelized_cloud", 1);
    // Subscribe to the point cloud topic
    ros::Subscriber sub = nh.subscribe("/obj_recognition/point_cloud", 1, pclCallback);
    
    ROS_INFO_STREAM("Segmentation started ...");
    
    
    ros::spin();
    return 0;
}

// Point cloud subscriber callback
void pclCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    // PCL Container for input and filtered output data
    pcl::PCLPointCloud2* cloud_input = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudptr(cloud_input);
    pcl::PCLPointCloud2 filtered_cloud;
    //pcl::PCLPointCloud2Ptr cloudptr_flitered(filtered_cloud);
    
    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud_input);
    
    //Performing Voxel Grid Downsampling Filtering to reduce points in cloud
    pcl::VoxelGrid<pcl::PCLPointCloud2> input_voxel_grid;
    input_voxel_grid.setInputCloud(cloudptr);
    input_voxel_grid.setLeafSize(0.01, 0.01, 0.01); // Set voxel size 0.01x0.01x0.01
    input_voxel_grid.filter(filtered_cloud);
    
    // Create pcl object to hold pcl point xyzrgb of voxel filtered cloud
    pcl::PointCloud<pcl::PointXYZRGB> *xyz_cloud = new pcl::PointCloud<pcl::PointXYZRGB>;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtr(xyz_cloud);
    
    //Convert from PCL PointCloud2 format to PCL PointCloud xyzrgb format
    pcl::fromPCLPointCloud2(filtered_cloud, *xyzCloudPtr);
    
    // Create pcl object to hold Passthrough filtered results
    pcl::PointCloud<pcl::PointXYZRGB> xyz_cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ> *xyz_cloud_filtered_kdtree = new pcl::PointCloud<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud_Ptr_filtered_kdtree (xyz_cloud_filtered_kdtree);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzCloudPtrFiltered(xyz_cloud_filtered);
    
    // Using Passthrough filter
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(xyzCloudPtr);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.8, 1.1);
    pass.filter(xyz_cloud_filtered);
    
    pcl::copyPointCloud(xyz_cloud_filtered, *xyz_cloud_filtered_kdtree);
    
    // PCL object to hold kdtree clustered PCL 2 object
    pcl::PCLPointCloud2 outputPCL;
    
    // Convert to PCL Point Cloud 2
    //pcl::toPCLPointCloud2(xyz_cloud_filtered, outputPCL);
    
    //Convert back to ROS PCL format for publishing
    sensor_msgs::PointCloud2 output_cloud;
    //pcl_conversions::fromPCL(outputPCL, output_cloud);

    // Euclidean cluster segmentation to seperate individual objects
    // Create KdTree object for extraction algorithm
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (xyz_cloud_Ptr_filtered_kdtree);
    
    // Container for extracted cluster
    std::vector<pcl::PointIndices> cluster_indices;
    // Euclidean Extraction object for the clusters
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> euclus;
    // Define Euclidean Cluster Parameters
    euclus.setClusterTolerance(0.02);
    euclus.setMinClusterSize (100);
    euclus.setMaxClusterSize (25000);
    euclus.setSearchMethod (tree);
    euclus.setInputCloud (xyz_cloud_Ptr_filtered_kdtree);
    // Store extracted indices of each cluster in a vector
    euclus.extract(cluster_indices);
    
    ROS_INFO_STREAM("Cluster indices size: " << cluster_indices.end()-cluster_indices.begin());
    // Container to store segmented cluster
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int>::const_iterator pit; 
    std::vector<pcl::PointIndices>::const_iterator it;
    it = cluster_indices.begin()+1;
    for(pit = it->indices.begin(); pit != it->indices.end(); pit++){
        // Add each point in cluster indices to cluster container
        cloud_cluster->points.push_back(xyz_cloud_Ptr_filtered_kdtree->points[*pit]);
    }
    
    //xyz_cloud_Ptr_filtered_kdtree->swap(*cloud_cluster);
    
    //cloud_cluster->width = cloud_cluster->points.size();
    //cloud_cluster->height = 1;
    //cloud_cluster->is_dense = true;
    std::cout << "Cluster has " << cloud_cluster->points.size() << " points.\n";
    
    
    
    // convert to pcl::PCLPointCloud2
    pcl::toPCLPointCloud2( *cloud_cluster ,outputPCL);
    
    // Set converted point cloud header frame as /world to transform it to this frame
    outputPCL.header.frame_id = "/world";
    //outputPCL.header.stamp = ros::Time::now();
    
    // Convert to ROS data type
    pcl_conversions::fromPCL(outputPCL, output_cloud);
    
     
    //output_cloud.header.frame_id = "/camera_depth_frame";
    //output_cloud.header.stamp = ros::Time::now();
    pub.publish(output_cloud);
    
    // Deallocate memory
    //delete cloud_input;
    //delete filtered_cloud;
    
    ROS_INFO_STREAM("Filtered point cloud!");
}

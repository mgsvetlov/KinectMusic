//
//  downsample.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 08/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "pcldownsample.hpp"
#include "pclutility.hpp"

void PclDownsample::downsample(Blob& blob, Blob& blob_filtered){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    PclUtility::blob2cloud(blob, cloud);
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2);
    toPCLPointCloud2 (*cloud, *cloud2);
    pcl::PCLPointCloud2::Ptr cloud2_filtered(new pcl::PCLPointCloud2);
    
    downsample(cloud2, cloud2_filtered);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    fromPCLPointCloud2 (*cloud2_filtered, *cloud_filtered);
    blob_filtered.setMatSize(blob.getMatSize());
    blob_filtered.setCentralCell(blob.getCentralCell());
    PclUtility::cloud2blob(cloud_filtered, blob_filtered);
    
}

void PclDownsample::downsample (pcl::PCLPointCloud2::Ptr cloud, pcl::PCLPointCloud2::Ptr cloud_filtered)
{
    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (10.f, 10.f, 10.f);
    sor.filter (*cloud_filtered);

}
#ifndef EXTRACTFEATURES_H
#define EXTRACTFEATURES_H

// include common header files and data type configuration
#include "commonFunc.h"
#include <iostream>
#include <fstream>
class extractFeatures
{
public:
    extractFeatures();

    // estimate surface normals
    NormalsT::Ptr estimateSurfaceNormals(const PointCloudT::Ptr &cloud, const f32 param[]);

    // func to detect Harris key point
    PointCloudT::Ptr keyPtsHarris3d(const PointCloudT::Ptr &cloud,
                                    NormalsT::Ptr normals, const f32 param[]);

    // func to detect Intrinsic Shape Signature key point
    PointCloudT::Ptr keyPtsIss3d(const PointCloudT::Ptr &cloud, const f32 param[]);

    // func to detect the SIFT key point in 3d
    PointCloudT::Ptr keyPtsSIFT (const PointCloudT::Ptr &cloud, const f32 param[]);

    // render range image
    pcl::RangeImage renderRangeImage(const PointCloudT::Ptr &cloud, const f32 param[]);

    // func to detect the Narf key points
    PointCloudT::Ptr keyPtsNARF(pcl::RangeImage &rangeImage, const f32 param[]);

    // func to remove unstable key points
    PointCloudT::Ptr keyPtsFilter(const PointCloudT::Ptr &cloud,
                                  const PointCloudT::Ptr &keyPts, const f32 param[]);

    // func to remove too close keypoints using voxel filtering
    PointCloudT::Ptr voxelFilter(const PointCloudT::Ptr& keyPts, const f32 param[]);

    // func to combine filtered key points
    PointCloudT::Ptr removeNon_exist(const PointCloudT::Ptr& refCloud,
                                     const PointCloudT::Ptr& inputCloud);

    // func to calculate the RIFT feature descriptor
    pcl::PointCloud<RIFT32>::Ptr RIFTcolorDescriptor(const PointCloudT::Ptr& cloudColor,
                                                     const PointCloudT::Ptr& keyPts,
                                                     const float params[]);
    // func to calculate the SHOT352 feature descriptor
    pcl::PointCloud<SHOT352>::Ptr Shot352Descriptor(const PointCloudT::Ptr& cloudColor,
                                                    const PointCloudT::Ptr& keyPts,
                                                    const float params[]);
    // func to calculate the SHOT1344 feature descriptor
    pcl::PointCloud<SHOT1344>::Ptr Shot1344Descriptor(const PointCloudT::Ptr& cloudColor,
                                                    const PointCloudT::Ptr& keyPts,
                                                    const float params[]);

    // overload func to match the descriptors using knn search
    uc8 matchKeyPts(const pcl::PointCloud<RIFT32>::Ptr & desc_1,
                    const pcl::PointCloud<RIFT32>::Ptr & desc_2,
                    std::vector<u16> *idx, std::vector<f32> *dist);
    uc8 matchKeyPts(const pcl::PointCloud<SHOT352>::Ptr & desc_1,
                    const pcl::PointCloud<SHOT352>::Ptr & desc_2,
                    std::vector<u16> *idx, std::vector<f32> *dist);
    uc8 matchKeyPts(const pcl::PointCloud<SHOT1344>::Ptr & desc_1,
                    const pcl::PointCloud<SHOT1344>::Ptr & desc_2,
                    std::vector<u16> *idx, std::vector<f32> *dist);

    // func to remove outliers matches using RANSAC
    std::vector<s16> matchRansacPCL(PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud2,
                                    std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                                    f32 errThd, u16 maxIter);

    std::vector<s16> matchRansac(PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud2,
                                    std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                                    f32 errThd, u16 maxIter);


    // func to clip far distance point clouds
    void distClip(PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud2,float Thd);
    void distClip(PointCloudT::Ptr &cloud, float Thd);

    // func to extract the edges points
    void extractEdge(PointCloudT::Ptr &cloud, PointCloudT::Ptr & edges);

    // overload func to match descriptors on both side
    uc8 crossMatching(const pcl::PointCloud<RIFT32>::Ptr & desc_1,
                      const pcl::PointCloud<RIFT32>::Ptr & desc_2,
                      std::vector<u16> *idx_1, std::vector<u16> *idx_2);

    uc8 crossMatching(const pcl::PointCloud<SHOT352>::Ptr & desc_1,
                      const pcl::PointCloud<SHOT352>::Ptr & desc_2,
                      std::vector<u16> *idx_1, std::vector<u16> *idx_2);

    uc8 crossMatching(const pcl::PointCloud<SHOT1344>::Ptr & desc_1,
                      const pcl::PointCloud<SHOT1344>::Ptr & desc_2,
                      std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                      std::vector<f32> *match_dist);


};

#endif // EXTRACTFEATURES_H

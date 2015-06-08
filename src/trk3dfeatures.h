#ifndef TRK3DFEATURES_H
#define TRK3DFEATURES_H
#include "commonFunc.h"
#include "commonHeader.h"
#include "extractFeatures.h"

class trk3dFeatures
{
public:
    trk3dFeatures();
    void trkInit(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr &cloud2,
                 const PointCloudT::Ptr &keyPts, const PointCloudT::Ptr &keyPts2,
                 const float params[],
                 pcl::PointCloud<SHOT1344>::Ptr &Shot1344Cloud_1,
                 std::vector<u16> &matchIdx1, std::vector<u16> &matchIdx2,
                 std::vector<f32> &matchDist);

    void trkSeq(const PointCloudT::Ptr &cloud2,
                const PointCloudT::Ptr &keyPts2,
                const float params[],
                PointCloudT::Ptr &keyPts,
                pcl::PointCloud<SHOT1344>::Ptr &Shot1344Cloud_1,
                std::vector<u16> &matchIdx1, std::vector<u16> &matchIdx2,
                std::vector<f32> &matchDist);
};

#endif // TRK3DFEATURES_H

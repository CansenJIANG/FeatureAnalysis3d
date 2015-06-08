#include "trk3dfeatures.h"

trk3dFeatures::trk3dFeatures()
{
}


void trk3dFeatures::trkInit(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr &cloud2,
                            const PointCloudT::Ptr &keyPts, const PointCloudT::Ptr &keyPts2,
                            const float params[],
                            pcl::PointCloud<SHOT1344>::Ptr &Shot1344Cloud_1,
                            std::vector<u16> &matchIdx1, std::vector<u16> &matchIdx2,
                            std::vector<f32> &matchDist)
{
    // construct descriptors
    pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_2;

    extractFeatures featureDetector;
    Shot1344Cloud_1 = featureDetector.Shot1344Descriptor(cloud, keyPts, params);

    Shot1344Cloud_2 = featureDetector.Shot1344Descriptor(cloud2, keyPts2, params);

    // match keypoints
    featureDetector.crossMatching(Shot1344Cloud_1, Shot1344Cloud_2,
                                  &matchIdx1, &matchIdx2, &matchDist);
    std::cout<<"trkInit done!\n";
    pcl::copyPointCloud(*Shot1344Cloud_2, *Shot1344Cloud_1);
}


void trk3dFeatures::trkSeq(const PointCloudT::Ptr &cloud2,
                           const PointCloudT::Ptr &keyPts2,
                           const float params[],
                           PointCloudT::Ptr &keyPts,
                           pcl::PointCloud<SHOT1344>::Ptr &Shot1344Cloud_1,
                           std::vector<u16> &matchIdx1, std::vector<u16> &matchIdx2,
                           std::vector<f32> &matchDist)
{
    // construct descriptors
    pcl::PointCloud<SHOT1344>::Ptr Shot1344Cloud_2(new pcl::PointCloud<SHOT1344>);

    extractFeatures featureDetector;
    Shot1344Cloud_2 = featureDetector.Shot1344Descriptor(cloud2, keyPts2, params);

//    // match keypoints
//    featureDetector.crossMatching(Shot1344Cloud_1, Shot1344Cloud_2,
//                                  &matchIdx1, &matchIdx2, &matchDist);


    // find the matching from desc_1 -> desc_2
    std::cout<<"size of Shot1344Cloud_1 in trkSeq: "<<Shot1344Cloud_1->points.size()<<std::endl;
    std::cout<<"size of Shot1344Cloud_2 in trkSeq: "<<Shot1344Cloud_2->points.size()<<std::endl;
    featureDetector.matchKeyPts(Shot1344Cloud_1, Shot1344Cloud_2, &matchIdx2, &matchDist);
    std::cout<<"size of matches in trkSeq: "<<matchIdx2.size()<<std::endl;
    std::cout<<"size of matchDist in trkSeq: "<<matchDist.size()<<std::endl;
    Shot1344Cloud_1.reset(new pcl::PointCloud<SHOT1344>);
    keyPts->points.clear();
    for(size_t i=0; i<matchIdx2.size();i++)
    {
        keyPts->push_back(keyPts2->points.at(matchIdx2[i]));
        Shot1344Cloud_1->push_back(Shot1344Cloud_2->points.at(matchIdx2[i]));
    }
}

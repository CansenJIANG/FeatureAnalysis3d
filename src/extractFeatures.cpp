#include "extractFeatures.h"

extractFeatures::extractFeatures()
{
}

/////////////////////////////////////////////////////////////////////////////////////
/// Intrinsic Shape Signature Keypoint Detector
/////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr
extractFeatures::keyPtsIss3d(const PointCloudT::Ptr &cloud, const f32 param[])
{
    //  ISS3D parameters
    f64 iss_salient_radius_ = (f64) param[0]; // radius of the spherical neighborhood used to
    // compute the scatter matrix
    f64 iss_non_max_radius_ = (f64) 1.5*param[0]; // parameter for non maxima supression algorithm
    f64 iss_gamma_21_       = (f64) param[2]; // upper bound ratio between 2nd and 1st eigen values
    f64 iss_gamma_32_       = (f64) param[3]; // upper bound ratio between 3rd and 2nd eigen values
    s16 iss_min_neighbors_  = (s16) param[4]; // minimum required neighbors to compute the descriptor
    u16 iss_threads_        = (u16) param[5]; // number of threads

    // construct kdtree for fast data accessing
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    pcl::ISSKeypoint3D<PointT, PointT> iss_detector;
    PointCloudT::Ptr keypts(new PointCloudT);

    iss_detector.setSearchMethod (tree);
    iss_detector.setSalientRadius (iss_salient_radius_);
    iss_detector.setNonMaxRadius (iss_non_max_radius_);
    iss_detector.setThreshold21 (iss_gamma_21_);
    iss_detector.setThreshold32 (iss_gamma_32_);
    iss_detector.setMinNeighbors (iss_min_neighbors_);
    iss_detector.setNumberOfThreads (iss_threads_);
    iss_detector.setInputCloud (cloud);
    iss_detector.compute (*keypts);

    return keypts;
}

///////////////////////////////////////////////////////////////////////////////////////
///// Harris 3d Key Point Detector
///////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr
extractFeatures::keyPtsHarris3d(const PointCloudT::Ptr &cloud, NormalsT::Ptr normals,
                                const f32 param[])
{
    // construct kdtree for fast data accessing
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    //    pcl::HarrisKeypoint3D<PointT, PointT, NormalT> Harris3d_detector;
    PointCloudT::Ptr keypts (new PointCloudT);

    //    Harris3d_detector.setNonMaxSupression(true);
    //    Harris3d_detector.setInputCloud(cloud);
    //    Harris3d_detector.setSearchMethod(tree);
    //    Harris3d_detector.setNormals(normals);
    //    Harris3d_detector.setRadius(0.01f);
    //    Harris3d_detector.setThreshold(0.01f);
    //    Harris3d_detector.setNumberOfThreads(8);
    //    Harris3d_detector.compute(*keypts);

    return keypts;
}

///////////////////////////////////////////////////////////////////////////////////////
///// SIFT Keypoint Detector
///////////////////////////////////////////////////////////////////////////////////////
// min_scale: the standard deviation of the smallest scale in the scale space
// nr_octaves: the number of octaves (i.e. doublings of stdev) to compute
// nr_scales_per_octave: the number of scales to compute within each octave
// min_contrast:	the minimum contrast required for detection
PointCloudT::Ptr
extractFeatures::keyPtsSIFT (const PointCloudT::Ptr & cloud, const f32 param[])
{

    f32 min_scale = param[0];
    uc8 nr_octaves = (uc8) param[1];
    uc8 nr_scales_per_octave = (uc8) param[2];
    f32 min_contrast = param[3];

    pcl::SIFTKeypoint<PointT, pcl::PointWithScale> sift_detector;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    sift_detector.setSearchMethod(tree);
    sift_detector.setScales (min_scale, nr_octaves, nr_scales_per_octave);
    sift_detector.setMinimumContrast (min_contrast);
    sift_detector.setInputCloud (cloud);
    pcl::PointCloud<pcl::PointWithScale> keypoints_temp;
    sift_detector.compute (keypoints_temp);
    PointCloudT::Ptr keypts (new PointCloudT);
    pcl::copyPointCloud (keypoints_temp, *keypts);
    return (keypts);
}

///////////////////////////////////////////////////////////////////////////////////////
///// Surface Normals Estimation
///////////////////////////////////////////////////////////////////////////////////////
NormalsT::Ptr
extractFeatures::estimateSurfaceNormals (const PointCloudT::Ptr & cloud,
                                         const f32 param[])
{
    f32 radius = param[0];
    pcl::NormalEstimation<PointT, NormalT> normal_estimation;
    pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
    normal_estimation.setSearchMethod(tree);
    normal_estimation.setRadiusSearch (radius);
    normal_estimation.setInputCloud (cloud);
    NormalsT::Ptr normals (new NormalsT);
    normal_estimation.compute (*normals);
    return (normals);
}
//computeLocalDescriptors (const PointCloud::Ptr & points, const SurfaceNormalsPtr & normals,
//float feature_radius)
//{
//pcl::FPFHEstimation<PointT, NormalT, LocalDescriptorT> fpfh_estimation;
//fpfh_estimation.setSearchMethod
//(pcl::KdTreeFLANN<PointT>::Ptr (new pcl::KdTreeFLANN<PointT>));
//fpfh_estimation.setRadiusSearch (feature_radius);
//fpfh_estimation.setInputCloud (points);
//fpfh_estimation.setInputNormals (normals);
//LocalDescriptorsPtr local_descriptors (new LocalDescriptors);
//fpfh_estimation.compute (*local_descriptors);
//return (local_descriptors);
//}

///////////////////////////////////////////////////////////////////////////////////////
///// Create Range Image
///////////////////////////////////////////////////////////////////////////////////////
pcl::RangeImage
extractFeatures::renderRangeImage(const PointCloudT::Ptr & cloud, const f32 param[])
{
    PointCloudT pCloud;
    pcl::copyPointCloud(*cloud,pCloud);
    pCloud.width = (u32) pCloud.points.size();
    pCloud.height = 1;

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    f32 angularResolution = (f32) (  1.0f * (M_PI/180.0f));  //   1.0 degree in radians
    f32 maxAngleWidth     = (f32) (360.0f * (M_PI/180.0f));  // 360.0 degree in radians
    f32 maxAngleHeight    = (f32) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    f32 noiseLevel=0.00;
    f32 minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    return rangeImage;
}

///////////////////////////////////////////////////////////////////////////////////////
///// Normal Aligned Radial Features
///////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr
extractFeatures::keyPtsNARF(pcl::RangeImage &rangeImage, const f32 param[])
{
    // construct NARF keypoint detector
    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector (&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&rangeImage);
    narf_keypoint_detector.getParameters().support_size = param[0];

    // compute NARF keypoints
    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);

    // get NARF keypoints from their indices
    PointCloudT::Ptr keypts(new PointCloudT);
    keypts->points.resize (keypoint_indices.points.size ());
    for (u16 i=0; i<keypoint_indices.points.size (); ++i)
    {
        keypts->points[i].getVector3fMap () =
                rangeImage.points[keypoint_indices.points[i]].getVector3fMap ();
    }
    return keypts;
}

///////////////////////////////////////////////////////////////////////////////////////
///// Key point Filtering
///////////////////////////////////////////////////////////////////////////////////////
// 1. remove key points located near the edges using knn search
// 2. remove close neighbors using voxel filtering
PointCloudT::Ptr
extractFeatures::keyPtsFilter(const PointCloudT::Ptr &cloud,
                              const PointCloudT::Ptr &keyPts, const f32 param[])
{
    // spatial search radius
    f64 searchRadius     = param[6];
    // pt density threshold
    f32 densityThreshold = param[7];

    // filtered key points
    PointCloudT::Ptr keyPtsFiltered (new PointCloudT);

    // Neighbors within radius search
    std::vector<s16> ptIdxRadiusSearch;
    std::vector<f32> ptRadiusSquaredDist;

    // consturct kdtree
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (cloud);

    // Neighbors within radius search
    for(u16 i = 0; i< keyPts->points.size(); ++i)
    {
        PointT searchPt = keyPts->points[i];
        u16 neighSize = kdtree.radiusSearch (searchPt, searchRadius,
                                             ptIdxRadiusSearch, ptRadiusSquaredDist);
        // Keep high density keypoints
        // scale the density estimation using the ditance of searching point
        float distScale = commonFunc::l2norm(searchPt.x, searchPt.y, searchPt.z) ;
        if ( (neighSize*distScale) > densityThreshold)
        {
            keyPtsFiltered->push_back(searchPt);
        }
    }

    return keyPtsFiltered;
}


///////////////////////////////////////////////////////////////////////////////////////
///// func to remove too close keypoints using voxel filtering
///////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr
extractFeatures::voxelFilter(const PointCloudT::Ptr& keyPts, const f32 param[])
{
    pcl::PCLPointCloud2::Ptr keyPts_filtered2 (new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr keyPts2(new pcl::PCLPointCloud2);

    // convert pointcloud to pointcloud2
    pcl::toPCLPointCloud2(*keyPts, *keyPts2);

    // Create the filtering object
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxGrid;
    voxGrid.setInputCloud (keyPts2);
    voxGrid.setLeafSize (param[8], param[9], param[10]);
    voxGrid.filter (*keyPts_filtered2);

    // convert pointcloud2 to pointcloud
    PointCloudT::Ptr keyPts_filtered(new PointCloudT);
    pcl::fromPCLPointCloud2(*keyPts_filtered2, *keyPts_filtered);

    return keyPts_filtered;
}


///////////////////////////////////////////////////////////////////////////////////////
///// func to remove points not exist from original point cloud
///////////////////////////////////////////////////////////////////////////////////////
PointCloudT::Ptr
extractFeatures::removeNon_exist(const PointCloudT::Ptr& refCloud,
                                 const PointCloudT::Ptr& inputCloud)
{
    // consturct kdtree
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud (refCloud);

    // K nearest neighbor search
    PointCloudT::Ptr existPts(new PointCloudT);

    // Neighbors within radius search
    for(u16 i = 0; i< inputCloud->points.size(); ++i)
    {
        s16 K = 1;
        std::vector<s16> ptIdxNKNSearch(K);
        std::vector<f32> ptNKNSquaredDist(K);
        PointT searchPt = inputCloud->points[i];
        kdtree.nearestKSearch (searchPt, K, ptIdxNKNSearch, ptNKNSquaredDist);
        // density too low, mark to remove
        if ( ptNKNSquaredDist[0] == 0)
        {
            existPts->push_back(searchPt);
        }
    }
    return existPts;
}


///////////////////////////////////////////////////////////////////////////////////////
///// RIFT color feature descriptor
///////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<RIFT32>::Ptr
extractFeatures::RIFTcolorDescriptor(const PointCloudT::Ptr& cloudColor,
                                     const PointCloudT::Ptr& keyPts,
                                     const float params[])
{
    // Object for storing the point cloud with intensity value.
    PointCloudI::Ptr cloudIntensity(new PointCloudI);
    PointCloudI::Ptr keyPtsIntensity(new PointCloudI);
    // Object for storing the intensity gradients.
    pcl::PointCloud<pcl::IntensityGradient>::Ptr gradients
            (new pcl::PointCloud<pcl::IntensityGradient>);
    // Object for storing the normals.
    NormalsT::Ptr normals(new NormalsT);
    // Object for storing the RIFT descriptor for each point.
    pcl::PointCloud<RIFT32>::Ptr descriptors(new pcl::PointCloud<RIFT32>());

    // Note: you would usually perform downsampling now. It has been omitted here
    // for simplicity, but be aware that computation can take a long time.

    // Convert the RGB to intensity.
    pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);
    pcl::PointCloudXYZRGBtoXYZI(*keyPts, *keyPtsIntensity);

    // Estimate the normals.
    pcl::NormalEstimation<PointI, NormalT> normalEstimation;
    normalEstimation.setInputCloud(cloudIntensity);
    normalEstimation.setRadiusSearch(params[0]);
    pcl::search::KdTree<PointI>::Ptr kdtree(new pcl::search::KdTree<PointI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // Compute the intensity gradients.
    pcl::IntensityGradientEstimation < PointI, NormalT, pcl::IntensityGradient,
            pcl::common::IntensityFieldAccessor<PointI> > ge;
    ge.setInputCloud(cloudIntensity);
    ge.setInputNormals(normals);
    ge.setRadiusSearch(params[0]);
    ge.compute(*gradients);

    // RIFT estimation object.
    pcl::RIFTEstimation<PointI, pcl::IntensityGradient, RIFT32> rift;
    rift.setSearchSurface(cloudIntensity);
    rift.setInputCloud(keyPtsIntensity);
    rift.setSearchMethod(kdtree);
    // Set the intensity gradients to use.
    rift.setInputGradient(gradients);
    // Radius, to get all neighbors within.
    rift.setRadiusSearch(params[1]);
    // Set the number of bins to use in the distance dimension.
    rift.setNrDistanceBins(params[2]);
    // Set the number of bins to use in the gradient orientation dimension.
    rift.setNrGradientBins(params[3]);

    // Note: you must change the output histogram size to reflect the previous values.
    rift.compute(*descriptors);

    return descriptors;
}
///////////////////////////////////////////////////////////////////////////////////////
///// SHOT color feature descriptor
///////////////////////////////////////////////////////////////////////////////////////
pcl::PointCloud<SHOT352>::Ptr
extractFeatures::Shot352Descriptor(const PointCloudT::Ptr& cloudColor,
                                   const PointCloudT::Ptr& keyPts,
                                   const float params[])

{
    // Object for storing the point cloud with intensity value.
    PointCloudI::Ptr cloudIntensity(new PointCloudI);
    PointCloudI::Ptr keyPtsIntensity(new PointCloudI);
    // Object for storing the normals.
    NormalsT::Ptr normals(new NormalsT);
    // Convert the RGB to intensity.
    pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);
    pcl::PointCloudXYZRGBtoXYZI(*keyPts, *keyPtsIntensity);

    // Estimate the normals.
    pcl::NormalEstimation<PointI, NormalT> normalEstimation;
    normalEstimation.setInputCloud(cloudIntensity);
    normalEstimation.setRadiusSearch(params[0]);
    pcl::search::KdTree<PointI>::Ptr kdtree(new pcl::search::KdTree<PointI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // compute descriptors
    pcl::PointCloud<SHOT352>::Ptr descriptors(new pcl::PointCloud<SHOT352>());
    pcl::SHOTEstimationOMP<PointT, NormalT, SHOT352> descr_est;
    descr_est.setRadiusSearch (params[1]);
    descr_est.setInputCloud (keyPts);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (cloudColor);
    descr_est.compute (*descriptors);
    u16 i = descriptors->size();
    while(i--)
    {
        SHOT352 desc_i = descriptors->at(i);
        if( pcl_isfinite (desc_i.descriptor[0]) )
        {
        }else
        {
            descriptors->erase(descriptors->begin()+i);
            keyPts->erase(keyPts->begin()+i);
        }
    }
    return descriptors;
}
///////////////////////////////////////////////////////////////////////////////////////
///// SHOT1344 color feature descriptor
///////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<SHOT1344>::Ptr
extractFeatures::Shot1344Descriptor(const PointCloudT::Ptr& cloudColor,
                                   const PointCloudT::Ptr& keyPts,
                                   const float params[])

{
    // Object for storing the point cloud with intensity value.
    PointCloudI::Ptr cloudIntensity(new PointCloudI);
    PointCloudI::Ptr keyPtsIntensity(new PointCloudI);
    // Object for storing the normals.
    NormalsT::Ptr normals(new NormalsT);
    // Convert the RGB to intensity.
    pcl::PointCloudXYZRGBtoXYZI(*cloudColor, *cloudIntensity);
    pcl::PointCloudXYZRGBtoXYZI(*keyPts, *keyPtsIntensity);

    // Estimate the normals.
    pcl::NormalEstimation<PointI, NormalT> normalEstimation;
    normalEstimation.setInputCloud(cloudIntensity);
    normalEstimation.setRadiusSearch(params[0]);
    pcl::search::KdTree<PointI>::Ptr kdtree(new pcl::search::KdTree<PointI>);
    normalEstimation.setSearchMethod(kdtree);
    normalEstimation.compute(*normals);

    // compute descriptors
    pcl::PointCloud<SHOT1344>::Ptr descriptors(new pcl::PointCloud<SHOT1344>());
    pcl::SHOTColorEstimationOMP<PointT, NormalT> descr_est;
    descr_est.setRadiusSearch (params[1]);
    descr_est.setInputCloud (keyPts);
    descr_est.setInputNormals (normals);
    descr_est.setSearchSurface (cloudColor);
    descr_est.compute (*descriptors);
    u16 i = descriptors->size();
    f32 precision = 1.0/1344.0;
    while(--i)
    {
        SHOT1344 *desc_i = &descriptors->at(i);
        if( pcl_isfinite ( (*desc_i).descriptor[0]) )
        {
            for(size_t scl=0; scl<desc_i->descriptorSize();scl++)
            {
                if( (*desc_i).descriptor[scl] < precision)
                {
                    (*desc_i).descriptor[scl] = 0;
                }
            }
        }else
        {
            descriptors->erase(descriptors->begin()+i);
            keyPts->erase(keyPts->begin()+i);
        }
    }
    return descriptors;
}


///////////////////////////////////////////////////////////////////////////////////////
///// Match feature descriptors using knn search
///////////////////////////////////////////////////////////////////////////////////////
// find the nearest descriptors
uc8
extractFeatures::matchKeyPts(const pcl::PointCloud<RIFT32>::Ptr & desc_1,
                             const pcl::PointCloud<RIFT32>::Ptr & desc_2,
                             std::vector<u16> *idx, std::vector<f32> *dist)
{
    if(desc_1->points.size()<1 || desc_2->points.size()<1)
    {
        return 0;
    }
    pcl::KdTreeFLANN<RIFT32> matchSearch;
    std::vector<s16> neighIndices (1);
    std::vector<f32> neighDists (1);
    idx->clear();
    dist->clear();

    // define the reference
    matchSearch.setInputCloud(desc_2);
    s16 i = 0;
    bool neighFound = false;
    while( i < desc_1->points.size())
    {
        RIFT32 descSearch = desc_1->at(i);
        neighIndices.clear();
        neighDists.clear();
        // find the close neighbors
        neighFound = matchSearch.nearestKSearch(descSearch, 1,
                                                neighIndices, neighDists);
        if(neighFound)
        {
            idx ->push_back( (u16) neighIndices[0] );
            dist->push_back( (f32) neighDists[0] );
        }
        ++i;
    }

    // matches found
    if( idx->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uc8
extractFeatures::matchKeyPts(const pcl::PointCloud<SHOT352>::Ptr & desc_1,
                             const pcl::PointCloud<SHOT352>::Ptr & desc_2,
                             std::vector<u16> *idx, std::vector<f32> *dist)
{
    if(desc_1->points.size()<1 || desc_2->points.size()<1)
    {
        return 0;
    }
    pcl::KdTreeFLANN<SHOT352> matchSearch;
    std::vector<s16> neighIndices (1);
    std::vector<f32> neighDists (1);
    idx->clear();
    dist->clear();

    // define the reference
    matchSearch.setInputCloud(desc_2);
    s16 i = 0;
    bool neighFound = false;
    std::ofstream distFile;
    distFile.open ("distFile.txt");
    while( i < desc_1->points.size())
    {
        SHOT352 descInquiry = desc_1->at(i);
        neighIndices.clear();
        neighDists.clear();
        // find the close neighbors
        if(pcl_isfinite (descInquiry.descriptor[0]))
        {
            neighFound = matchSearch.nearestKSearch(descInquiry, 1,
                                                    neighIndices, neighDists);
            if(neighFound)
            {
                idx ->push_back( (u16) neighIndices[0] );
                dist->push_back( (f32) neighDists[0] );
                distFile << neighDists[0] <<"\n";
            }else
            {
                idx ->push_back( desc_2->points.size() );
                dist->push_back( 100 );
                distFile << 100 <<"\n";
            }
        }
        ++i;
    }
    distFile.close();
    // matches found
    if( idx->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

uc8
extractFeatures::matchKeyPts(const pcl::PointCloud<SHOT1344>::Ptr & desc_1,
                             const pcl::PointCloud<SHOT1344>::Ptr & desc_2,
                             std::vector<u16> *idx, std::vector<f32> *dist)
{
    if(desc_1->points.size()<1 || desc_2->points.size()<1)
    {
        return 0;
    }
    pcl::KdTreeFLANN<SHOT1344> matchSearch;
    std::vector<s16> neighIndices (1);
    std::vector<f32> neighDists (1);
    idx->clear();
    dist->clear();

    // define the reference
    matchSearch.setInputCloud(desc_2);
    s16 i = 0;
    bool neighFound = false;
    std::ofstream distFile;
    distFile.open ("distFile.txt");
    while( i < desc_1->points.size())
    {
        SHOT1344 descInquiry = desc_1->at(i);
        neighIndices.clear();
        neighDists.clear();
        // find the close neighbors
        if(pcl_isfinite (descInquiry.descriptor[0]))
        {
            neighFound = matchSearch.nearestKSearch(descInquiry, 1,
                                                    neighIndices, neighDists);
            if(neighFound)
            {
                idx ->push_back( (u16) neighIndices[0] );
                dist->push_back( (f32) neighDists[0] );
                distFile << neighDists[0] <<"\n";
            }
        }
        ++i;
    }
    distFile.close();
    // matches found
    if( idx->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
///////////////////////////////////////////////////////////////////////////////////////
///// Match feature descriptors using knn search
///////////////////////////////////////////////////////////////////////////////////////
//template <typename descriptorT>
uc8
extractFeatures::crossMatching(const pcl::PointCloud<RIFT32>::Ptr & desc_1,
                               const pcl::PointCloud<RIFT32>::Ptr & desc_2,
                               std::vector<u16> *idx_1, std::vector<u16> *idx_2)
{
    std::vector<u16> idx_1to2;
    std::vector<u16> idx_2to1;
    std::vector<f32> dist_1to2;
    std::vector<f32> dist_2to1;

    // find the matching from desc_1 -> desc_2
    matchKeyPts(desc_1, desc_2, &idx_1to2, &dist_1to2);
    // find the matching from desc_2 -> desc_1
    matchKeyPts(desc_2, desc_1, &idx_2to1, &dist_2to1);

    {
        std::ofstream knnIdx1to2_RIFT, knnIdx2to1_RIFT;
        knnIdx1to2_RIFT.open("knnIdx1to2_RIFT.txt");
        for(int knn_i1 = 0; knn_i1<idx_1to2.size();knn_i1++)
        {
            knnIdx1to2_RIFT << idx_1to2.at(knn_i1)<<"\n";
        }
        knnIdx1to2_RIFT.close();

        knnIdx2to1_RIFT.open("knnIdx2to1_RIFT.txt");
        for(int knn_i2 = 0; knn_i2<idx_2to1.size();knn_i2++)
        {
            knnIdx2to1_RIFT << idx_2to1.at(knn_i2)<<"\n";
        }
        knnIdx2to1_RIFT.close();
    }

    idx_1->clear();
    idx_2->clear();
    std::ofstream matchIdx;
    matchIdx.open ("crossMatchIdx_RIFT.txt");
    for(u16 i=0; i<desc_1->points.size(); ++i)
    {
        if( i == idx_2to1.at(idx_1to2.at(i)) )
        {
            idx_1->push_back(i);
            idx_2->push_back(idx_1to2.at(i));
            matchIdx << i <<" "<< idx_1to2.at(i)<<"\n";
        }
    }
    matchIdx.close();

    // matches found
    if( idx_1->size()>1 && idx_2->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//template <typename descriptorT>
uc8
extractFeatures::crossMatching(const pcl::PointCloud<SHOT352>::Ptr & desc_1,
                               const pcl::PointCloud<SHOT352>::Ptr & desc_2,
                               std::vector<u16> *idx_1, std::vector<u16> *idx_2)
{
    std::vector<u16> idx_1to2;
    std::vector<u16> idx_2to1;
    std::vector<f32> dist_1to2;
    std::vector<f32> dist_2to1;

    // find the matching from desc_1 -> desc_2
    matchKeyPts(desc_1, desc_2, &idx_1to2, &dist_1to2);
    // find the matching from desc_2 -> desc_1
    matchKeyPts(desc_2, desc_1, &idx_2to1, &dist_2to1);

    idx_1->clear();
    idx_2->clear();
    std::ofstream matchIdx;
    matchIdx.open ("crossMatchIdx_SHOT.txt");
    for(u16 i=0; i<desc_1->points.size(); ++i)
    {
        if( i == idx_2to1.at(idx_1to2.at(i)) )
        {
            idx_1->push_back(i);
            idx_2->push_back(idx_1to2.at(i));
            matchIdx << i <<" "<< idx_1to2.at(i)<<"\n";
        }
    }
    matchIdx.close();
    // matches found
    if( idx_1->size()>1 && idx_2->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

//template <typename descriptorT>
uc8
extractFeatures::crossMatching(const pcl::PointCloud<SHOT1344>::Ptr & desc_1,
                               const pcl::PointCloud<SHOT1344>::Ptr & desc_2,
                               std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                               std::vector<f32> *match_dist)
{
    std::vector<u16> idx_1to2;
    std::vector<u16> idx_2to1;
    std::vector<f32> dist_1to2;
    std::vector<f32> dist_2to1;

    // find the matching from desc_1 -> desc_2
    matchKeyPts(desc_1, desc_2, &idx_1to2, &dist_1to2);
    // find the matching from desc_2 -> desc_1
    matchKeyPts(desc_2, desc_1, &idx_2to1, &dist_2to1);

    idx_1->clear();
    idx_2->clear();
    match_dist->clear();
    std::ofstream matchIdx;
    matchIdx.open ("crossMatchIdx_SHOT.txt");
    for(u16 i=0; i<desc_1->points.size(); ++i)
    {
        if( i == idx_2to1.at(idx_1to2.at(i)) )
        {
            idx_1->push_back(i);
            idx_2->push_back(idx_1to2.at(i));
            matchIdx << i <<" "<< idx_1to2.at(i)<<"\n";
            match_dist->push_back(dist_1to2.at(i));
        }
    }

    matchIdx.close();
    // matches found
    if( idx_1->size()>1 && idx_2->size()>1 )
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

///////////////////////////////////////////////////////////////////////////////////////
///// Remove outliers using RANSAC
///////////////////////////////////////////////////////////////////////////////////////
std::vector<s16>
extractFeatures::matchRansacPCL(PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud2,
                             std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                             f32 errThd, u16 maxIter)
{
    PointCloudT::Ptr corr_1 (new PointCloudT);
    PointCloudT::Ptr corr_2 (new PointCloudT);

    // get correspondences
    for(u16 i=0; i<idx_1->size(); ++i)
    {
        corr_1->push_back(cloud->points[ idx_1->at(i) ]);
        corr_2->push_back(cloud2->points[ idx_2->at(i) ]);
    }

    // created RandomSampleConsensus object and compute the appropriated model
    std::vector<s16> inliers;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crsac;
    crsac.setInputSource(corr_1);
    crsac.setInputTarget(corr_2);
    crsac.setInlierThreshold(errThd);
    crsac.setMaximumIterations(maxIter);
    crsac.getInliersIndices(inliers);
    return inliers;
}



std::vector<s16>
extractFeatures::matchRansac(PointCloudT::Ptr &cloud, PointCloudT::Ptr &cloud2,
                             std::vector<u16> *idx_1, std::vector<u16> *idx_2,
                             f32 errThd, u16 maxIter)
{
    PointCloudT::Ptr corr_1 (new PointCloudT);
    PointCloudT::Ptr corr_2 (new PointCloudT);

    // get correspondences
    for(u16 i=0; i<idx_1->size(); ++i)
    {
        corr_1->push_back(cloud->points[ idx_1->at(i) ]);
        corr_2->push_back(cloud2->points[ idx_2->at(i) ]);
    }

    // Ransac
    u16 i = 0;
    while(i<maxIter)
    {
        u16 randIdx[3] = {0, 0, 0};

        commonFunc::randomIdx(randIdx, (u16) corr_1->points.size());
        pcl::TransformationFromCorrespondences transFromCorr;
        for ( uc8 idx =0; idx<3; ++idx)
        {
               Eigen::Vector3f from(corr_1->points.at(randIdx[idx]).x,
                                    corr_1->points.at(randIdx[idx]).y,
                                    corr_1->points.at(randIdx[idx]).z);

               Eigen::Vector3f  to (corr_2->points.at(randIdx[idx]).x,
                                    corr_2->points.at(randIdx[idx]).y,
                                    corr_2->points.at(randIdx[idx]).z);
               transFromCorr.add(from, to, 1.0);//all the same weight
        }
        Eigen::Matrix4f transMat= transFromCorr.getTransformation().matrix();
        ++i;
    }

    // created RandomSampleConsensus object and compute the appropriated model
    std::vector<s16> inliers;
    pcl::registration::CorrespondenceRejectorSampleConsensus<PointT> crsac;
    crsac.setInputSource(corr_1);
    crsac.setInputTarget(corr_2);
    crsac.setInlierThreshold(errThd);
    crsac.setMaximumIterations(maxIter);
    crsac.getInliersIndices(inliers);
    return inliers;
}

///////////////////////////////////////////////////////////////////////////////////////
///// Remove far distance points
///////////////////////////////////////////////////////////////////////////////////////
void
extractFeatures::distClip(PointCloudT::Ptr &cloud,
                          PointCloudT::Ptr &cloud2,float Thd)
{
    pcl::copyPointCloud(*cloud, *cloud2);
    s16 idx = cloud2->points.size();
    while(idx--)
    {   // estimate the distance to the camera
        if( std::abs(cloud->points.at(idx).x) > Thd || std::abs(cloud->points.at(idx).y) > Thd ||
            std::abs(cloud->points.at(idx).z) > Thd)
        {
            cloud2->points.erase(cloud2->points.begin()+idx);
        }
    }
}

void
extractFeatures::distClip(PointCloudT::Ptr &cloud, float Thd)
{
    s16 idx = cloud->points.size();
    while(idx--)
    {
        if(  std::abs(cloud->points.at(idx).x) > Thd || std::abs(cloud->points.at(idx).y) > Thd ||
             std::abs(cloud->points.at(idx).z) > Thd)
        {
            cloud->points.erase(cloud->points.begin()+idx);
        }
    }
}


///////////////////////////////////////////////////////////////////////////////////////
///// extract edges from point cloud
///////////////////////////////////////////////////////////////////////////////////////
void
extractFeatures::extractEdge(PointCloudT::Ptr &cloud, PointCloudT::Ptr & edges)
{
    PointT *pt = &cloud->points.at(0);
}

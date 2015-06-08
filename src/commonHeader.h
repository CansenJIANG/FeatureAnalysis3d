#ifndef COMMONHEADER_H
#define COMMONHEADER_H

// standard library
#include <iostream>
#include <vector>

// Qt header
#include <QString>
#include <QtDebug>
#include <QMainWindow>
#include <QDialog>
#include <QFileDialog>
#include <QTextEdit>
#include <QPlainTextEdit>
#include <QtCore/QEvent>
#include <QtGui/QKeyEvent>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_base.h>
#include <pcl/common/point_operators.h>
#include <pcl/register_point_struct.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common_headers.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/keypoints/keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/features/usc.h>
#include <pcl/filters/filter.h>
#include <pcl/features/vfh.h>
#include <pcl/common/eigen.h>
#include <pcl/range_image/range_image.h>
#include <pcl/features/range_image_border_extractor.h>
//#include <pcl/visualization/range_image_visualizer.h> /* CONFLICT WITH QT */
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <boost/thread/thread.hpp>
#include <pcl/console/parse.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types_conversion.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/io/pcd_grabber.h>

// headers for ransac
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>


// PCL point cloud registration
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/warp_point_rigid_6d.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>

// Eigen
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include <Eigen/Core>
#include <Eigen/Dense>

// Data type configuration
typedef signed char c8;
typedef signed int  s16;
typedef signed long int s32;
typedef signed short int ss16;

typedef unsigned char uc8;
typedef unsigned int u16;
typedef unsigned short int us16;
typedef unsigned long int u32;

typedef float f32;
typedef double f64;

// Pcl data type configuration
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointT> PointColor;
typedef pcl::Normal NormalT;
typedef pcl::PointCloud<NormalT> NormalsT;

typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointCloudI;

// Pcl feature descriptor
typedef pcl::Histogram<32> RIFT32;
typedef pcl::SHOT352 SHOT352;
typedef pcl::SHOT1344 SHOT1344;
#ifndef PI
#define PI 3.141592654
#endif

#endif // COMMONHEADER_H



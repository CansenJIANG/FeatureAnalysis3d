#ifndef PCLVIEWER_H
#define PCLVIEWER_H

// include common header files and data type configuration
#include "commonFunc.h"
#define EIGEN_YES_I_KNOW_SPARSE_MODULE_IS_NOT_STABLE_YET
#include "extractFeatures.h"
#include <Eigen/Core>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "trk3dfeatures.h"

/////////////////////////////////////////////////////////////////////////////////////
/// define call back structure for point clicking selection
/////////////////////////////////////////////////////////////////////////////////////
struct str_clickPts{
    // structure used to pass arguments to the callback function
    PointCloudT::Ptr clicked_points_3d;
    // pcl::visualization::PCLVisualizer::Ptr viewerPtr;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerPtr;
    // text editor output
    QPlainTextEdit *txtEditor;
    // color of keypoints
    uc8 ptColor[3];
};

/////////////////////////////////////////////////////////////////////////////////////
/// define key point detector constructor
/////////////////////////////////////////////////////////////////////////////////////
struct str_featDescr{
    // key point name in pcl viewer
    std::string name;
    // key point detector index
    uc8 detectorIdx;
    // Key point detector setting parameters
    f32 params[5];
    // key point color
    uc8 matchColor[3];
    // key point size
    uc8 viewSize;
    // match indices
    std::vector<u16> matchIdx1;
    std::vector<u16> matchIdx2;
    std::vector<f32> matchDist;
    std::vector<f32> matchLineWidth;
    std::vector<f32> goodMatchDist;
    // state of drawing lines
    uc8 lineDrawOn;
    // draw line index
    u16 lineIdx;
    // line width
    f32 lineWidth;
};

/////////////////////////////////////////////////////////////////////////////////////
/// define feature descriptor constructor
/////////////////////////////////////////////////////////////////////////////////////
struct str_keyPts{
    // key point detector name
    std::string name;
    // filtered key point name
    std::string filteredName;
    std::string filteredName2;
    // key point detector index
    uc8 detectorIdx;
    // Key point detector setting parameters
    f32 params[15];
    // key point color
    uc8 viewColor[3];
    // key point size
    uc8 viewSize;
};

/////////////////////////////////////////////////////////////////////////////////////
/// define point cloud sequence analysis constructor
/////////////////////////////////////////////////////////////////////////////////////
struct str_loadSeq
{
    bool seqMode;
    u16 seqIdx;
    PointCloudT::Ptr pcSeq;
    bool repeatSeq;
    QString pcdPath;
    QStringList files;
    bool loadFullSeq;
    std::vector<PointCloudT> fullSeq;
    float fpsSeq;
    bool trackNext;
    u16 drawMatchIdx;
//    std::vector< std::vector<u16> > trkCurrIdx;
//    std::vector< std::vector<u16> > trkNextIdx;
    std::vector< PointCloudT > trkCurrPts;
    std::vector< PointCloudT > trkNextPts;
    std::vector< std::vector<f32> > trkCorrDist;
    pcl::PointCloud<SHOT1344>::Ptr trkCurrDesc;
    std::vector< PointCloudT > trkSeqPts;
    std::vector< std::vector< f32 > > trkSeqDist;
};

struct str_drawShape
{
    uc8 color[3];
};

namespace Ui
{
    class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit PCLViewer (QWidget *parent = 0);
    ~PCLViewer ();

public slots:
    // slider to change the size of point cloud
    void pSliderValueChanged (int value);
    // slider to change the line width
    void lineWidthSlider(int value);

protected:
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer; // Point cloud viewer
    std::string fName;                  // name of the point cloud 1
    std::string fName2;                  // name of the point cloud 2
    PointCloudT::Ptr cloud;             // Point cloud of scene
    PointCloudT::Ptr cloud2;            // New point cloud of scene
    PointCloudT::Ptr keyPts;            // key points detected from the scene
    PointCloudT::Ptr keyPts2;           // key points detected from new point cloud
    PointCloudT::Ptr featurePts;        // feature points manually selected from the point cloud
    PointCloudT::Ptr featurePts2;       // feature points manually selected from the point cloud
    PointCloudT::Ptr filteredKeyPts;    // remained key points after filtering
    PointCloudT::Ptr filteredKeyPts2;   // remained key points after filtering
    extractFeatures *featureDetector;   // 3d point cloud feature detector
    extractFeatures *featureDetector2;  // 3d point cloud feature detector

    PointCloudT::Ptr thirdPc;
    // container for point cloud colors
    bool showColor;
    std::vector<uc8> cloudR;
    std::vector<uc8> cloudG;
    std::vector<uc8> cloudB;
    uc8 red;
    uc8 green;
    uc8 blue;

    // Preprocessing params
    f32 clipThd;
    f32 shiftPC_X;
    f32 shiftPC_Y;
    f32 shiftPC_Z;

    // point cloud sequence analysis structure
    str_loadSeq loadSeqStr;

    // drawing analysis
    str_drawShape drawShapeStr;

    // Keypoint detector parameters structure
    struct str_keyPts keyPtsStr;

    // Feature detector parameters structure
    struct str_featDescr featDescrStr;

    // func to select feature points
    void on_getPoint_clicked();

    // func to draw key points
    void drawKeyPts(const PointCloudT::Ptr &keyPts, const std::string pcName,
                       const uc8 pColor[], const uc8 ptSize);

    // func to draw matching lines
    void drawMatches(const PointCloudT::Ptr &cloud, const PointCloudT::Ptr & cloud2,
                     const std::vector<f32> &matchDist, const uc8 viewColor[]);

    // func to detect the key points
    void detectKeypts(PointCloudT::Ptr &cloud, PointCloudT::Ptr &keyPts,
                      extractFeatures *fDetector,  str_keyPts &keyPtsStr);

    // func to conduct key point filtering
    void filterKeyPts(PointCloudT::Ptr &cloud, PointCloudT::Ptr &keyPts,
                      PointCloudT::Ptr &filteredKeyPts,
                      extractFeatures *fDetector, str_keyPts &keyPtsStr);

    // func to track feature points
    void trkFeatures2Frames(void);
    void drawSeqMatches(void);
    void removeSeqLines(void);

    // Add point picking callback to viewer:
    struct str_clickPts clickPtsStr;

private slots:

    ///*******************************************
    ///* Point Cloud Loading SLOTs               *
    ///*******************************************
    // click botton to load point cloud
    void on_LoadPC_clicked();
    void on_showCloud_1_clicked();
    void on_add_PC_clicked();
    void on_addThirdPc_clicked();
    void on_showCloud_2_clicked();
    void on_showCloud_3_clicked();
    // check box to visualize point cloud color
    void on_chkbox_withColor_clicked();

    // click button to save selected features
    void on_saveFeatures_clicked();

    // click button to take a screenshot of the widget
    void on_takeScreenshot_clicked();

    // click button to delete all the selected features
    void on_cleanFeatures_clicked();

    // click button to delete the last selected feature
    void on_delOnePt_clicked();

    // click button to remove points further than threshold
    void on_clipThreshold_editingFinished();


    ///*******************************************
    ///* Key Points detectors SLOTs              *
    ///*******************************************
    // compose box for different feature detectors
    void on_keyPtDetectors_activated(int index);

    // text editor to set key point detectors' parameters
    void on_kParamVal_0_editingFinished();
    void on_kParamVal_1_editingFinished();
    void on_kParamVal_2_editingFinished();
    void on_kParamVal_3_editingFinished();
    void on_kParamVal_4_editingFinished();
    void on_kParamVal_5_editingFinished();
    void on_kParamVal_6_editingFinished();
    void on_kParamVal_7_editingFinished();
    void on_kParamVal_8_editingFinished();
    void on_kParamVal_9_editingFinished();
    void on_kParamVal_10_editingFinished();
    void on_kParamVal_11_editingFinished();

    // func to initialize key point detectors' parameters
    void on_keyPtColor_activated(int index);

    // func to set keypoint visualization size
    void on_keyPtSize_activated(int index);

    // func to delete the detected key points
    void on_clearKeypts_clicked();

    // click button to run key point detector
    void on_runKeyPtsDetector_1_clicked();
    // func to show/hide the keypoints
    void on_showKeypts_1_clicked();
    // func to filter the keypoint using knn and voxel filtering
    void on_filterKeypts_1_clicked();
    // func to show/hide filtered keypoints
    void on_showFilteredKeypts_1_clicked();
    // func for point cloud 2
    void on_runKeyPtsDetector_2_clicked();
    void on_showKeypts_2_clicked();
    void on_filterKeypts_2_clicked();
    void on_showFilteredKeypts_2_clicked();



    ///*******************************************
    ///* Feature Matching SLOTs                  *
    ///*******************************************
    // text editor to set feature point descriptors' parameters
    void on_fParamVal_0_editingFinished();
    void on_fParamVal_1_editingFinished();
    void on_fParamVal_2_editingFinished();
    void on_fParamVal_3_editingFinished();
    void on_fParamVal_4_editingFinished();
    void on_fParamVal_5_editingFinished();

    // func to initialize the feature descriptor structure
    void on_featureDescriptor_activated(int index);

    // func to match the detected key points
    void on_matchKeypts_clicked();

    // func to remove outliers using ransac
//    void on_matchRansac_clicked();

    // func extract the edges from point cloud
    void on_extractEdge_1_clicked();

    // func to draw matches
    void on_drawMatches_clicked();

    // func to remove matching lines
    void on_removeLines_clicked();

    void on_comboBox_activated(int index);


    ///*******************************************
    ///* Preprocessing of point cloud            *
    ///*******************************************

    void on_loadSelectedFeat_clicked();

    void on_loadMatchIdx_clicked();

    void on_transformPc_clicked();

    void on_clipPC_clicked();

    void on_shiftX_val_editingFinished();
    void on_shiftY_val_editingFinished();
    void on_shiftZ_val_editingFinished();


private:
    Ui::PCLViewer *ui;

};

#endif // PCLVIEWER_H

Feature Analysis in 3d
=================
GUI for Feature Analysis in 3d, mainly on performances evaluations on 3d feature detector and descriptors.

Project file Overview
---------------------

    project
    |- doc/             # documentation for the study
    |  +- paper/        # manuscript(s), whether generated or not
    |
    |- data             # raw and primary data, are not changed once created 
    |  |- raw/          # raw data, will not be altered
    |  +- clean/        # cleaned data, will not be altered once created
    |
    |- src/             # any programmatic code
    |- results          # all output from workflows and analyses
    |  |- figures/      # graphs, likely designated for manuscript figures
    |  +- pictures/     # diagrams, images, and other non-graph graphics
    |
    |- scratch/         # temporary files that can be safely deleted or lost
    |
    |- README           # the top level description of content
    |- FeatureAnalysis3d.pro # the project descriptor for QTcreator
    |- Makefile         # executable Makefile for this study, if applicable
    |- datapackage.json # metadata for the (input and output) data files 


Dependencies
------------
* Qt-4 (https://www.qt.io/download-open-source/)
* VTK-5.8 Library (http://www.vtk.org/download/)
* Point Cloud Library 1.8.0 (http://pointclouds.org/) 
* Eigen Library (http://eigen.tuxfamily.org/index.php?title=Main_Page)
* Boost Library (http://www.boost.org/)

For linux user:
Your can download most of the software from Synaptic Package Manager (https://help.ubuntu.com/community/SynapticHowto). 
For the latest version of PCL, you are suggest to download from the Github release (https://github.com/PointCloudLibrary/pcl).

For windows user: Not tested yet. 

Todo
----
- [ ] Update README file whenever new functionalities added to the GUI.
- [ ] Detail analysis for evaluation on different feature detector and descriptor.
- [ ] Comment all the functions that you added.
- [ ] Weekly reports for the progress.
- [ ] 



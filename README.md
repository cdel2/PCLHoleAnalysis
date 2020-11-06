# PCLHoleAnalysis
Detecting holes in a point cloud and giving measures such as radii, center and normal at the center with PCL Library.

First, edges are detected by using ConcaveHull, then clustered using EuclideanClusterExtraction (see colors of the different clusters) :
![PCLVisualization](img/PieceVisualization.png)

This enable to isolate 2 circles for computing metrics :
![PCLVisualization](img/PieceMetrics.png)

A file "HoleDetectionDataWithNormals.ply" containing the normals of the initial cloud's points is then created.
The two last points in the file are the normals of the detected circle's centers.

## Build & Run program - Requirements
- PCL version 1.2 or above installed
- CMake

Build the code using CMake, with Visual Studio 2015 generator and platform Win32.


#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <pcl/io/io.h>
#include <pcl/io/auto_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <cmath>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

using namespace std;
using namespace pcl;

PointCloud <PointXYZ>::Ptr hull(new PointCloud<PointXYZ>());
PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>());
PointCloud<Normal>::Ptr normals(new PointCloud<Normal>);
vector<PointCloud<PointXYZ>::Ptr> clusters;
vector<PointXYZ> holeCenters;

void visualize() {
	visualization::PCLVisualizer::Ptr viewer(new visualization::PCLVisualizer("3D Viewer"));
	visualization::PointCloudColorHandlerCustom<PointXYZ> single_color(hull, 255, 255, 255);
	visualization::PointCloudColorHandlerCustom<PointXYZ> cluster_color_a(clusters[0], 255, 0, 0);
	visualization::PointCloudColorHandlerCustom<PointXYZ> cluster_color_b(clusters[1], 0, 255, 0);
	visualization::PointCloudColorHandlerCustom<PointXYZ> cluster_color_c(clusters[2], 0, 0, 255);

	viewer->addCoordinateSystem(10, 67.9136, -129.446, -34.7497);


	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);

		if (hull->width != 0 && !viewer->updatePointCloud(hull, single_color, "hull points")) {
			viewer->addPointCloud(hull, single_color, "hull points");
		}

		if (clusters[0]->width != 0 && !viewer->updatePointCloud(clusters[0], cluster_color_a, "cluster 0")) {
			viewer->addPointCloud(clusters[0], cluster_color_a, "cluster 0");
		}

		if (clusters[1]->width != 0 && !viewer->updatePointCloud(clusters[1], cluster_color_b, "cluster 1")) {
			viewer->addPointCloud(clusters[1], cluster_color_b, "cluster 1");
		}

		if (clusters[2]->width != 0 && !viewer->updatePointCloud(clusters[2], cluster_color_c, "cluster 2")) {
			viewer->addPointCloud(clusters[2], cluster_color_c, "cluster 2");
		}
	}
}

void findConcaveHull()
{
	ConcaveHull<PointXYZ> chull;
	chull.setInputCloud(cloud);
	chull.setAlpha(0.4);
	chull.reconstruct(*hull);
}

void printClusterTolerance() {
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
	tree->setInputCloud(hull);

	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZ> ec;
	ec.setSearchMethod(tree);
	ec.setInputCloud(hull);

	for (float i = 0; i < 5; i += 0.1) {
		ec.setClusterTolerance(i);
		ec.extract(cluster_indices);

		cout << "Setting max distance between two points of a cluster to " << i << " results in " << cluster_indices.size() << " clusters" << endl;

		cluster_indices.empty();
		cluster_indices.resize(0);
	}
}

void findClusters() {
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>());
	tree->setInputCloud(hull);

	vector<PointIndices> cluster_indices;
	EuclideanClusterExtraction<PointXYZ> ec;
	ec.setClusterTolerance(0.6);
	ec.setSearchMethod(tree);
	ec.setInputCloud(hull);
	ec.extract(cluster_indices);

	int j = 0;
	for (vector<PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		PointCloud<PointXYZ>::Ptr hull_cluster(new PointCloud<PointXYZ>());
		for (vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit) {
			hull_cluster->push_back((*hull)[*pit]);
		}

		hull_cluster->width = hull_cluster->size();
		hull_cluster->height = 1;
		hull_cluster->is_dense = true;

		clusters.push_back(hull_cluster);

		j++;
	}
}


float findEuclidianDistance(const PointXYZ& pointA, const PointXYZ& pointB) {
	return sqrt(pow(pointA.x - pointB.x, 2) + pow(pointA.y - pointB.y, 2) + pow(pointA.z - pointB.z, 2));
}

float findMaxHoleRadius(const PointCloud<PointXYZ>::Ptr& hole, pair<PointXYZ, PointXYZ> &maxCoord) {
	float max = 0;

	for (size_t i = 0; i < hole->size(); i++) {
		for (size_t j = i + 1; j < hole->size(); j++) {
			if (findEuclidianDistance(hole->points[i], hole->points[j]) > max) {
				max = findEuclidianDistance(hole->points[i], hole->points[j]);
				maxCoord = make_pair(hole->points[i], hole->points[j]);
			}
		}
	}

	return max / 2;
}

float findMeanMaxHoleRadius(const PointCloud<PointXYZ>::Ptr& hole) {
	vector<float> radius;

	for (size_t i = 0; i < hole->size(); i++) {
		for (size_t j = i + 1; j < hole->size(); j++) {
			radius.push_back(findEuclidianDistance(hole->points[i], hole->points[j]) / 2);
		}
	}

	sort(radius.begin(), radius.begin() + radius.size());

	radius.erase(radius.begin(), radius.begin() + radius.size() - hole->size() / 2);

	float mean = 0;
	for (size_t i = 0; i < radius.size(); i++) {
		mean += radius[i];
	}

	return mean / (hole->size() / 2);
}

PointXYZ findMeanHoleCenter(const PointCloud<PointXYZ>::Ptr& hole, const pair<PointXYZ, PointXYZ>& maxCoord) {
	float x = (maxCoord.first.x + maxCoord.second.x) / 2;
	float y = (maxCoord.first.y + maxCoord.second.y) / 2;
	float z = (maxCoord.first.z + maxCoord.second.z) / 2;

	return(PointXYZ(x, y, z));
}

void computeNormals() {
	NormalEstimation<PointXYZ, Normal> n;
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	n.setInputCloud(cloud);
	n.setSearchMethod(tree);
	n.setRadiusSearch(9);
	n.compute(*normals);

	PointCloud<PointNormal>::Ptr cloud_with_normals(new PointCloud<PointNormal>);
	concatenateFields(*cloud, *normals, *cloud_with_normals);

	io::savePLYFileASCII("HoleDetectionDataWithNormals.ply", *cloud_with_normals);
}

void printMetrics() {
	cout << "Initial cloud's size : " << cloud->points.size() << endl;
	cout << "Concave hull has: " << hull->points.size() << " data points." << endl;

	for (size_t i = 1; i < clusters.size(); i++) {
		cout << "---------------------------------" << endl;

		pair<PointXYZ, PointXYZ> maxCoord;
		cout << "Radius for hole " << i << " : " << findMaxHoleRadius(clusters[i], maxCoord) << endl;
		cout << "Mean radius for hole " << i << " : " << findMeanMaxHoleRadius(clusters[i]) << endl;

		PointXYZ center = findMeanHoleCenter(clusters[i], maxCoord);
		cout << "Center for hole " << i << " : (" << center.x << "," << center.y << "," << center.z << ")" << endl;

		cloud->push_back(center);
	}
}

int main() {
	ifstream input("../data/HoleDetectionTestData.txt");
	string line;
	PointXYZ point;

	while (getline(input, line)) {
		istringstream record(line);
		record >> point.x;
		record >> point.y;
		record >> point.z;

		cloud->push_back(point);
	}

	findConcaveHull();

	//printClusterTolerance();
	findClusters();

	printMetrics();

	//computeNormals();

	visualize();

	return 0;
}
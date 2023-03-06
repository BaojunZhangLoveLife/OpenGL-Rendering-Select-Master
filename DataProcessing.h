#pragma once
#include <QVector3D>
//PCL
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply/ply.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
//VTK
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>

struct SurfaceData{
	std::vector<float> vecPoints;
	std::vector<float> vecFaceTriangles;
	std::vector<float> vecVertexNormals;
};

class DataProcessing{
public:
	DataProcessing();
	~DataProcessing();
	// Get the current path of the program
	std::string getProgramPath();
	// normalize the original point cloud data
	void normalize(std::vector<QVector3D> data);

	void loadPointData(const char* filename);
	void loadMeshData(char* filename);

	void getMeshData(pcl::PolygonMesh meshPath);
	void getNormalVector(std::string cloudPath);

	/// ----------------------------------------
	/// Interface for data type conversion
	/// ----------------------------------------
	void writePlyData(pcl::PolygonMesh mesh);
	void ply2ply(std::string src, std::string dst);
	void ply2pcd(std::string ply, std::string pcd);
	void txt2pcd(std::string filename, std::string pcdPath);

	std::vector<int> nearestKSearch(std::string txtPath, pcl::PointXYZ query_point);
	// Find the nearest vertex of the world coordinate point
	int findNearestVertex(QVector3D worldPos, std::vector<QVector3D> meshVertices);
	/// ----------------------------------------
	/// Set of variables for processing data
	/// ----------------------------------------
	std::vector<QVector3D>		pointData;
	std::vector<float>			meshData;
	SurfaceData					surfaceData;

private:
	void clearMeshData();
	void getXYZMaxMin();
	void getCenterPoint(QVector3D& vec);

	QVector3D maxCoordinate;
	QVector3D minCoordinate;

	pcl::PointCloud<pcl::Normal>::Ptr normalsRefined;
};
#pragma once
#include <QVector3D>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>

struct SurfaceModelData{
	std::vector<float> vecPoints;
	std::vector<float> vecFaceTriangles;
	std::vector<float> vecVertexNormals;
};

class DataProcessing{
public:
	DataProcessing();
	~DataProcessing();
	std::string GetAppPath();
	void Normalize(std::vector<QVector3D> data);

	void LoadPointData(const char* path);
	void loadMeshData(char* filename);

	void GetMeshData(pcl::PolygonMesh mesh);
	void GetNormalVector(std::string pcdPath);

	void WritePlyData(pcl::PolygonMesh mesh);
	void Ply2Pcd(std::string ply, std::string pcd);
	void Stl2Ply(std::string stl, std::string ply);
	void Ply2Stl(std::string ply, std::string stl);
	void Ply2Ply(std::string src, std::string dst);
	void meshConvert(std::string filename);
	std::vector<QVector3D>		pointData;
	std::vector<float>			meshData;
	SurfaceModelData			surfaceModelData;

private:
	void ClearMeshData();
	void GetXYZMaxMin();
	void GetCenterPoint(QVector3D& vec);

	QVector3D maxCoordinate;
	QVector3D minCoordinate;

	pcl::PointCloud<pcl::Normal>::Ptr normalsRefined;
};
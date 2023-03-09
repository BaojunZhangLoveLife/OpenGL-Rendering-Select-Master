#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"

#include <QMatrix4x4>

DataProcessing::DataProcessing() {
	normalsRefined.reset(new pcl::PointCloud<pcl::Normal>);
}
DataProcessing::~DataProcessing() {

}
// clear mesh data
void DataProcessing::clearMeshData(){
	surfaceData.vecFaceTriangles.clear();
	surfaceData.vecVertexNormals.clear();
	surfaceData.vecPoints.clear();
}
std::string DataProcessing::getProgramPath(){
	QString qAppDir = QCoreApplication::applicationDirPath();
	std::string::size_type iPos = (qAppDir.toStdString().find_last_of('\\') + 1) == 0 ? qAppDir.toStdString().find_last_of('/') + 1 : qAppDir.toStdString().find_last_of('\\') + 1;
	return qAppDir.toStdString().substr(0, iPos);
}
void DataProcessing::loadPointData(const char* path){
	std::fstream readTextData(path);
	if (!readTextData) return;
	float x, y, z;
	while (readTextData >> x >> y >> z){
		QVector3D data = { x, y, z };
		pointData.emplace_back(data);
	}
	readTextData.close();
}
// get center point of point data.
void DataProcessing::getCenterPoint(QVector3D& vec){
	if (0 == pointData.size()) return;
	vec = {
		(maxCoordinate.x() + minCoordinate.x()) / 2,
		(maxCoordinate.y() + minCoordinate.y()) / 2,
		(maxCoordinate.z() + minCoordinate.z()) / 2
	};
}
void DataProcessing::txt2pcd(std::string filename, std::string pcdPath){
	std::ifstream fs;
	pcl::PointCloud<pcl::PointXYZ> cloud;
	fs.open(filename.c_str(), std::ios::binary);
	if (!fs.is_open() || fs.fail()){
		PCL_ERROR("Could not open file '%s'! Error : %s\n", filename.c_str(), strerror(errno));
		fs.close();
		return;
	}
	std::string line;
	std::vector<std::string> st;

	while (!fs.eof()){
		std::getline(fs, line);
		// Ignore empty lines
		if (line.empty())
			continue;
		boost::trim(line);
		boost::split(st, line, boost::is_any_of("\t\r "), boost::token_compress_on);

		if (st.size() != 3)
			continue;
		cloud.push_back(pcl::PointXYZ(float(atof(st[0].c_str())), float(atof(st[1].c_str())), float(atof(st[2].c_str()))));
	}
	fs.close();
	cloud.width = cloud.size(); cloud.height = 1; cloud.is_dense = true;
	pcl::PCDWriter w;
	w.writeASCII(pcdPath,cloud);

}
// normalize the original point cloud data
void DataProcessing::normalize(std::vector<QVector3D> data){
	pointData = data;
	getXYZMaxMin();

	QVector3D centerPoint;
	getCenterPoint(centerPoint);
	for (int i = 0; i < pointData.size(); i++){
		pointData[i].setX(pointData[i].x() - centerPoint.x());
		pointData[i].setY(pointData[i].y() - centerPoint.y());
		pointData[i].setZ(pointData[i].z() - centerPoint.z());
	}
	getXYZMaxMin();

	float max = 0;
	if (max <= maxCoordinate.x())	max = maxCoordinate.x();
	if (max <= maxCoordinate.y())	max = maxCoordinate.y();
	if (max <= maxCoordinate.z())	max = maxCoordinate.z();

	float factor = 1.0 / max;
	for (int i = 0; i < pointData.size(); i++){
		pointData[i].setX(pointData[i].x() * factor);
		pointData[i].setY(pointData[i].y() * factor);
		pointData[i].setZ(pointData[i].z() * factor);
	}
}
// max and min XYZ.
void DataProcessing::getXYZMaxMin(){
	if (0 == pointData.size())	return;
	QVector3D vecMax = {pointData[0].x(),pointData[0].y(),pointData[0].z()};
	QVector3D vecMin = vecMax;

	for (int i = 0; i < pointData.size(); i++){
		if (vecMax.x() < pointData[i].x())	vecMax.setX(pointData[i].x());
		if (vecMax.y() < pointData[i].y())	vecMax.setY(pointData[i].y());
		if (vecMax.z() < pointData[i].z())	vecMax.setZ(pointData[i].z());

		if (vecMin.x() > pointData[i].x())	vecMin.setX(pointData[i].x());
		if (vecMin.y() > pointData[i].y())	vecMin.setY(pointData[i].y());
		if (vecMin.z() > pointData[i].z())	vecMin.setZ(pointData[i].z());
	}
	maxCoordinate = vecMax;
	minCoordinate = vecMin;
}
// transform a mesh into a mesh ,the former mesh has various faces(triangle,quadrangle,pentagon.etc), the next mesh has only a triangle face.
void DataProcessing::ply2ply(std::string src, std::string dst) {
	vtkSmartPointer<vtkPLYReader> ply2plyPlyReader = vtkSmartPointer<vtkPLYReader>::New();
	ply2plyPlyReader->SetFileName(src.c_str());
	ply2plyPlyReader->Update();

	vtkSmartPointer<vtkTriangleFilter> ply2plyStlFilter = vtkSmartPointer<vtkTriangleFilter>::New();
	ply2plyStlFilter->SetInputData(ply2plyPlyReader->GetOutput());

	vtkSmartPointer<vtkPLYWriter> ply2plyPlyWriter = vtkSmartPointer<vtkPLYWriter>::New();
	ply2plyPlyWriter->SetFileName(dst.c_str());
	ply2plyPlyWriter->SetInputConnection(ply2plyStlFilter->GetOutputPort());
	ply2plyPlyWriter->SetFileTypeToASCII();
	ply2plyPlyWriter->SetColorModeToOff();
	ply2plyPlyWriter->Update();
	ply2plyPlyWriter->Write();
}

void DataProcessing::ply2pcd(std::string ply, std::string pcd){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(ply, *cloud);
	pcl::io::savePCDFile(pcd, *cloud);
}
std::vector<int> DataProcessing::nearestKSearch(pcl::PolygonMesh mesh, pcl::PointXYZ query_point) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);
	std::vector<int> k_indices;
	std::vector<float> k_distances;
	kdtree->nearestKSearch(query_point, 10, k_indices, k_distances);
	return k_indices;
}

//Data Type Conversion(transfer mesh object to a ply file)
void DataProcessing::writePlyData(pcl::PolygonMesh mesh,std::string path){
	std::ofstream fs;
	fs.open(path);
	if (fs){
		int nr_points = mesh.cloud.width * mesh.cloud.height;
		int point_size = mesh.cloud.data.size() / nr_points;
		int nr_faces = mesh.polygons.size();

		// Write header
		fs << "ply";
		fs << "\nformat ascii 1.0";
		fs << "\ncomment PCL generated";
		// Vertices
		fs << "\nelement vertex " << mesh.cloud.width * mesh.cloud.height;
		fs << "\nproperty float x"
			"\nproperty float y"
			"\nproperty float z";

		fs << "\nproperty float nx"
			"\nproperty float ny"
			"\nproperty float nz";
		// Faces
		fs << "\nelement face " << nr_faces;
		fs << "\nproperty list uchar int vertex_indices";
		fs << "\nend_header\n";

		for (std::size_t i = 0; i < nr_points; i++) {
			for (std::size_t d = 0; d < mesh.cloud.fields.size(); ++d) {
				// adding vertex
				if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (mesh.cloud.fields[d].name == "x" ||mesh.cloud.fields[d].name == "y" ||mesh.cloud.fields[d].name == "z")) {
					float value;
					memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset], sizeof(float));
					fs << value << " ";
				}
			}
			fs << normalsRefined->points[i].normal_x << " " << normalsRefined->points[i].normal_y << " " << normalsRefined->points[i].normal_z << " " << "\n";
		}
		// Write down faces
		for (std::size_t i = 0; i < nr_faces; i++) {
			fs << mesh.polygons[i].vertices.size() << " ";
			for (std::size_t j = 0; j < mesh.polygons[i].vertices.size() - 1; ++j)
				fs << mesh.polygons[i].vertices[j] << " ";
			fs << mesh.polygons[i].vertices.back() << '\n';
		}
		fs.close();
	}
}
//Mesh
void DataProcessing::loadMeshData(char* filename){
	FILE* file = fopen(filename, "r");
	if (file){
		fseek(file, 0, SEEK_END);
		clearMeshData();
		float* surfaceVertexXYZ = (float*)malloc(ftell(file));
		float* surfaceVertexNorm = (float*)malloc(ftell(file));
		fseek(file, 0, SEEK_SET);

		char buffer[MESH_BUFFER_MAX_SIZE];
		fgets(buffer, MESH_BUFFER_SIZE, file);
		// Find total vertex
		while (strncmp("element vertex", buffer, strlen("element vertex")) != 0){
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}
		strcpy(buffer, buffer + strlen("element vertex"));

		int surfaceTotalConnectedPoints;
		sscanf(buffer, "%d", &surfaceTotalConnectedPoints);

		// Find total face
		fseek(file, 0, SEEK_SET);
		while (strncmp("element face", buffer, strlen("element face")) != 0){
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}
		strcpy(buffer, buffer + strlen("element face"));
		int surfaceTotalFaces;
		sscanf(buffer, "%d", &surfaceTotalFaces);

		// go to end_header
		while (strncmp("end_header", buffer, strlen("end_header")) != 0){
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}

		// read vertices
		for (int iterator = 0,index = 0; iterator < surfaceTotalConnectedPoints; iterator++){
			if (iterator == 0)  pointData.clear();
			fgets(buffer, MESH_BUFFER_SIZE, file);

			sscanf(buffer, "%f %f %f %f %f %f",
				&surfaceVertexXYZ[index], &surfaceVertexXYZ[index + 1], &surfaceVertexXYZ[index + 2],
				&surfaceVertexNorm[index], &surfaceVertexNorm[index + 1], &surfaceVertexNorm[index + 2]);

			QVector3D data = { surfaceVertexXYZ[index], surfaceVertexXYZ[index + 1], surfaceVertexXYZ[index + 2] };
			pointData.emplace_back(data);
			index += 3;
		}
		normalize(pointData);
		for (int i = 0; i < pointData.size(); i++){
			surfaceData.vecPoints.emplace_back(pointData[i].x());
			surfaceData.vecPoints.emplace_back(pointData[i].y());
			surfaceData.vecPoints.emplace_back(pointData[i].z());
		}

		// read faces
		for (int iterator = 0; iterator < surfaceTotalFaces; iterator++){
			fgets(buffer, MESH_BUFFER_SIZE, file);
			if (buffer[0] == '3'){
				int vertex1 = 0, vertex2 = 0, vertex3 = 0;
				buffer[0] = ' ';

				sscanf(buffer, "%d %d %d", &vertex1, &vertex2, &vertex3);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex1]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex1 + 1]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex1 + 2]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex2]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex2 + 1]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex2 + 2]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex3]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex3 + 1]);
				surfaceData.vecFaceTriangles.emplace_back(surfaceData.vecPoints[3 * vertex3 + 2]);

				// Normal
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1 + 1]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1 + 2]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2 + 1]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2 + 2]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3 + 1]);
				surfaceData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3 + 2]);
			}
		}

		free(surfaceVertexXYZ);		surfaceVertexXYZ = NULL;
		free(surfaceVertexNorm);	surfaceVertexNorm = NULL;
		fclose(file);
	}
}
// Get Data From Mesh
void DataProcessing::getMeshData(pcl::PolygonMesh mesh){
	if (mesh.cloud.data.empty()) PCL_ERROR("[pcl::io::savePLYFile] Input point cloud has no data!\n");
	// number of points
	int nr_points = mesh.cloud.width * mesh.cloud.height;
	// size of points
	int point_size = mesh.cloud.data.size() / nr_points;
	// number of faces
	int nr_faces = mesh.polygons.size();
	
	std::vector<float> meshVertex1D; 
	for (std::size_t i = 0; i < nr_points; i++){
		for (std::size_t d = 0; d < mesh.cloud.fields.size(); ++d){
			// adding vertex
			if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32)
				&& (mesh.cloud.fields[d].name == "x" || mesh.cloud.fields[d].name == "y" || mesh.cloud.fields[d].name == "z")){
				float value;
				memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset], sizeof(float));
				meshVertex1D.emplace_back(value);
			}
		}
	}
	std::vector<QVector3D> meshVertex3D;
	meshVertex3D.resize(meshVertex1D.size() / 3);
	for (int i = 0,index = 0; i < meshVertex1D.size() / 3; i++){
		meshVertex3D[i] = { meshVertex1D[index], meshVertex1D[index+1], meshVertex1D[index+2] };
		index += 3;
	}
	normalize(meshVertex3D);
	meshData.clear();
	for (std::size_t i = 0; i < nr_faces; i++){
		for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++){
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].x());
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].y());
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].z());
		}
	}
}

// get normal vector of point cloud
void DataProcessing::getNormalVector(std::string pcdPath){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud) == -1)  PCL_ERROR("Could not read file\n");
	
	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;

	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud);
	search.nearestKSearch(*cloud, pcl::Indices(), 50, k_indices, k_sqr_distances);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	for (size_t i = 0; i < cloud->size(); ++i){
		pcl::Normal normal;
		ne.computePointNormal(*cloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*cloud)[i], (*cloud).sensor_origin_[0], (*cloud).sensor_origin_[1], (*cloud).sensor_origin_[2], normal.normal_x, normal.normal_y, normal.normal_z);
		normals->emplace_back(normal);
	}
	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normals);
	nr.setMaxIterations(50);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefined);
}
// Find the nearest vertex of the world coordinate point
int DataProcessing::findNearestVertex(QVector3D worldPos, std::vector<QVector3D> meshVertices) {
	int nearestVertexIndex = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < meshVertices.size(); i++) {
		float dist = (meshVertices[i] - worldPos).length();
		if (dist < minDist) {
			nearestVertexIndex = i;
			minDist = dist;
		}
	}
	return nearestVertexIndex;
}
void DataProcessing::rotateModel(QPoint& point, QMatrix4x4& model, QMatrix4x4& modelUse, QMatrix4x4& modelSave) {
	model.setToIdentity();
	float angleNow = qSqrt(qPow(point.x(), 2) + qPow(point.y(), 2)) / 5;
	model.rotate(angleNow, -point.y(), point.x(), 0.0);
	model = model * modelUse;
	;
	modelSave.setToIdentity();
	modelSave.rotate(angleNow, -point.y(), point.x(), 0.0);
	modelSave = modelSave * modelUse;
}
void DataProcessing::translateModel(QPoint& point, QMatrix4x4& model, QMatrix4x4& modelUse, QMatrix4x4& modelSave) {
	model.setToIdentity();
	model.translate((float)point.x() / 200, (float)point.y() / 200);
	model = model * modelUse;

	modelSave.setToIdentity();
	modelSave.translate((float)point.x() / 200, (float)point.y() / 200);
	modelSave = modelSave * modelUse;
}

std::vector<float> DataProcessing::getSurfaceData(std::string oriPlyPath,std::string transMeshPlyPath,std::string transMeshPcdPath, std::string finalMeshPath) {
	ply2ply(oriPlyPath, transMeshPlyPath);
	ply2pcd(transMeshPlyPath, transMeshPcdPath);
	getNormalVector(transMeshPcdPath);
	pcl::PolygonMesh mesh;
	pcl::io::loadPLYFile(transMeshPlyPath, mesh);
	writePlyData(mesh, finalMeshPath);
	loadMeshData(finalMeshPath.data());

	for (int i = 0, meshLineMarker = 0; i < surfaceData.vecFaceTriangles.size() / 3; i++) {
		if (i == 0) meshData.clear();
		meshData.emplace_back(surfaceData.vecFaceTriangles[meshLineMarker]);
		meshData.emplace_back(surfaceData.vecFaceTriangles[meshLineMarker + 1]);
		meshData.emplace_back(surfaceData.vecFaceTriangles[meshLineMarker + 2]);
		
		meshData.emplace_back(surfaceData.vecVertexNormals[meshLineMarker]);
		meshData.emplace_back(surfaceData.vecVertexNormals[meshLineMarker + 1]);
		meshData.emplace_back(surfaceData.vecVertexNormals[meshLineMarker + 2]);

		meshLineMarker += 3;
	}
	return 	meshData;
}

pcl::PolygonMesh DataProcessing::eraseMesh(pcl::PolygonMesh mesh, std::vector<int> verticesToDelete) {
	std::vector<int> polygonsToDelete;
	for (int i = 0; i < mesh.polygons.size(); i++) {
		const pcl::Vertices& polygon = mesh.polygons[i];
		bool deletePolygon = true;
		for (int j = 0; j < verticesToDelete.size(); j++) {
			if (std::find(polygon.vertices.begin(), polygon.vertices.end(), verticesToDelete[j]) == polygon.vertices.end()) {
				deletePolygon = false;
				break;
			}
		}
		if (deletePolygon) {
			polygonsToDelete.push_back(i);
		}
	}
	// Delete polygons containing the vertices to delete
	for (int i = 0; i < polygonsToDelete.size(); ++i) {
		int polygonIndex = polygonsToDelete[i];
		const pcl::Vertices& polygon = mesh.polygons[polygonIndex];

		// Remove references to this polygon from other polygons
		for (int j = 0; j < mesh.polygons.size(); ++j) {
			if (j != polygonIndex) {
				pcl::Vertices& otherPolygon = mesh.polygons[j];
				auto it = std::find(otherPolygon.vertices.begin(), otherPolygon.vertices.end(), polygon.vertices[0]);
				if (it != otherPolygon.vertices.end()) {
					otherPolygon.vertices.erase(it);
				}
			}
		}
		mesh.polygons.erase(mesh.polygons.begin() + polygonIndex);
	}
	return mesh;
}
std::vector<QVector3D> DataProcessing::convertPolygonMeshToQVector3D(const pcl::PolygonMesh& mesh) {
	std::vector<QVector3D> vertices;

	// Get the point cloud from the mesh
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	// Convert each point to a QVector3D
	for (const auto& point : cloud->points) {
		QVector3D vertex(point.x, point.y, point.z);
		vertices.push_back(vertex);
	}

	return vertices;
}
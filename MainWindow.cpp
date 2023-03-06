#include "MainWindow.h"
#include <QFileDialog>
#include <QElapsedtimer>
#include <qelapsedtimer.h>
#include <QDebug>
#include "MyGLWidget.h"
#include "Macro.h"

MainWindow::MainWindow(QWidget *parent): QMainWindow(parent){
	ui.setupUi(this);
    meshDataProc = new DataProcessing();
    surface = new SurfaceReconsturction();
    isOpenGLThreadStart = true;
    addOpengGLWidget();
  
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(gLWidgetUpdate()));
}

MainWindow::~MainWindow(){
    delete surface;
    delete myMeshGLWidget;
    delete meshDataProc;
}

// add opengl widget
void MainWindow::addOpengGLWidget(){
    myMeshGLWidget = new MyGLWidget(this);
    myMeshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myMeshGLWidget);
}

void MainWindow::gLWidgetUpdate() {
    myMeshGLWidget->update();
}
// Begin render
void MainWindow::startRendering(){
    QString fileName = "C:/Project/OpenGL-Rendering-Master-Build/originalData.txt";
    meshDataProc->loadPointData(fileName.toStdString().c_str());
    pointData3D.resize(meshDataProc->pointData.size());
    pointData3D = meshDataProc->pointData;

    auto collectDataFunc = [=]() {
        int pointLine = 0;
        while (pointLine < pointData3D.size()){
            originalPointData.emplace_back(QVector3D{ pointData3D[pointLine].x(), pointData3D[pointLine].y(), pointData3D[pointLine].z() });
            pointLine++;
            if ((originalPointData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                if (((pointLine % MESH_INCREASE_SIZE) == 0) || (pointLine >= pointData3D.size())) {
                    surface->construction(originalPointData);
         
                    std::string oriPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                    std::string transMeshPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/transMesh.ply";
                    std::string transMeshPcdPath = "C:/Project/OpenGL-Rendering-Master-Build/transMesh.pcd";
                    std::string finalMeshPath = "C:/Project/OpenGL-Rendering-Master-Build/finalMesh.ply";

                    meshDataProc->ply2ply(oriPlyPath, transMeshPlyPath);
                    meshDataProc->ply2pcd(transMeshPlyPath, transMeshPcdPath);
                    meshDataProc->getNormalVector(transMeshPcdPath);
                    pcl::PolygonMesh mesh;
                    pcl::io::loadPLYFile(transMeshPlyPath, mesh);
                    meshDataProc->writePlyData(mesh);
                    meshDataProc->loadMeshData(finalMeshPath.data());

                    for (int i = 0, meshLineMarker = 0; i < meshDataProc->surfaceData.vecFaceTriangles.size() / 3; i++) {
                        if (i == 0) meshData.clear();
                        meshData.emplace_back(meshDataProc->surfaceData.vecFaceTriangles[meshLineMarker]);
                        meshData.emplace_back(meshDataProc->surfaceData.vecFaceTriangles[meshLineMarker + 1]);
                        meshData.emplace_back(meshDataProc->surfaceData.vecFaceTriangles[meshLineMarker + 2]);
                        meshData.emplace_back(meshDataProc->surfaceData.vecVertexNormals[meshLineMarker]);
                        meshData.emplace_back(meshDataProc->surfaceData.vecVertexNormals[meshLineMarker + 1]);
                        meshData.emplace_back(meshDataProc->surfaceData.vecVertexNormals[meshLineMarker + 2]);

                        meshLineMarker += 3;
                    }
                    myMeshGLWidget->setImageData(meshData);
                    emit signal_glUpdate();
                }
            }
            if (pointLine >= pointData3D.size()){
                std::fstream fs;
                std::string txtPath = "C:/Project/OpenGL-Rendering-Master-Build/gl_PointCloud.txt";
                std::string pcdPath = "C:/Project/OpenGL-Rendering-Master-Build/gl_PointCloud.pcd";
                fs.open(txtPath,std::ios::app);
                meshData3D.clear();
                meshData3D.resize(meshDataProc->surfaceData.vecPoints.size() / 3);
                for (int i = 0; i < meshDataProc->surfaceData.vecPoints.size() / 3 ; i++){
                    meshData3D[i].setX(meshDataProc->surfaceData.vecPoints[i]);
                    meshData3D[i].setY(meshDataProc->surfaceData.vecPoints[i + 1]);
                    meshData3D[i].setZ(meshDataProc->surfaceData.vecPoints[i + 2]);
                    fs << meshData3D[i].x() << " " << meshData3D[i].y() << " " << meshData3D[i].z() << " " << std::endl;
                }
                fs.close();          
                meshDataProc->txt2pcd(txtPath, pcdPath);

                myMeshGLWidget->meshVertices.clear();
                myMeshGLWidget->meshVertices.resize(meshData3D.size());
                myMeshGLWidget->meshVertices = meshData3D;
            }
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
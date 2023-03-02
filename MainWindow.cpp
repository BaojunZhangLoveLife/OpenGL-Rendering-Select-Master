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
    fs.open("C:/Project/OpenGL-Rendering-Select-Master-Build/Release/Data/aaa.txt");
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(gLWidgetUpdate()));
}

MainWindow::~MainWindow(){
    delete surface;
    delete myMeshGLWidget;
    delete meshDataProc;
    fs.close();
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
    QString fileName = "C:/Project/OpenGL-Rendering-Select-Master/Data/originalData.txt";
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

                    for (int i = 0, meshLineMarker = 0; i < meshDataProc->surfaceModelData.vecFaceTriangles.size() / 3; i++) {
                        if (i == 0) meshData.clear();
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker]);
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker + 1]);
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker + 2]);
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker]);
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker + 1]);
                        meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker + 2]);

                        meshLineMarker += 3;
                    }
                    myMeshGLWidget->setImageData(meshData);
                    emit signal_glUpdate();
                }
            }
            if (pointLine >= pointData3D.size()){
                meshData3D.clear();
                meshData3D.resize(meshDataProc->surfaceModelData.vecPoints.size() / 3);
                for (int i = 0; i < meshDataProc->surfaceModelData.vecPoints.size() / 3 ; i++){
                    meshData3D[i].setX(meshDataProc->surfaceModelData.vecPoints[i]);
                    meshData3D[i].setY(meshDataProc->surfaceModelData.vecPoints[i + 1]);
                    meshData3D[i].setZ(meshDataProc->surfaceModelData.vecPoints[i + 2]);
                }
                myMeshGLWidget->meshVertices.clear();
                myMeshGLWidget->meshVertices.resize(meshData3D.size());
                myMeshGLWidget->meshVertices = meshData3D;
            }
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
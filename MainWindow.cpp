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
    myMeshGLWidget = new MyGLWidget(this, MeshType);
    myMeshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myMeshGLWidget);
}

void MainWindow::gLWidgetUpdate() {
    myMeshGLWidget->update();
}
// Begin render
void MainWindow::startRendering(){
    QString fileName = "C:/Project/OpenGL-Rendering-Select-Master/data/heart.txt";
    meshDataProc->LoadPointData(fileName.toStdString().c_str());
    pointData3D.resize(meshDataProc->pointData.size());
    pointData3D = meshDataProc->pointData;

    QElapsedTimer mstimer;
    mstimer.start();

    auto collectDataFunc = [=]() {
        int pointLine = 0;
        while (pointLine < pointData3D.size()){
            originalPointData.emplace_back(QVector3D{ pointData3D[pointLine].x(), pointData3D[pointLine].y(), pointData3D[pointLine].z() });
            pointLine++;
            if ((originalPointData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                if (((pointLine % MESH_INCREASE_SIZE) == 0) || (pointLine >= pointData3D.size())) {
                    surface->construction(originalPointData);
                    std::string resultPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                    std::string savePlyFilePath = "C:/Project/OpenGL-Rendering-Master-Build/savePLYFile.ply";

                    meshDataProc->MeshConvert(resultPath);
                    meshDataProc->LoadMeshData(savePlyFilePath.data());

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
                float time = (double)mstimer.nsecsElapsed() / (double)1000000;
                qDebug() << "time= " << time/1000 << "s";
            }
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
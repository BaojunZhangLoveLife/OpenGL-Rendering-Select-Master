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
    meshDataProc->loadPointData(ORIGINAL_DATA);
    pointData3D.resize(meshDataProc->pointData.size());
    pointData3D = meshDataProc->pointData;

    auto collectDataFunc = [=]() {
        int pointLine = 0;
        while (pointLine < pointData3D.size()){
            originalPointData.emplace_back(QVector3D{ pointData3D[pointLine].x(), pointData3D[pointLine].y(), pointData3D[pointLine].z()});
            pointLine++;
            // Build model and display
            if ((originalPointData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                if (((pointLine % MESH_INCREASE_SIZE) == 0) || (pointLine >= pointData3D.size())) {
                    surface->construction(originalPointData);
                    meshData = meshDataProc->getRenderData(ORI_PLY_PATH, TRANS_MESH_PLY_PATH, TRANS_MESH_PCD_PATH, FINAL_MESH_PASH);
                    myMeshGLWidget->setImageData(meshData);
                    emit signal_glUpdate();
                }
            }
            // Save the final model data that is built
            if (pointLine >= pointData3D.size()){
                std::fstream fs;
                fs.open(GL_POINTCLOUD_TXT_PATH,'w');
                meshData3D.resize(meshDataProc->surfaceData.vecPoints.size() / 3);
       
                for (int i = 0, count = 0; i < meshData3D.size(); count += 3,i++){
                    meshData3D[i].setX(meshDataProc->surfaceData.vecPoints[count]);
                    meshData3D[i].setY(meshDataProc->surfaceData.vecPoints[count + 1]);
                    meshData3D[i].setZ(meshDataProc->surfaceData.vecPoints[count + 2]);
         
                    fs << meshData3D[i].x() << " " << meshData3D[i].y() << " " << meshData3D[i].z() << " " << std::endl;
                }
                fs.close();          
                meshDataProc->txt2pcd(GL_POINTCLOUD_TXT_PATH, GL_POINTCLOUD_PCD_PATH);
                myMeshGLWidget->meshVertices.resize(meshData3D.size());
                myMeshGLWidget->meshVertices = meshData3D;
            }
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
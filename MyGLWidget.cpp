#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <glut.h>
#include <cmath>
#include <fstream>
#include <sstream> 
#include <DataProcessing.h>
#include <Macro.h>

MyGLWidget::MyGLWidget(QWidget* parent){
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
    dataProc = new DataProcessing();
    surface = new SurfaceReconsturction();
}

MyGLWidget::~MyGLWidget(){
    delete meshShader;
    //delete selectShader;
    glFunc->glDeleteVertexArrays(1, &meshVAO);
    glFunc->glDeleteBuffers(1, &meshVBO);
}

void MyGLWidget::setImageData(std::vector<float> data){
    vertices = data;
}

void MyGLWidget::setupShader() {
    meshShader->use();
    meshShader->setUniformVec3("viewPos", QVector3D(0.0f, 0.0f, 3.0f));

    meshShader->setUniformVec3("material.ambient", QVector3D(0.5f, 0.5f, 0.5f));
    meshShader->setUniformVec3("material.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
    meshShader->setUniformVec3("material.specular", QVector3D(0.5f, 0.5f, 0.5f));
    meshShader->setUniformFloat("material.shininess", 16.0f);

    meshShader->setUniformVec3("light1.ambient", QVector3D(0.2f, 0.2f, 0.2f));
    meshShader->setUniformVec3("light1.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
    meshShader->setUniformVec3("light1.specular", QVector3D(0.1f, 0.1f, 0.1f));
    meshShader->setUniformVec3("light1.direction", QVector3D(1.0f, 1.0f, 3.0f));

    meshShader->setUniformVec3("light2.ambient", QVector3D(0.2f, 0.2f, 0.2f));
    meshShader->setUniformVec3("light2.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
    meshShader->setUniformVec3("light2.specular", QVector3D(0.1f, 0.1f, 0.1f));
    meshShader->setUniformVec3("light2.direction", QVector3D(1.0f, 1.0f, -3.0f));

    meshShader->setUniformMat4("model", model);
    meshShader->setUniformMat4("view", camera->getViewMatrix());
    meshShader->setUniformMat4("proj", proj);
}
// Draw Mesh Image
void MyGLWidget::drawMesh() {
    GLuint meshVAO, meshVBO;
    glFunc->glGenVertexArrays(1, &meshVAO);
    glFunc->glGenBuffers(1, &meshVBO);
    glFunc->glBindVertexArray(meshVAO);
    glFunc->glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glFunc->glEnableVertexAttribArray(0);
    glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glFunc->glEnableVertexAttribArray(1);
    glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

    glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
    glFunc->glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
}
// initialize OpenGL
void MyGLWidget::initializeGL(){
    meshShader = new ShaderProgram(MESH_VERT, MESH_FRAG);
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    glFunc->glEnable(GL_DEPTH_TEST);
    glFunc->glEnable(GL_SELECT);
}
// paintGL
void MyGLWidget::paintGL(){
    glFunc->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    setupShader();
    drawMesh();
}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint mousePosition = event->pos();
    convertPoint(mousePosition);
    QPoint subPoint = mousePosition - pressPosition;
    if (event->buttons() & Qt::LeftButton) {
        dataProc->rotateModel(subPoint, model, modelUse, modelSave);
    }
    if (event->buttons() & Qt::RightButton) {
        dataProc->translateModel(subPoint, model, modelUse, modelSave);
    }
    update();
}

void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (isShiftPressed && (event->buttons() & Qt::LeftButton)) {
        QVector3D worldPos  = convertScreenToWorld(event->pos());
        int vertexIndex = dataProc->findNearestVertex(worldPos, meshVertices);
        if (vertexIndex != -1) {
            pcl::PointXYZ query_point;
            query_point.x = meshVertices[vertexIndex].x();
            query_point.y = meshVertices[vertexIndex].y();
            query_point.z = meshVertices[vertexIndex].z();
            pcl::PolygonMesh mesh;
            pcl::io::loadPLYFile(FINAL_MESH_PASH, mesh);

            std::vector<int> toRemove = dataProc->nearestKSearch(mesh, query_point);
            pcl::io::savePLYFile("C:/Project/OpenGL-Rendering-Master-Build/result111.ply", dataProc->eraseMesh(mesh, toRemove));
            dataProc->loadMeshData(FINAL_MESH_PASH);

            //for (auto it = toRemove.rbegin(); it != toRemove.rend(); it++){
            //    meshVertices.erase(meshVertices.begin() + *it);
            //}

            //surface->construction(meshVertices);
            //vertices = dataProc->getSurfaceData(ORI_PLY_PATH, TRANS_MESH_PLY_PATH, TRANS_MESH_PCD_PATH, FINAL_MESH_PASH);
        }
        paintGL();
    }else{
        setPressPosition(event->pos());
        modelUse = modelSave;
    }
}

void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {
    setPressPosition(event->pos());
    modelUse = modelSave;
}
void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QPoint offset = event->angleDelta();
    camera->mouseScroll(offset.y() / 20);
    update();
}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift) 
        isShiftPressed = true;
}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift) 
        isShiftPressed = false;
}


void MyGLWidget::setPressPosition(QPoint pressPos) {
    convertPoint(pressPos);
    pressPosition = pressPos;
}
void MyGLWidget::convertPoint(QPoint& oriPos) {
    oriPos.setX(oriPos.x() - this->width() / 2);
    oriPos.setY(-(oriPos.y() - this->height() / 2));
}

// Convert screen coordinates to world coordinates
QVector3D MyGLWidget::convertScreenToWorld(QPoint screenPoint) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    double modelViewMatrix[16];
    double projectionMatrix[16];
    QMatrix4x4 mVMatrix = (camera->getViewMatrix()) * model;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            modelViewMatrix[i * 4 + j] = mVMatrix(i, j);
            projectionMatrix[i * 4 + j] = proj(i, j);
        }
    }
    double worldX, worldY, worldZ;
    gluUnProject(
        screenPoint.x(), 
        height() - screenPoint.y(),
        0, 
        modelViewMatrix, 
        projectionMatrix, 
        viewport,
        &worldX,
        &worldY,
        &worldZ
    );
    return QVector3D(worldX, worldY, worldZ);
}
#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <glut.h>
MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
}

MyGLWidget::~MyGLWidget(){
    delete meshShader;
    delete selectShader;
    glFunc->glDeleteVertexArrays(1, &meshVAO);
    glFunc->glDeleteBuffers(1, &meshVBO);
}

void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::initializeShader() {
    QString qAppDir = QCoreApplication::applicationDirPath();
    QString pointVert = qAppDir + "/Shader/point.vert", pointFrag = qAppDir + "/Shader/point.frag";
    QString meshVert = qAppDir + "/Shader/mesh.vert", meshFrag = qAppDir + "/Shader/mesh.frag";

    meshShader = new ShaderProgram(meshVert.toStdString().c_str(), meshFrag.toStdString().c_str());
    selectShader = new ShaderProgram(pointVert.toStdString().c_str(), pointFrag.toStdString().c_str());
}
// initialize OpenGL
void MyGLWidget::initializeGL(){
    initializeShader();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    glFunc->glEnable(GL_DEPTH_TEST);
    glFunc->glEnable(GL_SELECT);
}
// paintGL
void MyGLWidget::paintGL(){
    GLuint meshVAO, meshVBO;
    glFunc->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glFunc->glGenVertexArrays(1, &meshVAO);
    glFunc->glGenBuffers(1, &meshVBO);
    glFunc->glBindVertexArray(meshVAO);
    glFunc->glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    glFunc->glEnableVertexAttribArray(0);
    glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glFunc->glEnableVertexAttribArray(1);
    glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
    
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

    glFunc->glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);

    if (isShiftPressed) {
        GLuint selectVAO, selectVBO;
        glFunc->glGenVertexArrays(1, &selectVAO);
        glFunc->glGenBuffers(1, &selectVBO);
        glFunc->glBindVertexArray(selectVAO);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, selectVBO);
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glFunc->glEnableVertexAttribArray(1);
        glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * selectPoints.size(), selectPoints.data(), GL_DYNAMIC_DRAW);

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
        glFunc->glPointSize(10);
        for (int i = 0; i < selectPoints.size(); i++)
        {
            meshShader->setUniformVec3("pickPosition", selectPoints[i]);
           
        }
        glFunc->glDrawArrays(GL_POINTS, 0, selectPoints.size());
    }


}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint posMouse = event->pos();
    translatePoint(posMouse);
    QPoint subPoint = posMouse - pressPosition;
    if (event->buttons() & Qt::LeftButton) {
        rotateModel(subPoint);
    }
    if (event->buttons() & Qt::RightButton) {
        translateModel(subPoint);
    }
    update();
}

void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (isShiftPressed && (event->buttons() & Qt::LeftButton)) {
        int x = event->x();
        int y = height() - event->y();
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
        double selectPoint[3];
        GLfloat winX, winY, winZ;
        winX = (float)x;
        winY = (float)y;
        gluUnProject(winX, winY, 0, modelViewMatrix, projectionMatrix, viewport, &selectPoint[0], &selectPoint[1], &selectPoint[2]);
        std::cout << "selectPoint = " << selectPoint[0] << "  " << selectPoint[1] << "  " << selectPoint[2] << std::endl;
        QVector3D selectPoint111(selectPoint[0], selectPoint[1], selectPoint[2]);
        selectPoints.push_back(selectPoint111);
        update();
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
    if (event->key() == Qt::Key_Shift)  isShiftPressed = true;
}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift)  isShiftPressed = false;
}

void MyGLWidget::setPressPosition(QPoint pressPos) {
    translatePoint(pressPos);
    pressPosition = pressPos;
}
void MyGLWidget::translatePoint(QPoint& oriPos) {
    oriPos.setX(oriPos.x() - this->width() / 2);
    oriPos.setY(-(oriPos.y() - this->height() / 2));
}
void MyGLWidget::rotateModel(QPoint point) {
    model.setToIdentity();
    GLfloat angleNow = qSqrt(qPow(point.x(), 2) + qPow(point.y(), 2)) / 5;
    model.rotate(angleNow, -point.y(), point.x(), 0.0);
    model = model * modelUse;
    ;
    modelSave.setToIdentity();
    modelSave.rotate(angleNow, -point.y(), point.x(), 0.0);
    modelSave = modelSave * modelUse;
}
void MyGLWidget::translateModel(QPoint point) {
    model.setToIdentity();
    model.translate((float)point.x() / 200, (float)point.y() / 200);
    model = model * modelUse;

    modelSave.setToIdentity();
    modelSave.translate((float)point.x() / 200, (float)point.y() / 200);
    modelSave = modelSave * modelUse;
}
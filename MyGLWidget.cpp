#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <mutex>

MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
}

MyGLWidget::~MyGLWidget(){
    delete mShader;
    glFunc->glDeleteVertexArrays(1, &meshVAO);
    glFunc->glDeleteBuffers(1, &meshVBO);
}

void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::initializeShader() {
    QString qAppDir = QCoreApplication::applicationDirPath();
    if (dataType == DataType::PointType) {
        QString pointVert = qAppDir + "/Shader/point.vert";
        QString pointFrag = qAppDir + "/Shader/point.frag";
        mShader = new ShaderProgram(pointVert.toStdString().c_str(), pointFrag.toStdString().c_str());
    }else {
        QString meshVert = qAppDir + "/Shader/mesh.vert";
        QString meshFrag = qAppDir + "/Shader/mesh.frag";
        mShader = new ShaderProgram(meshVert.toStdString().c_str(), meshFrag.toStdString().c_str());

        mShader->setUniformVec3("viewPos", QVector3D(0.0f, 0.0f, 3.0f));
        mShader->setUniformVec3("material.ambient", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformVec3("material.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
        //mShader->setUniformVec3("material.specular", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformFloat("material.shininess", 16.0f);

        mShader->setUniformVec3("light1.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        mShader->setUniformVec3("light1.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
        //mShader->setUniformVec3("light1.specular", QVector3D(0.1f, 0.1f, 0.1f));
        mShader->setUniformVec3("light1.direction", QVector3D(1.0f, 1.0f, 3.0f));

        mShader->setUniformVec3("light2.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        mShader->setUniformVec3("light2.diffuse", QVector3D(0.9f, 0.9f, 0.9f));
        //mShader->setUniformVec3("light2.specular", QVector3D(0.1f, 0.1f, 0.1f));
        mShader->setUniformVec3("light2.direction", QVector3D(1.0f, 1.0f, -3.0f));
    }
}
// initialize OpenGL
void MyGLWidget::initializeGL(){
    initializeShader();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    glFunc->glEnable(GL_DEPTH_TEST);
    glFunc->glGenVertexArrays(1, &meshVAO);
    glFunc->glBindVertexArray(meshVAO);
    glFunc->glGenBuffers(1, &meshVBO);
    glFunc->glBindBuffer(GL_ARRAY_BUFFER, meshVBO);
    if (DataType::PointType == dataType){
        glFunc->glPointSize(1.0f);
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    }else{
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glFunc->glEnableVertexAttribArray(1);
        glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    }
    auto renderFunc = [=]() {
        while (true) {
            Sleep(10);
            paintGL();
        }
    };
    std::thread renderThread(renderFunc);
    renderThread.detach();
}
// PaintGL
void MyGLWidget::paintGL(){
    if (vertices.size() == 0)   return;
    glFunc->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
    mShader->setUniformMat4("model", model);
    mShader->setUniformMat4("view", camera->getViewMatrix());
    mShader->setUniformMat4("proj", proj);
    DataType::PointType == dataType ? glFunc->glDrawArrays(GL_POINTS, 0, vertices.size() / 3):
        glFunc->glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
    update();
}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint p_ab = event->pos();
    translate_point(p_ab);
    QPoint subPoint = p_ab - pressPosition;

    model.setToIdentity();
    modelSave.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
            // Rotate the 3D image
            GLfloat angleNow = qSqrt(qPow(subPoint.x(), 2) + qPow(subPoint.y(), 2)) / 5;
            model.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
            model = model * modelUse;

            modelSave.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
            modelSave = modelSave * modelUse;
    }
    if (event->buttons() & Qt::RightButton) {
        model.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        model = model * modelUse;

        modelSave.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        modelSave = modelSave * modelUse;
    }
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (isShiftPressed) {
        if (event->buttons() & Qt::LeftButton) {
            double objX, objY, objZ;
            GLint viewport[4];
            double modelViewMatrix[16];
            double projectionMatrix[16];

            glFunc->glGetIntegerv(GL_VIEWPORT, viewport);
            glFunc->glGetDoublev(GL_MODELVIEW_MATRIX, modelViewMatrix);
            glFunc->glGetDoublev(GL_PROJECTION_MATRIX, projectionMatrix);

            double winX = event->x();
            double winY = height() - event->y();
            double winZ = 0.1;

            gluUnProject(winX, winY, winZ, modelViewMatrix, projectionMatrix, viewport, &objX, &objY, &objZ);

            gluPickMatrix(event->x(), height() - event->y(), 5, 5, viewport);
        }
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
}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift){
        isShiftPressed = true;
    }
}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift){
        isShiftPressed = false;
    }
}

// Set the position where the mouse is pressed
void MyGLWidget::setPressPosition(QPoint p_ab) {
    translate_point(p_ab);
    pressPosition = p_ab;
}
// Move the origin to the center of the screen.
void MyGLWidget::translate_point(QPoint& p_ab) {
    int x = p_ab.x() - this->width() / 2;
    int y = -(p_ab.y() - this->height() / 2);
    p_ab = {x,y};
}
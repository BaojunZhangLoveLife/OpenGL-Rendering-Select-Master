#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <mutex>

MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    //qDebug() << "MyGLWidget.proj = " << proj;
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

    QString pointVert = qAppDir + "/Shader/point.vert";
    QString pointFrag = qAppDir + "/Shader/point.frag";
    selectShader = new ShaderProgram(pointVert.toStdString().c_str(), pointFrag.toStdString().c_str());

    QString meshVert = qAppDir + "/Shader/mesh.vert";
    QString meshFrag = qAppDir + "/Shader/mesh.frag";
    meshShader = new ShaderProgram(meshVert.toStdString().c_str(), meshFrag.toStdString().c_str());
}
// initialize OpenGL
void MyGLWidget::initializeGL(){
    initializeShader();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
}
// PaintGL
void MyGLWidget::paintGL(){
    if (vertices.size() == 0)   return;
    glFunc->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glFunc->glEnable(GL_DEPTH_TEST);
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
    update();
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

            gluUnProject(winX, winY, winZ, (double*)modelViewMatrix, (double*)projectionMatrix, viewport, &objX, &objY, &objZ);

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
    update();
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
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
    selectShader = new ShaderProgram(pointVert.toStdString().c_str(), pointFrag.toStdString().c_str());
    meshShader = new ShaderProgram(meshVert.toStdString().c_str(), meshFrag.toStdString().c_str());
}
// initialize OpenGL
void MyGLWidget::initializeGL(){
    initializeShader();
    glFunc->glEnable();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
}
// paintGL
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
    QPoint posMouse = event->pos();
    translatePoint(posMouse);
    QPoint subPoint = posMouse - pressPosition;

    if (event->buttons() & Qt::LeftButton) {
        model.setToIdentity();
        GLfloat angleNow = qSqrt(qPow(subPoint.x(), 2) + qPow(subPoint.y(), 2)) / 5;
        model.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
        model = model * modelUse;

        modelSave.setToIdentity();
        modelSave.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
        modelSave = modelSave * modelUse;
    }
    if (event->buttons() & Qt::RightButton) {
        model.setToIdentity();
        model.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        model = model * modelUse;

        modelSave.setToIdentity();
        modelSave.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        modelSave = modelSave * modelUse;
    }
    update();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (isShiftPressed) {
        if (event->buttons() & Qt::LeftButton) {
            GLint viewport[4];
            glGetIntegerv(GL_VIEWPORT, viewport);
            const int BUFSIZE = 512;
            GLuint selectBuf[BUFSIZE];
            glSelectBuffer(BUFSIZE, selectBuf);
            glRenderMode(GL_SELECT);
            glInitNames();
            glPushName(0);
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            gluPickMatrix(event->x(), viewport[3] - event->y(), 5.0, 5.0, viewport);
            glMultMatrixf(proj.constData());

  

            double objX, objY, objZ;
   
            double modelViewMatrix[16];
            double projectionMatrix[16];
            QMatrix4x4 mVMatrix = (camera->getViewMatrix()) * model;
            double winX = event->x();
            double winY = height() - event->y();
            double winZ = 1;


            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();
            glMultMatrixf(mVMatrix.constData());

            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    projectionMatrix[i * 4 + j] = proj(i, j);
                    modelViewMatrix[i * 4 + j] = mVMatrix(i, j);
                }
            }
            gluUnProject(winX, winY, winZ, modelViewMatrix, projectionMatrix, viewport, &objX, &objY, &objZ);
            std::cout << "objX = " << objX << "\t" << "objY = " << objY << "\t" << "objZ = " << objZ << std::endl
                << "----------------------------------------------------" << std::endl;
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

void MyGLWidget::setPressPosition(QPoint pressPos) {
    translatePoint(pressPos);
    pressPosition = pressPos;
}
void MyGLWidget::translatePoint(QPoint& oriPos) {
    int x = oriPos.x() - this->width() / 2;
    int y = -(oriPos.y() - this->height() / 2);
    oriPos = {x,y};
}
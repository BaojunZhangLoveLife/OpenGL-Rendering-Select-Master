#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
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
    gl = QOpenGLContext::currentContext()->extraFunctions();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    glFunc->glEnable(GL_DEPTH_TEST);
    glFunc->glEnable(GL_SELECT);
}
// paintGL
void MyGLWidget::paintGL(){
    GLuint meshVAO, meshVBO;
    GLuint selectVAO, selectVBO;
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
    glInitNames();
    glPushName(0);
    glLoadName(1);
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
        ;
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
    if (isShiftPressed && (event->buttons() & Qt::LeftButton)) {
        //std::fill(selectBuffer.begin(), selectBuffer.end(), 0);
        //glSelectBuffer(selectBufferSize, &selectBuffer[0]);
        //// Draw for selection buffer
        //glRenderMode(GL_SELECT);

        //int viewport[4];
        //glGetIntegerv(GL_VIEWPORT, viewport);
        //gluPickMatrix(event->x(), height() - event->y(), 1, 1, viewport);
        ////proj.setToIdentity();
        ////proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
        //paintGL();

        //int hits = glRenderMode(GL_RENDER);
        //printf("%d hits\n", hits);
        //if (hits > 0) {
        //    int id = 0;
        //    for (int i = 0; i < hits; i++) {
        //        printf("Level: %u\n", selectBuffer[id + 0]);
        //        printf("Min: %f\n", (double)selectBuffer[id + 1] / UINT_MAX);
        //        printf("Max: %f\n", (double)selectBuffer[id + 2] / UINT_MAX);
        //        printf("ID: %u\n", selectBuffer[id + 3]);
        //        id += 4;
        //    }
        //}

    
            // 获取鼠标位置


            // 渲染到帧缓冲区中
            gl->glBindFramebuffer(GL_FRAMEBUFFER_EXT, 0);
           
            glViewport(0, 0, width(), height());
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            QPoint pos = event->pos();
            // 读取深度缓冲区中的深度值
            GLfloat depth;
            glReadPixels(pos.x(), height() - pos.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

            // 在渲染到屏幕之前，需要将帧缓冲区绑定到默认的帧缓冲区上
            gl->glBindFramebuffer(GL_FRAMEBUFFER, defaultFramebufferObject());

            // 检查选定的对象并执行相应的操作
            if (depth < 1.0) {
                // 选定了一个网格对象
                std::cout << "可以" << std::endl;
            }
            else {
                // 没有选中任何网格对象
                std::cout << "不可以" << std::endl;
            }

            // 渲染到屏幕上
            update();
        

 /*       double p1[3], p2[3];
        double modelViewMatrix[16];
        double projectionMatrix[16];
        QMatrix4x4 mVMatrix = (camera->getViewMatrix()) * model;
        double pickingX = event->x();
        double pickingY = height() - event->y();


        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                modelViewMatrix[i * 4 + j] = mVMatrix(i, j);
                projectionMatrix[i * 4 + j] = proj(i, j);
            }
        }
        gluUnProject(pickingX, pickingY, 0, modelViewMatrix, projectionMatrix, viewport, &p1[0], &p1[1], &p1[2]);
        std::cout << "p1[0] = " << p1[0] << "\t" << "p1[1] = " << p1[1] << "\t" << "p1[2] = " << p1[2] << std::endl
            << "----------------------------------------------------" << std::endl;
        gluPickMatrix(event->x(), height() - event->y(), 5, 5, viewport);*/

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
    oriPos.setX(x);
    oriPos.setY(y);
}
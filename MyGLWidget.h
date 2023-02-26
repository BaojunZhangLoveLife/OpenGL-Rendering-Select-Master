#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H
#include <QOpenGLWidget>
#include <QtMath>
#include <QKeyEvent>
#include "ShaderProgram.h"
#include "camera.h"
#include <QOpenGLExtraFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLFunctions>
#include <gl/GLU.h>

enum DataType{
    PointType,
    MeshType
};

class MyGLWidget:public QOpenGLWidget{
    Q_OBJECT
public:
    MyGLWidget(QWidget* parent, int DT);
    ~MyGLWidget();
    void setImageData(std::vector<GLfloat> data);
protected:
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override; 
    void initializeShader();
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void setPressPosition(QPoint p_ab);
    void translatePoint(QPoint& p_ab);
    void rotateModel(QPoint point);
    void translateModel(QPoint point);

private:
    ShaderProgram* meshShader;
    ShaderProgram* selectShader;
    QMatrix4x4 model;
    QMatrix4x4 modelUse;
    QMatrix4x4 modelSave;
    Camera* camera;
    QMatrix4x4 proj;
    std::vector<QVector3D> selectPoints;

    QOpenGLFunctions_4_5_Core* glFunc;

    std::vector<GLfloat> vertices;
    int dataType;
    QPoint pressPosition; 
    GLuint meshVAO, meshVBO;
    GLuint selectVAO, selectVBO;
    bool isShiftPressed = false;
    static const int selectBufferSize = 100;
    std::vector<uint> selectBuffer = std::vector<uint>(selectBufferSize);
};

#endif 
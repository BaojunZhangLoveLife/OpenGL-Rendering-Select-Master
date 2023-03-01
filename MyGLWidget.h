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
    MyGLWidget(QWidget* parent);
    ~MyGLWidget();
    void setImageData(std::vector<GLfloat> data);
    // Mesh vertices that saved
    std::vector<QVector3D> meshVertices;
protected:
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override; 
    /* Mouse Event */
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    /* Key Event */
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    // 
    void setPressPosition(QPoint p_ab);
    void translatePoint(QPoint& p_ab);
    // 
    void rotateModel(QPoint point);
    void translateModel(QPoint point);
    // Draw Mesh Image
    void drawMesh();
    // Setup shader
    void setupShader();
    // Convert screen coordinates to world coordinates
    QVector3D convertScreenToWorld(QPoint point);
    // Find the nearest vertex of the world coordinate point
    int findNearestVertex(QVector3D worldPos);
private:
    ShaderProgram* meshShader;
    ShaderProgram* selectShader;
    QMatrix4x4 model;
    QMatrix4x4 modelUse;
    QMatrix4x4 modelSave;
    Camera* camera;
    QMatrix4x4 proj;

    QOpenGLFunctions_4_5_Core* glFunc;
    std::vector<GLfloat> vertices;

    QPoint pressPosition; 
    GLuint meshVAO, meshVBO;
    GLuint selectVAO, selectVBO;
    bool isShiftPressed = false;
};

#endif 
#pragma once
#include <QMainWindow>
#include "ui_MainWindow.h"
#include <QTimer>
#include "MyGLWidget.h"
#include "DataProcessing.h"
#include <surfacereconsturction.h>
class MainWindow : public QMainWindow{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

public slots:
	void chooseFile();
	void startRendering();
	void gLWidgetUpdate();
signals:
	void signal_glUpdate();

private:
	Ui::MainWindowClass ui;
	void addOpengGLWidget();

	MyGLWidget* myMeshGLWidget;

	SurfaceReconsturction* surface;
	DataProcessing* meshDataProc;

	std::vector<GLfloat> meshData;

	std::vector<QVector3D> originalPointData;
	std::vector<QVector3D>	pointData3D;
	bool isOpenGLThreadStart;
};	

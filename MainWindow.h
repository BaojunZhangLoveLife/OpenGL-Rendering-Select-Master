#pragma once
#include <QMainWindow>
#include "ui_MainWindow.h"
#include "MyGLWidget.h"
#include "DataProcessing.h"
#include "surfacereconsturction.h"
#include <fstream>
class MainWindow : public QMainWindow{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();

public slots:
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
	std::fstream fs;
	std::vector<QVector3D> originalPointData;
	std::vector<QVector3D>	pointData3D;
	bool isOpenGLThreadStart;
};	

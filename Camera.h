#pragma once

#include <QVector3D>
class Camera{
public:
	Camera(
		QVector3D position = QVector3D(0.0f, 0.0f, 3.0f),
		QVector3D front = QVector3D(0.0, 0.0, 0.0),
		QVector3D up = QVector3D(0.0f, 1.0f, 0.0f)
	);

	QMatrix4x4 getViewMatrix();
	void mouseScroll(float yoffset);

	// Camera Attributes
	QVector3D Position;
	QVector3D Front;
	QVector3D WorldUp;

	float scrollSensitivity;
};


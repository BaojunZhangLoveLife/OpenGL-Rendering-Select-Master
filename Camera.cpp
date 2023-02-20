#include "camera.h"
#include <QMatrix4x4>
Camera::Camera(QVector3D position,QVector3D front,QVector3D up) {
	this->scrollSensitivity = 0.01f;
	this->Position = position;
	this->WorldUp = up;
	this->Front = front;
}

QMatrix4x4 Camera::getViewMatrix(){
	QMatrix4x4 matrix;
	matrix.lookAt(Position, Front, WorldUp);
	return matrix;
}
void Camera::mouseScroll(float yoffset){
	Position.setZ(Position.z() + yoffset * scrollSensitivity);
}
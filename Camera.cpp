#include "camera.h"
#include <QMatrix4x4>
QMatrix4x4 Camera::getViewMatrix(){
	QMatrix4x4 matrix;
	matrix.lookAt(Position, Front, WorldUp);
	return matrix;
}
void Camera::mouseScroll(float yoffset){
	yoffset *= scrollSensitivity;
	if (Position.z() < 0.00001f){
		Position.setZ(0.00001f);
	}else{
		Position.setZ(Position.z() + yoffset);
	}
}
#pragma once
#include <QOpenGLShaderProgram>

class ShaderProgram{
public:
	ShaderProgram(const char* vertexPath, const char* fragmentPath);
	~ShaderProgram();
	void use();
	void setUniformInt(const char* name, const int value)			const;
	void setUniformFloat(const char* name, const GLfloat value)		const;
	void setUniformMat4(const char* name, const QMatrix4x4 trans)	const;
	void setUniformMat3(const char* name, const QMatrix3x3 trans)	const;
	void setUniformVec4(const char* name, const QVector4D value)	const;
	void setUniformVec3(const char* name, const QVector3D value)	const;

private:
	QOpenGLShaderProgram* ID;
};
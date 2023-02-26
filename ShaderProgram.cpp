#include "ShaderProgram.h"
#include <QOpenGLShader>
#include <string>
#include <fstream>
#include <sstream>
#include <iostream>

ShaderProgram::ShaderProgram(const char* vertexPath, const char* fragmentPath){
	std::string vertexCode, fragmentCode;
	std::ifstream vShaderFile, fShaderFile;
	std::stringstream vShaderStream, fShaderStream;

	vShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	fShaderFile.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	try{
		vShaderFile.open(vertexPath);
		fShaderFile.open(fragmentPath);

		vShaderStream << vShaderFile.rdbuf();
		fShaderStream << fShaderFile.rdbuf();

		vShaderFile.close();
		fShaderFile.close();

		vertexCode = vShaderStream.str();
		fragmentCode = fShaderStream.str();
	}
	catch (std::ifstream::failure e){
		std::cout << "ERROR::SHADER::FIEL_NOT_SUCCESSFULLY_READ" << std::endl;
	}
	const char* vShaderCode = vertexCode.c_str();
	const char* fShaderCode = fragmentCode.c_str();

	QOpenGLShader verShader(QOpenGLShader::Vertex);
	QOpenGLShader fraShader(QOpenGLShader::Fragment);

	verShader.compileSourceCode(vShaderCode);
	fraShader.compileSourceCode(fShaderCode);

	ID = new QOpenGLShaderProgram();
	ID->addShader(&verShader);
	ID->addShader(&fraShader);
	
	ID->link();
}
void ShaderProgram::use(){
	ID->bind();
}
ShaderProgram::~ShaderProgram(){
	delete ID;
}
void ShaderProgram::setUniformMat4(const char* name, const QMatrix4x4 trans) const{
	ID->setUniformValue(name, trans);
}
void ShaderProgram::setUniformMat3(const char* name, const QMatrix3x3 trans) const{
	ID->setUniformValue(name, trans);
}
void ShaderProgram::setUniformFloat(const char* name, const GLfloat value) const{
	ID->setUniformValue(name, value);
}
void ShaderProgram::setUniformInt(const char* name, const int value) const{
	ID->setUniformValue(name, value);
}
void ShaderProgram::setUniformVec4(const char* name, const QVector4D value) const{
	ID->setUniformValue(name, value);
}
void ShaderProgram::setUniformVec3(const char* name, const QVector3D value) const{
	ID->setUniformValue(name, value);
}
void ShaderProgram::setUniformPnt3(const char* name, const QPoint value)	const {
	ID->setUniformValue(name, value);
}
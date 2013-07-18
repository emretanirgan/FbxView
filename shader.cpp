#include "shader.h"
#include <string.h>
#include <iostream>
#include <stdlib.h>

using namespace std;

static char* textFileRead(const char *fileName) {
	char* text;
    
	if (fileName != NULL) {
        FILE *file = fopen(fileName, "rt");
        
		if (file != NULL) {
            fseek(file, 0, SEEK_END);
            int count = ftell(file);
            rewind(file);
            
			if (count > 0) {
				text = (char*)malloc(sizeof(char) * (count + 1));
				count = fread(text, sizeof(char), count, file);
				text[count] = '\0';
			}
			fclose(file);
		}
	}
	return text;
}

Shader::Shader() {
    
}

Shader::Shader(const char *vsFile, const char *fsFile) 
{
    init(vsFile, fsFile);
}

Shader::~Shader() 
{
	glDetachShader(shader_id, shader_fp);
	glDetachShader(shader_id, shader_vp);
    
	glDeleteShader(shader_fp);
	glDeleteShader(shader_vp);
	glDeleteProgram(shader_id);
}

void Shader::printShaderInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetShaderiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

    if (infologLength > 0)
    {
        infoLog = (char *)malloc(infologLength);
        glGetShaderInfoLog(obj, infologLength, &charsWritten, infoLog);
	printf("%s\n",infoLog);
        free(infoLog);
    }
}

void Shader::printProgramInfoLog(GLuint obj)
{
    int infologLength = 0;
    int charsWritten  = 0;
    char *infoLog;

    glGetProgramiv(obj, GL_INFO_LOG_LENGTH,&infologLength);

    if (infologLength > 0)
    {
        infoLog = (char *)malloc(infologLength);
        glGetProgramInfoLog(obj, infologLength, &charsWritten, infoLog);
	printf("%s\n",infoLog);
        free(infoLog);
    }
}

void Shader::init(const char *vsFile, const char *fsFile) 
{
	shader_vp = glCreateShader(GL_VERTEX_SHADER);
	shader_fp = glCreateShader(GL_FRAGMENT_SHADER);
    
	char* vsText = textFileRead(vsFile);
	char* fsText = textFileRead(fsFile);	
    
    if (vsText == NULL || fsText == NULL) {
        cerr << "Either vertex shader or fragment shader file not found." << endl;
        return;
    }
    

	const char * vv = vsText;
	const char * ff = fsText;

	glShaderSource(shader_vp, 1, &vv, 0);
	glShaderSource(shader_fp, 1, &ff, 0);
    
    free(vsText);free(fsText);

	glCompileShader(shader_vp);
    printShaderInfoLog(shader_vp);

	glCompileShader(shader_fp);
    printShaderInfoLog(shader_fp);
    
	shader_id = glCreateProgram();
	glAttachShader(shader_id, shader_fp);
	glAttachShader(shader_id, shader_vp);
	glLinkProgram(shader_id);
    printProgramInfoLog(shader_id);
}

unsigned int Shader::id() {
	return shader_id;
}

void Shader::bind() {
	glUseProgram(shader_id);
}

void Shader::unbind() {
	glUseProgram(0);
}

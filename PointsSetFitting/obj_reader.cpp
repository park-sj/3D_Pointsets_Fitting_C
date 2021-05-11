#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <iostream>
#include "obj_reader.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>

using namespace std;

objReader::objReader()
{
    m = NULL;
    nVertex = 0;
    nFaces = 0;
    size = 0;
}

void objReader::objLoadFile(char* filename)
{
    {
        size_t bytes = 0; // 메모리 크기를 리턴하는 sizeof() 함수의 리턴 형
        FILE* file = fopen(filename, "rt");

        if(file != NULL){
            fseek(file, 0, SEEK_END); //파일 포인터를 파일 끝으로 이동시킴
            size_t end = ftell(file); //파일 포인터의 현재 위치를 얻음
            fseek(file, 0, SEEK_SET);

            m = (char*) malloc(end); //파일 크기 만큼 메모리 할당
            bytes = fread(m, sizeof(char), end, file);

            fclose(file);
        }

        size = bytes;

    }
}

void objReader::objLoadModel()
{
    char* p = NULL, * e = NULL;
    bool start = true;
    char mtl[50];
    p = m;
    e = m + size;

    while(p != e){
        if(memcmp(p, "v", 1) == 0) nVertex++;
        else if(memcmp(p, "f", 1) ==0) nFaces++;

        while(*p++ != (char) 0x0A); //0x0A 줄 바꿈
    }
    
    vertexArray = (fvector*) malloc(sizeof(fvector) * nVertex);
    faceArray = (face*) malloc(sizeof(face)* nFaces);

    p = m;
    int nV = 0, nF =0;

    v = Eigen::MatrixXf(nVertex, 3);
    f = Eigen::MatrixXf(nFaces, 3);

    while(p != e){
        if(memcmp(p, "v", 1)==0){
            sscanf(p, "v %lf %lf %lf", &vertexArray[nV].x, &vertexArray[nV].y, &vertexArray[nV].z);
            v(nV, 0) = vertexArray[nV].x;
            v(nV, 1) = vertexArray[nV].y;
            v(nV, 2) = vertexArray[nV].z;
            nV++;
        }
        else if(memcmp(p, "f", 1)==0){
            sscanf(p, "f %d %d %d", &faceArray[nF].vertex[0], &faceArray[nF].vertex[1], &faceArray[nF].vertex[2]); 
            strcpy(faceArray[nF].material, mtl);
            f(nV, 0) = faceArray[nV].vertex[0];
            f(nV, 1) = faceArray[nV].vertex[1];
            f(nV, 2) = faceArray[nV].vertex[2];
            nF++;
        }
        while(*p++ != (char) 0x0A);
    }
    
}
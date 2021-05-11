#ifndef OBJ_READER_H
#define OBJ_READER_H
#include <stdlib.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <vector>

using namespace Eigen;

typedef struct {
        double x, y, z;
} fvector;

typedef struct {
        int vertex[3];
        char material[50];
} face;

class objReader
{
public:
        objReader();
        void objLoadModel();
        void objLoadFile(char* filename);
        char* m;

        size_t size;
        fvector* vertexArray;
        face* faceArray;
        int nVertex, nFaces;

        Eigen::MatrixXf v;
        Eigen::MatrixXf f;

};

#endif
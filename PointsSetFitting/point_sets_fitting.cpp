#include <stdlib.h>
#include <stdio.h>
#include <memory.h>
#include <iostream>
#include <typeinfo>
#include "obj_reader.h"
#include "point_sets_fitting.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/FFT>
#include <tuple>

using namespace std;
using namespace Eigen;

point_sets_fitting::point_sets_fitting()
{
    h_3x3 = Eigen::Matrix3f::Zero();

    centered_template, centered_template = Eigen::MatrixXf::Zero(3,3);
    translation_template, translation_target = Eigen::VectorXf::Zero(3,1);
    u_orth, vh_orth, lsq_rotation = Eigen::MatrixXf::Zero(3,3);
    rigid_transformation = Eigen::MatrixXf::Zero(4,4);
    det = 0;
}

tuple<Eigen::MatrixXf, float> point_sets_fitting::Fitting(objReader set_a, objReader set_b)
{
    // Computes the rigid transformation between two sets of corresponding points.
    // :param set_a: Point set A
    // :param set_b: Point set B
    // :return: (Rigid transformation, fitting error)

    if (set_a.nVertex != set_b.nVertex)
    {
        printf("Mismatching point set sizes\n");
        printf("size of set_a = %d\n", set_a.nVertex);
        printf("size of set_b = %d\n", set_b.nVertex);
        exit(0);
    }

    // move point sets to center

    move_point_set_to_center(set_a, centered_template, translation_template);
    move_point_set_to_center(set_b, centered_target, translation_target);


    // compute matix h_3x3

    for (int count=0; count<set_a.nVertex; count++)
    {
        Eigen::MatrixXf pnt_a = centered_template.row(count).adjoint();
        Eigen::MatrixXf pnt_b = centered_target.row(count);
        h_3x3 = h_3x3 +(pnt_a * pnt_b);
    } 

    Eigen::JacobiSVD<Eigen::MatrixXf> svd(h_3x3, Eigen::ComputeFullU | Eigen::ComputeFullV);

    u_orth = svd.matrixU();
    vh_orth = svd.matrixV();

    lsq_rotation = u_orth * vh_orth.adjoint();


    // fix rotation if necessary -> reflections

    det = lsq_rotation.determinant();

    if (det < 0)
    {
        vh_orth.row(2) = vh_orth.row(2) * -1;
        lsq_rotation = vh_orth.adjoint() * u_orth.adjoint();
        det = lsq_rotation.determinant();
    }

    try
    {
        if(det!=1.0)
        {
            throw string("Invalid rotation matrix determinant: %.6f", det);
        }
    }
    catch(string error)
    {
        cout << error << endl;
    }

    // compose a 4x4 matrix of rotation and translation
    rigid_transformation.block(0, 0, 3, 3) = lsq_rotation;
    rigid_transformation.block(0, 3, 3, 1) = translation_target - (lsq_rotation * translation_template);

    return tuple<Eigen::MatrixXf, float>(rigid_transformation, compute_fitting_error(set_a, set_b, rigid_transformation));
}

void point_sets_fitting::move_point_set_to_center(objReader point_set, Eigen::MatrixXf& centered_set, Eigen::VectorXf& translation)
{
    // Move point set to its center

    translation = compute_point_set_center(point_set);
    centered_set = point_set.v.rowwise() - translation.adjoint();
}

Eigen::Vector3f point_sets_fitting::compute_point_set_center(objReader point_set)
{
    // Computes the center of a point set

    Eigen::Vector3f center = point_set.v.colwise().sum() * (1.0 / point_set.v.rows());

    return center;
}

float point_sets_fitting::compute_fitting_error(objReader set_a, objReader set_b, Eigen::MatrixXf transformation)
{
    // return: Transformation error

    Eigen::MatrixXf homogeneous_set_a = to_homogeneous_repr(set_a.v).adjoint();
    Eigen::MatrixXf homogeneous_set_b = to_homogeneous_repr(set_b.v).adjoint();

    Eigen::MatrixXf diff_set_b = transformation * homogeneous_set_a - homogeneous_set_b;

    return diff_set_b.squaredNorm();

}

Eigen::MatrixXf point_sets_fitting::to_homogeneous_repr(Eigen::MatrixXf points)
{
    // Adds the homogeneous 4th line

    Eigen::MatrixXf pnt_h = Eigen::MatrixXf::Ones(points.rows(), points.cols() + 1);
    pnt_h.block(0, 0, points.rows(), points.cols()) = points;
    
    return pnt_h;
}

// tuple<float> move_point_set_to_center(vector<float> point_set);
// vector<float> to_homogeneous_repr(vector<float> points);
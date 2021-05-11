#ifndef POINT_SETS_FITTING_H
#define POINT_SETS_FITTING_H
#include "obj_reader.h"
#include <tuple>

using namespace std;


class point_sets_fitting
{
public:
    point_sets_fitting();
    tuple<Eigen::MatrixXf, float> Fitting(objReader set_a, objReader set_b);
    void move_point_set_to_center(objReader point_set, Eigen::MatrixXf& centered_set, Eigen::VectorXf& translation);
    // tuple<Eigen::Matrix3f, Eigen::Vector3f> move_point_set_to_center(objReader point_set);
    Eigen::Vector3f compute_point_set_center(objReader point_set);
    float compute_fitting_error(objReader set_a, objReader set_b, Eigen::MatrixXf transformation);
    Eigen::MatrixXf to_homogeneous_repr(Eigen::MatrixXf points);

    // bool __isclose(float a_val, float b_val, float rel_tol=1e-09, float abs_tol=0.0);
    // vector<float> to_homogeneous_repr(vector<float> points);

    Eigen::Matrix3f h_3x3;
    Eigen::MatrixXf centered_template, centered_target;
    Eigen::VectorXf translation_template, translation_target;
    Eigen::MatrixXf u_orth, vh_orth, lsq_rotation;
    float det;
    Eigen::MatrixXf rigid_transformation;

};

#endif
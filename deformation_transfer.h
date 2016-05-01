#pragma once

#include "ofApp.h"

using namespace Eigen;

void Deformation_Transfer(Mesh* mesh1, Mesh* mesh2, Mesh* mesh3, EigenMatrixXs& x);
void Source_Transformation(Mesh* mesh1, Mesh* mesh2, VectorXd& f_x, VectorXd& f_y, VectorXd& f_z);
void SystemMatrix(Mesh* mesh3, Eigen::SparseMatrix<double>& a);
void QRFactorize(const MatrixXd& a, MatrixXd& q, MatrixXd& r);
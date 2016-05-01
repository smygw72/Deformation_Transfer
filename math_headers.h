#ifndef _MATH_HEADERS_H_
#define _MATH_HEADERS_H_

// definitions
#define HIGH_PRECISION
//#define OPENMP // omp



// single or double presicion
#ifdef HIGH_PRECISION
    typedef double ScalarType;
    #define TW_TYPE_SCALAR_TYPE TW_TYPE_DOUBLE
#else
    typedef float ScalarType;
    #define TW_TYPE_SCALAR_TYPE TW_TYPE_FLOAT
#endif

// small number and large number
#ifdef HIGH_PRECISION
    #define EPSILON 1e-15
#else
    #define EPSILON 1e-6
#endif

#define LARGER_EPSILON 1e-6

// eigen
#include "Eigen/Core"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <unsupported/Eigen/SparseExtra>
#include <unsupported/Eigen/KroneckerProduct>

// global header
//#include "global_headers.h"

// glm
//#include "glm.hpp"
//#include "gtc\matrix_transform.hpp"

// omp
//#include <omp.h>

// eigen vectors and matrices
typedef int IndexType;
typedef unsigned int uint;
typedef Eigen::Matrix<ScalarType, 2, 2, 0, 2 ,2> EigenMatrix2;
typedef Eigen::Matrix<ScalarType, 3, 3, 0, 3 ,3> EigenMatrix3;
typedef Eigen::Matrix<ScalarType, 4, 4, 0, 4 ,4> EigenMatrix4;
typedef Eigen::Matrix<ScalarType, 2, 1, 0, 2 ,1> EigenVector2;
typedef Eigen::Matrix<ScalarType, 3, 1, 0, 3 ,1> EigenVector3;
typedef Eigen::Matrix<ScalarType, 4, 1, 0, 4 ,1> EigenVector4;
typedef Eigen::Matrix<ScalarType, 9, 1, 0, 9, 1> EigenVector9;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 1> EigenVectorX;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, 3> EigenMatrix3X;
typedef Eigen::Matrix<ScalarType, Eigen::Dynamic, Eigen::Dynamic> EigenMatrixXs;
typedef Eigen::Matrix<IndexType, Eigen::Dynamic, Eigen::Dynamic>  EigenMatrixXi;
typedef Eigen::Quaternion<ScalarType> EigenQuaternion;
typedef Eigen::SparseMatrix<ScalarType>      SparseMatrix;
typedef Eigen::Triplet<ScalarType,IndexType> SparseMatrixTriplet;

// omp
//#define OPENMP

// eigen vector accessor
#define block_vector(a) block<3,1>(3*(a), 0)
#define block_matrix(a) block<9,1>(9*(a), 0)
#define block_quaternion(a) block<4,1>(4*(a), 0)

// vec 2 mat, mat 2 vec
EigenMatrix3 MatVec2Mat(const EigenVector9& vector9);
EigenVector9 Mat2MatVec(const EigenMatrix3& matrix3);

EigenVector4 Mat2QuatVec(const EigenMatrix3& matrix);
EigenVector4 MatVec2QuatVec(const EigenVector9& matrix);

// eigen 2 glm, glm 2 eigen
//glm::vec3 Eigen2GLM(const EigenVector3& eigen_vector);
//EigenVector3 GLM2Eigen(const glm::vec3& glm_vector);

void Matrix3x3x3x3MultiplyMatrix3x3(const EigenMatrix3 lhs[3][3], const EigenMatrix3 &rhs, EigenMatrix3 output[3][3]); 

// solver
void factorizeDirectSolverLLT(const SparseMatrix& A, Eigen::SimplicialLLT<SparseMatrix, Eigen::Upper>& lltSolver, char* warning_msg = ""); // factorize matrix A using LLT decomposition
void factorizeDirectSolverLDLT(const SparseMatrix& A, Eigen::SimplicialLDLT<SparseMatrix, Eigen::Upper>& ldltSolver, char* warning_msg = ""); // factorize matrix A using LDLT decomposition

// utility
void computeTetBarycenter(const EigenMatrixXs& V, const EigenMatrixXs& T, EigenVectorX& barycenter);
static ScalarType ReciprocalSqrt(const ScalarType x){ return 1.0/sqrt(x); }
#endif
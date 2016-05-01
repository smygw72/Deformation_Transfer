#include "deformation_transfer.h"

using namespace Eigen;

void Deformation_Transfer(Mesh* mesh1,Mesh* mesh2,Mesh* mesh3,EigenMatrixXs& x) {

	int triangle_num = mesh1->getTriNum();	// Number of mesh
	int vertex_num = mesh1->getVertNum();	// Number of vertex
	x = MatrixXd::Zero(vertex_num, 3);	// Matrix for deformed target vertex
	VectorXd F_x(3 * triangle_num), F_y(3 * triangle_num), F_z(3 * triangle_num);	// Source deformation matrix
	Eigen::SparseMatrix<double> A(3 * triangle_num, vertex_num);	// System matrix

	Source_Transformation(mesh1, mesh2, F_x, F_y, F_z);
	SystemMatrix(mesh3, A);
		
	// LU decomposition ---Equation(3.27)
	SparseLU<Eigen::SparseMatrix<double>> lu(A.transpose()*A);
	x.col(0) = lu.solve(A.transpose()*F_x);
	x.col(1) = lu.solve(A.transpose()*F_y);
	x.col(2) = lu.solve(A.transpose()*F_z);

	//vertex constraint
	double error[3];
	for (int i = 0; i < 3; ++i) {
		error[i] = x(0, i) - mesh3->getm_V(0, i);
		for (int j = 0; j < vertex_num; ++j) {
			x(j, i) -= error[i];
		}
	}
}



//	mesh1: undeformed_source_mesh	mesh2: deformed_source_mesh
void Source_Transformation(Mesh* mesh1, Mesh* mesh2,VectorXd& f_x,VectorXd& f_y,VectorXd& f_z) {
	
	int triangle_num = mesh1->getTriNum();	// Number of mesh
	f_x = VectorXd::Zero(3 * triangle_num);
	f_y = VectorXd::Zero(3 * triangle_num);
	f_z = VectorXd::Zero(3 * triangle_num);
	
	for (int j = 0; j < triangle_num; ++j) {	// Index j for each triangle

		// Vector for source undeformed/deformed vertex
		Vector3d V_undeformed_source[4], V_deformed_source[4];
		
		for (int i = 0; i < 3; ++i) {	// Index i  for each triangle vertex
			int index_undeformed[3], index_deformed[3];

			index_undeformed[i] = mesh1->getm_F(j, i);
			index_deformed[i] = mesh2->getm_F(j, i);

			for (int k = 0; k < 3; ++k) {	// k expresses x, y or z
				V_undeformed_source[i](k) = mesh1->getm_V(index_undeformed[i], k);
				V_deformed_source[i](k) = mesh2->getm_V(index_deformed[i], k);
			}
		}


		// Make fouth undeformed/defomed Vertex ---Equation.(3.7)	
		Vector3d e1_undeformed = V_undeformed_source[1] - V_undeformed_source[0];
		Vector3d e2_undeformed = V_undeformed_source[2] - V_undeformed_source[0];
		Vector3d e3_undeformed = e1_undeformed.cross(e2_undeformed);	// Cross product
		double e3_normlize_undeformed = e3_undeformed.norm();
		Vector3d V_cross_undeformed = e3_undeformed.array() / e3_normlize_undeformed;
		V_undeformed_source[3] = V_undeformed_source[0] + V_cross_undeformed;

		Vector3d e1_deformed = V_deformed_source[1] - V_deformed_source[0];
		Vector3d e2_deformed = V_deformed_source[2] - V_deformed_source[0];
		Vector3d e3_deformed = e1_deformed.cross(e2_deformed);	// Cross product
		double e3_normlize_deformed = e3_deformed.norm();
		Vector3d V_cross_deformed = e3_deformed.array() / e3_normlize_deformed;
		V_deformed_source[3] = V_deformed_source[0] + V_cross_deformed;


		// Make Source_EdgeMatrix ---Equation(3.5)
		Matrix3d V_Edge_undeformedSource(3, 3), V_Edge_deformedSource(3, 3);
		for (int i = 0; i < 3; ++i) {
			V_Edge_undeformedSource.col(i) = V_undeformed_source[i + 1] - V_undeformed_source[0];
			V_Edge_deformedSource.col(i) = V_deformed_source[i + 1] - V_deformed_source[0];
		}


		// Compute source transformation ---Equation(3.6)
		Matrix3d S;
		S = V_Edge_deformedSource*(V_Edge_undeformedSource.inverse());


		// Make f_x,f_y,f_z
		for (int i = 0; i < 3; ++i) {
			f_x(3 * j + i) = S.coeff(0, i);
			f_y(3 * j + i) = S.coeff(1, i);
			f_z(3 * j + i) = S.coeff(2, i);
		}
	}
}




//	mesh3: undeformed_target_mesh
void SystemMatrix(Mesh* mesh3, Eigen::SparseMatrix<double>& a){

	int triangle_num = mesh3->getTriNum();	// Number of mesh
	int vertex_num = mesh3->getVertNum();	// Number of vertex	
	Vector3d V_undeformed_target[3];	// Vector for undeformed triangle vertex
	
	typedef Triplet<double> Tri;
	std::vector<Tri> triplets;

	for (int j = 0; j < triangle_num; ++j) {

		// Compute Vector for undeformed target vertex
		for (int i = 0; i < 3; ++i) {
			int index[3];
			index[i] = mesh3->getm_F(j, i);

			for (int k = 0; k < 3; ++k) {
				V_undeformed_target[i](k) = mesh3->getm_V(index[i], k);
			}
		}

		// Make target edgeMatrix ---Equation(3.8)
		MatrixXd W(3, 2);
		for (int i = 0; i < 2; ++i) {
			W.col(i) = V_undeformed_target[i + 1] - V_undeformed_target[0];
		}

		// Compute QR decomposeition ---Equation(3.10)
		MatrixXd Q(3, 3);	// Orthogonal Matrix
		MatrixXd R(3, 2);	// Upper triangle Matrix
		R.setZero();
		MatrixXd Q_block(3, 2);
		MatrixXd R_block(2, 2);
		QRFactorize(W, Q, R);
		Q_block = Q.block(0, 0, 3, 2);
		R_block = R.block(0, 0, 2, 2);

		// Make T including [a`f] elements
		MatrixXd T(2, 3);
		T = R_block.inverse()*Q_block.transpose();

		// Make system matrix A
		int index_target[3];
		for (int i = 0; i < 3; ++i) {
			index_target[i] = mesh3->getm_F(j, i);
		}

		for (int i = 0; i < 3; ++i) {
			triplets.push_back(Tri(3 * j + i, index_target[1], T.coeff(0, i)));
			triplets.push_back(Tri(3 * j + i, index_target[2], T.coeff(1, i)));
			triplets.push_back(Tri(3 * j + i, index_target[0], -T.coeff(0, i) - T.coeff(1, i)));
		}
	}
	a.setFromTriplets(triplets.begin(), triplets.end());
}




// Compute QR decomposition
void QRFactorize(const MatrixXd &a, MatrixXd &q, MatrixXd &r){
	int i, j, imax, jmax;
	imax = a.rows();
	jmax = a.cols();

	for (j = 0; j<jmax; j++)
	{
		Eigen::VectorXd v(a.col(j));
		for (i = 0; i<j; i++)
		{
			Eigen::VectorXd qi(q.col(i));
			r(i, j) = qi.dot(v);
			v = v - r(i, j)*qi;
		}
		float vv = (float)v.squaredNorm();
		float vLen = sqrtf(vv);
		if (vLen < EPSILON){
			r(j, j) = 1;
			q.col(j).setZero();
		}
		else{
			r(j, j) = vLen;
			q.col(j) = v / vLen;
		}
	}
}

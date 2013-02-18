#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "curvature.h"
using namespace OpenMesh;
using namespace Eigen;
using namespace std;

void computeCurvature(Mesh &mesh, OpenMesh::VPropHandleT<CurvatureInfo> &curvature) 
{
	Eigen::Matrix3d I = Matrix3d::Identity();
	for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it) {
		const Vec3f & vertex_iF = mesh.point(v_it.handle());
		const Vec3f & vertexNormal_iF = mesh.normal(v_it.handle());
		
		Vector3d v_i(vertex_iF[0], vertex_iF[1], vertex_iF[2]);
		Vector3d N_i(vertexNormal_iF[0], vertexNormal_iF[1], vertexNormal_iF[2]);

		double sumWeight_i = 0.0;
		Eigen::Matrix3d M_i = Matrix3d::Zero();
		for (Mesh::VertexOHalfedgeIter voh_v_it = mesh.voh_iter(v_it); voh_v_it; ++voh_v_it) {
			const Vec3f & vertex_jF = mesh.point(mesh.to_vertex_handle(voh_v_it));
			Vector3d v_j(vertex_jF[0], vertex_jF[1], vertex_jF[2]);
			
			// Calculate tangent vector
			Vector3d T_ij = (I - N_i * N_i.transpose()) * (v_i - v_j);
			T_ij.normalize();
			
			// Calculate curvature
			Vector3d v_ji = v_j - v_i;
			double kappa_ij = N_i.dot(v_ji) * 2.0 / v_ji.squaredNorm();

			// Calculate weight
			double w_ij = 0.0;
			// Warning: assume mesh is closed
			w_ij += mesh.calc_sector_area(voh_v_it.handle());
			w_ij += mesh.calc_sector_area(mesh.opposite_halfedge_handle(voh_v_it.handle()));

			M_i += (w_ij * kappa_ij) * (T_ij * T_ij.transpose());
			sumWeight_i += w_ij;
		}
		// Normalize M_i
		M_i /= sumWeight_i;
		
		// Find the two eigenvectors
		EigenSolver<Matrix3d> solver(M_i);
		CurvatureInfo info;
		for (int i=0, j=0; i<3; ++i) {
			double eig = real(solver.eigenvalues()(i));
			if (abs(eig) > 1e-6) {
				info.curvatures[j] = eig;
				Vector3d v = solver.pseudoEigenvectors().block(0, i, 3, 1);
				info.directions[j++] = Vec3f(v[0], v[1],v[2]);
			}
		} 
		mesh.property(curvature, v_it) = info;
	}
}

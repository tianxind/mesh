#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include "curvature.h"
using namespace OpenMesh;
using namespace Eigen;
using namespace std;

void computeCurvature(Mesh &mesh, OpenMesh::VPropHandleT<CurvatureInfo> &curvature) {

  int matched = 0;
  for (Mesh::VertexIter it = mesh.vertices_begin(); it != mesh.vertices_end(); ++it) {
    // WRITE CODE HERE TO COMPUTE THE CURVATURE AT THE CURRENT VERTEX
    Vec3f v_i = mesh.point(it.handle());
    Vector3d vi(v_i[0], vi[1], vi[2]);
    Vec3f normal = mesh.normal(it.handle());
    Vector3d N(normal[0],normal[1],normal[2]);

    Mesh::VertexVertexIter vv_it;
    Eigen::Matrix3d M = Matrix3d::Zero();
    Eigen::Matrix3d I = Matrix3d::Ones();
    double sum_weight = 0.0;
    for (vv_it = mesh.vv_iter(it); vv_it; ++vv_it) {
      Vec3f v_j = mesh.point(vv_it.handle());
      Vector3d vj(v_j[0], v_j[1], v_j[2]);
      // Calculate tangent vector
      Vector3d T = (I - N * N.transpose()) * (vi - vj);
      T.normalize();
      // Calculate curvature
      Vector3d vji = vj - vi;
      double kappa = N.dot(vji) * 2 / vji.squaredNorm();
      // Calculate weight
      Mesh::VertexOHalfedgeIter voh_it;
      double w = 0.0;
      for (voh_it = mesh.voh_iter(it); voh_it; ++voh_it) {
	Vec3f to_vertex = mesh.point(mesh.to_vertex_handle(voh_it));
        Vec3f dist = to_vertex - v_j;
        Vector3d dist_vec(dist[0], dist[1], dist[2]);
	if (dist_vec.norm() < 0.001) {
          matched++;
	  w = mesh.calc_sector_area(voh_it.handle());
	  if (!mesh.is_boundary(voh_it))
	    w += mesh.calc_sector_area(mesh.opposite_halfedge_handle(voh_it.handle()));
	}
      }
      sum_weight += w;
      M += w * kappa * T * T.transpose();
    }
    // Normalize M
    M /= sum_weight;
    // Find the two eigenvectors
    EigenSolver<Matrix3d> solver(M);
    int store = 0;
    CurvatureInfo info;
    for (int i = 0; i < 3; ++i) {
      Vector3d v = solver.pseudoEigenvectors().block(0, i, 3, 1);
      double eig = real(solver.eigenvalues()(i));
      if (eig != 0) {
        info.curvatures[store] = eig;
        info.directions[store] = Vec3f(v[0], v[1],v[2]);
        store++;
      }
    } 
    mesh.property(curvature, it) = info;
    // ------------------------------------------------------------------------------------------------
  }
  cout << "Matches found: " << matched << endl;
}

#include <Eigen/Dense>
#include <iostream>

// J is the number of joints
template <int J> class RobotArm {
  private:
    // we're going to represent each screw axis as
    // a row vector, which will make it easier to construct
    // the matrix representation
    const Eigen::Matrix<double, J, 6> Slist;
    // this is the home configuration
    Eigen::Matrix4d M;

    /**
     * This method computes a matrix exponential using Slist and a
     * given angle. It's pretty much a helper function for the
     * methods which compute forward kinematics.
     */
    Eigen::Vector<double, 6> matrixExp6(Eigen::Vector<double, 6> screw_axis, double angle) {
      Eigen::Matrix4d mat_exp = Eigen::Matrix4d::Identity();
      
      Eigen::Vector3d omega = screw_axis.head<3>();
      Eigen::Vector3d v = screw_axis.tail<3>();

      if (omega.norm() == 1) {
        // computing the skew-symmetric representation
        // of omega
        Eigen::Matrix3d skew_symm_mat {
          { 0,       -omega(2),  omega(1) },
          { omega(2), 0,         -omega(0)},
          {-omega(1), omega(0),  0        }
        };

        Eigen::Matrix3d skew_symm_mat_sr = skew_symm_mat * skew_symm_mat;

        // compute the orientation
        mat_exp.topLeftCorner<3, 3>() = 
            Eigen::Matrix3d::Identity() 
              + std::sin(angle) * skew_symm_mat
              + (1 - std::cos(angle)) * skew_symm_mat_sr;

        // compute the position
        mat_exp.block<3, 1>(0, 3) = 
            (
              Eigen::Matrix3d::Identity()
              + (1 - std::cos(angles[i])) * skew_symm_mat
              + (angles[i] - std::sin(angle)) * skew_symm_mat_sr
            ) * v;
      }

      else if (omega.norm() == 0) {
        mat_exp.block<3, 1>(0, 3) = v * angle;
      }

      return mat_exp;
    }
    

  public:
    // TODO: might be worthwhile to add a check which verifies
    // the norm of the rotational component is 1 or 0.
    RobotArm(Eigen::Matrix4d M, Eigen::Matrix<double, J, 6> Slist) 
      : M(M), Slist(Slist) {}

    
    /**
     * The forward kinematics in the space frame are computed using
     * the terms exp(Slist[i] * angle[i]) for i = 0, ..., J-1.
     * 
     * This function returns a transformation matrix which represents
     * the final pose of the arm.
     */
    Eigen::Matrix4d forwardKinSpace(Eigen::Vector<double, J> angles) {
      Eigen::Matrix4d T_sb = M;

      for (int i = J - 1; i >= 0; i--) {
        Eigen::Matrix4d mat_exp = matrixExp6Space(Slist.row(i), angles[i]);
        T_sb = mat_exp * T_sb;
      }

      return T_sb;
    }

    Eigen::Matrix4d forwardKinBody(Eigen::Vector<double, J> angles) {
      Eigen::Matrix4d T_bb = M;

      for (int i = 0; i < J; i++) {
        Eigen::Vector<double, 6> B = M.inverse().adjoint() * Slist.row(i);
        Eigen::Matrix4d mat_exp = matrixExp6Body(B, angles[i]);
        T_bb =  T_bb * mat_exp;
      }

      return T_bb;

    }

    void inverseKinSpace() {}

    void inverseKinBody() {}

    void printSlist() {}
};
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <Eigen/Geometry>
#include <iostream>
#include <cmath> 

// J is the number of joints
template <int J> class RobotArm {
  private:
    // we're going to represent each screw axis as
    // a row vector, which will make it easier to construct
    // the matrix representation
    Eigen::Matrix<double, J, 6> Slist;
    // this is the home configuration
    Eigen::Matrix4d M;

    /*********************************************************************
    *
    *                      Helper Methods
    * 
    *********************************************************************/

    /**
     * This method computes a matrix exponential using Slist and a
     * given angle. It's pretty much a helper function for the
     * methods which compute forward kinematics.
     */
    Eigen::Matrix4d matrixExp6(Eigen::Vector<double, 6> screw_axis, double angle) {
      Eigen::Isometry3d mat_exp = Eigen::Isometry3d::Identity();
      
      Eigen::Vector3d omega = screw_axis.head<3>();
      Eigen::Vector3d v = screw_axis.tail<3>();

      if (omega.norm() == 1) {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        // computing the skew-symmetric representation
        // of omega
        Eigen::Matrix3d skew_mat {
          { 0,       -omega(2),  omega(1) },
          { omega(2), 0,         -omega(0)},
          {-omega(1), omega(0),  0        }
        };

        // compute the orientation (we have to use linear,
        // because it returns a writable version of R)
        mat_exp.linear() = I + std::sin(angle) * skew_mat
                              + (1 - std::cos(angle)) * skew_mat * skew_mat;

        // compute the position
        mat_exp.translation() = (I + (1 - std::cos(angle)) * skew_mat
                                + (angle - std::sin(angle)) 
                                * skew_mat * skew_mat) * v;
      }

      else if (omega.norm() == 0) {
        mat_exp.translation() = v * angle;
      }

      return mat_exp.matrix();
    }

    Eigen::Matrix<double, 6, 6> Ad(Eigen::Matrix4d T) {
      Eigen::Matrix<double, 6, 6> Ad_T = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Isometry3d T_iso (T);
      Eigen::Vector3d p = T_iso.translation();
      Eigen::Matrix3d R = T_iso.rotation();


      Eigen::Matrix3d skew_mat {
        { 0,    -p(2),  p(1) },
        { p(2), 0,     -p(0) },
        {-p(1), p(0),   0    }
      };

      Ad_T.topLeftCorner<3, 3>() = R;
      Ad_T.bottomRightCorner<3, 3>() = R;
      Ad_T.bottomLeftCorner<3, 3>() = skew_mat * R;

      return Ad_T;
    }

    /**
     * This method computes the Jacobian of the robot arm.
     */
    Eigen::Matrix<double, 6, J> jacobian(Eigen::Vector<double, J> angles) {
      Eigen::Matrix<double, 6, J> jacobian = Eigen::Matrix<double, 6, J>::Zero();
      Eigen::Matrix4d T_i = Eigen::Matrix4d::Identity();

      jacobian.col(0) = Slist.row(0);

      for (int i = 1; i < J; i++) {
        T_i = T_i * matrixExp6(Slist.row(i -1), angles(i - 1));
        jacobian.col(i) = Ad(T_i) * Slist.row(i).transpose();
      }
      
      return jacobian;
    }

    /**
     * This method computes the pseudo inverse of a Jacobian matrix. The method used
     * is to first use factorize the matrix using singular value decomposition U S V^T, then
     * return the inverse V S_inv U^T
     **/
    Eigen::Matrix<double, J, 6> pseudo_jacobian(Eigen::Matrix<double, 6, J> jacobian) {
      Eigen::Matrix<double, J, 6> sigma = Eigen::Matrix<double, J, 6>::Zero();
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, J>> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::VectorXd singular_values = svd.singularValues();
      int n = singular_values.size();
          
      // invert sigma
      for (int i = 0; i < n; i++) {
        // set the diagonal to 0 for numerical stability
        if (singular_values(i) <= 1e-6) {
          sigma(i, i) = 0.0;
        }
        else
          sigma(i, i) = 1.0 / singular_values(i);
      }

      return svd.matrixV() * sigma * svd.matrixU().transpose();
    }

    bool tolerance_satisfied(
      Eigen::Matrix4d T1, Eigen::Matrix4d T2,
      double position_tol, double orientation_tol
    ) {
      Eigen::Isometry3d T1_iso (T1);
      Eigen::Isometry3d T2_iso (T2);
      // Position difference (Euclidean distance)
      double position_error = (T1_iso.translation() - T2_iso.translation()).norm();

      // Orientation difference (angle between rotations)
      Eigen::Matrix3d R1 = T1_iso.rotation();
      Eigen::Matrix3d R2 = T2_iso.rotation();

      Eigen::Matrix3d dR = R1.transpose() * R2;
      Eigen::AngleAxisd angle_axis(dR);
      double orientation_error = std::abs(angle_axis.angle());

      return position_error <= position_tol &&
              orientation_error <= orientation_tol;
    }


  public:
    // TODO: might be worthwhile to add a check which verifies
    // the norm of the angular velocity is 1 or 0.
    RobotArm(Eigen::Matrix4d M, Eigen::Matrix<double, J, 6> Slist) 
      : M(M), Slist(Slist) {}

    
    /*********************************************************************
    *
    *                      Forward Kinematics
    * 
    *********************************************************************/

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
        Eigen::Matrix4d mat_exp = matrixExp6(Slist.row(i), angles[i]);
        T_sb = mat_exp * T_sb;
      }

      return T_sb;
    }

    Eigen::Matrix4d forwardKinBody(Eigen::Vector<double, J> angles) {
      Eigen::Matrix4d T_bb = M;

      for (int i = 0; i < J; i++) {
        // screw axis relative to the base frame
        Eigen::Vector<double, 6> B = Ad(M.inverse()) * Slist.row(i).transpose();
        Eigen::Matrix4d mat_exp = matrixExp6(B, angles[i]);
        T_bb =  T_bb * mat_exp;
      }

      return T_bb;
    }

    /*********************************************************************
    *
    *                       Inverse Kinematics
    * 
    *********************************************************************/

    Eigen::Vector<double, J> inverseKinSpace(
      Eigen::Matrix4d T_sd,
      Eigen::Vector<double, J> initial_guess,
      int max_iters = 20,
      double position_tol = 1e-3,
      double orientation_tol = 1e-3
    ) {
      Eigen::Matrix4d T_sb = forwardKinSpace(initial_guess);
      Eigen::Vector<double, J> angles = initial_guess;
      bool tol_satisfied = tolerance_satisfied(
        T_sd, T_sb, position_tol, orientation_tol
      );
      int i = 0;

      while (i < max_iters && !tol_satisfied) {
        // compute the Jacobian and its pseudo inverse
        Eigen::Matrix<double, 6, J> j = jacobian(angles);
        Eigen::Matrix<double, J, 6> j_inv = pseudo_jacobian(j);
        
        // update the angles using Newton-Raphson method
        angles = angles - j_inv * T_sb;

        // update current solution
        T_sb = forwardKinSpace(angles);
        tol_satisfied = tolerance_satisfied(
          T_sd, T_sb, position_tol, orientation_tol
        );
      }

      return T_sb;
    }

    void inverseKinBody() {}

    void printSlist() {
      std::cout << "Screw Axis List\n";
      std::cout << Slist << std::endl;
    }
};
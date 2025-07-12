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
    const Eigen::Matrix<double, J, 6> Slist;
    // this is the home configuration
    const Eigen::Matrix4d M;

    /*********************************************************************
    *
    *                      Helper Methods
    * 
    *********************************************************************/

    /**
     * This method computes the skew symmetric matrix
     * representation of a vector v.
     */
    Eigen::Matrix3d vector_to_skew_symm_mat(const Eigen::Vector3d &v) {
      Eigen::Matrix3d skew_symm_matrix {
        { 0,    -v(2),  v(1) },
        { v(2), 0,     -v(0) },
        {-v(1), v(0),   0    }
      };
      return skew_symm_matrix;
    }

    /**
     * This method computes the the vector representation of a
     * skew symmetric matrix.
     */
    Eigen::Vector3d skew_symm_mat_to_vector(
      const Eigen::Matrix3d &skew_symm_matrix) {
      Eigen::Vector3d v {
        skew_symm_matrix(2, 1),
        skew_symm_matrix(0, 2),
        skew_symm_matrix(1, 0)
      };
      return v;
    }


    /**
     * This method computes a matrix exponential using Slist and a
     * given angle. It's pretty much a helper function for the
     * methods which compute forward kinematics.
     */
    Eigen::Matrix4d matrixExp6(
      const Eigen::Vector<double, 6> &screw_axis,
      double angle
    ) {
      Eigen::Isometry3d mat_exp = Eigen::Isometry3d::Identity();
      
      Eigen::Vector3d omega = screw_axis.head<3>();
      Eigen::Vector3d v = screw_axis.tail<3>();

      if (omega.norm() == 1) {
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        // computing the skew-symmetric representation
        // of omega
        auto skew_symm_matrix = vector_to_skew_symm_mat(omega);

        // compute the orientation (we have to use linear,
        // because it returns a writable version of R)
        mat_exp.linear() = I + std::sin(angle) * skew_symm_matrix
                             + (1 - std::cos(angle)) 
                             * skew_symm_matrix * skew_symm_matrix;

        // compute the position
        auto G = I*angle + (1 - std::cos(angle)) * skew_symm_matrix
                                + (angle - std::sin(angle)) 
                                * skew_symm_matrix * skew_symm_matrix;
        mat_exp.translation() = G * v;
      }

      else if (omega.norm() == 0) {
        mat_exp.translation() = v * angle;
      }

      return mat_exp.matrix();
    }

    Eigen::Matrix<double, 6, 6> Ad(const Eigen::Matrix4d &T) {
      Eigen::Matrix<double, 6, 6> Ad_T = Eigen::Matrix<double, 6, 6>::Zero();
      Eigen::Isometry3d T_iso (T);
      Eigen::Vector3d p = T_iso.translation();
      Eigen::Matrix3d R = T_iso.rotation();


      auto skew_symm_matrix = vector_to_skew_symm_mat(p);

      Ad_T.topLeftCorner<3, 3>() = R;
      Ad_T.bottomRightCorner<3, 3>() = R;
      Ad_T.bottomLeftCorner<3, 3>() = skew_symm_matrix * R;

      return Ad_T;
    }

    /*
     * This method computes the matrix logarithm for a matrix T in SE(3).
     * 
     * @param T: the matrix to compute the matrix logarithm of
     * @param[out] twist: A 6-element vector to store the 
     *                    resulting twist coordinates (w, v).
     * @param[out] theta: A double to store the magnitude of rotation.
     */
    void matrixLog6(
      const Eigen::Matrix4d &T,
      Eigen::Vector<double, 6> &twist,
      double &theta
    ) {
      Eigen::Isometry3d T_iso(T);
      Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
      Eigen::Matrix3d skew_symm_matrix;
      Eigen::Vector3d omega {0.0, 0.0, 0.0};
      Eigen::Vector3d v {0.0, 0.0, 0.0};
      auto R = T_iso.rotation();
      auto p = T_iso.translation();

      // handling special case (no rotation)
      // in this case, we leave omega at zero
      if (R == I) {
        // leave angular components at zero
        theta = p.norm();
        v(0) = p(0) / theta;
        v(1) = p(1) / theta;
        v(2) = p(2) / theta;
      }

      // Computing the angular velocity
      else if (R.trace() == -1) {
        // set theta to pi
        theta = std::numbers::pi;

        // choose a feasible solution
        if (R(2, 2) != -1) {
          double term = 1.0 / std::sqrt(2 * (1 + R(2, 2)));
          omega(0) = term * R(0, 2);
          omega(1) = term * R(1, 2);
          omega(2) = term * (R(2, 2) + 1);
        }
        
        else if (R(1, 1) != -1) {
          double term = 1.0 / std::sqrt(2 * (1 + R(1, 1)));
          omega(0) = term * R(0, 1);
          omega(1) = term * (R(1, 1) + 1);
          omega(2) = term * R(2, 1);
        }

        else if (R(0, 0) != -1) {
          double term = 1.0 / std::sqrt(2 * (1 + R(0, 0)));
          omega(0) = term * (R(0, 0) + 1);
          omega(1) = term * R(1, 0);
          omega(2) = term * R(2, 0);
        }
        skew_symm_matrix = vector_to_skew_symm_mat(omega);
      }

      else {
        theta = std::acos(0.5 * (R.trace() - 1.0));
        skew_symm_matrix = (1.0 / 2.0 * std::sin(theta)) * (R - R.transpose());
        // extracting angular components from skew symmetric matrix
        omega = skew_symm_mat_to_vector(skew_symm_matrix);
      }

      // compute the linear components
      auto G_inv = ((1.0 / theta) * I) -
        (0.5 * skew_symm_matrix) +
        ((1.0 / theta) -
          0.5 * (1.0 / std::tan(theta / 2.0))) *
          skew_symm_matrix * skew_symm_matrix;

      v = G_inv * p;

      // set the twist vector
      twist << omega(0), omega(1), omega(2), v(0), v(1), v(2);
    }

    /**
     * This method computes the Jacobian of the robot arm
     * relative to the space frame.
     * 
     * @param angles: the angles of the robot arm
     * @return the Jacobian matrix of the robot arm
     */
    Eigen::Matrix<double, 6, J> space_jacobian(const Eigen::Vector<double, J> &angles) {
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
     * This method computes the pseudo inverse of a Jacobian matrix using SVD.
     *
     * @param jacobian: the Jacobian matrix to compute the pseudo inverse of
     * @return the inverse V S_inv U^T
     **/
    Eigen::Matrix<double, J, 6> pseudo_inv_jacobian(
      const Eigen::Matrix<double, 6, J> &jacobian) {
      Eigen::Matrix<double, J, 6> sigma = Eigen::Matrix<double, J, 6>::Zero();
      Eigen::JacobiSVD<Eigen::Matrix<double, 6, J>> svd(
        jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV
      );
      Eigen::VectorXd singular_values = svd.singularValues();
      int n = singular_values.size();
          
      // invert sigma
      for (int i = 0; i < n; i++) {
        // if the singular value is close to zero
        // set it to 0 for numerical stability
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
    Eigen::Matrix4d forwardKinSpace(const Eigen::Vector<double, J> &angles) {
      Eigen::Matrix4d T_sb = M;

      for (int i = J - 1; i >= 0; i--) {
        Eigen::Matrix4d mat_exp = matrixExp6(Slist.row(i), angles[i]);
        T_sb = mat_exp * T_sb;
      }

      return T_sb;
    }

    Eigen::Matrix4d forwardKinBody(const Eigen::Vector<double, J> &angles) {
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

    /**
     * This method computes the inverse kinematics of the robot arm
     * in the space frame.
     * 
     * @param T_sd: The desired end-effector pose in the space frame.
     * @param[out] angles: This is the initial guess for the 
     * angles of the robot arm.
     * The angles are updated in place to the solution of the inverse kinematics.
     * @param max_iters: The maximum number of iterations to perform.
     * @param position_tol: The tolerance for the position error.
     * @param orientation_tol: The tolerance for the orientation error.
     * 
     */
    void inverseKinSpace(
      const Eigen::Matrix4d &T_sd,
      Eigen::Vector<double, J> &angles,
      int max_iters = 20,
      double position_tol = 1e-3,
      double orientation_tol = 1e-3
    ) {
      // compute the forward kinematics of the initial guess
      Eigen::Matrix4d T_sb = forwardKinSpace(angles);
      double theta;
      
      bool tol_satisfied = tolerance_satisfied(
        T_sd, T_sb, position_tol, orientation_tol
      );
      
      int i = 0;

      while (i < max_iters && !tol_satisfied) {
        // compute the Jacobian and its pseudo inverse
        Eigen::Matrix<double, 6, J> space_j = space_jacobian(angles);
        // Eigen::Matrix<double, J, 6> j_inv = pseudo_inv_jacobian(space_j);
        
        // compute the body twist
        Eigen::Vector<double, 6> twist;
        matrixLog6(T_sb.inverse() * T_sd, twist, theta);

        // compute the spatial twist
        Eigen::Vector<double, 6> spatial_twist = Ad(T_sb) * twist;

        // update the joint angles
        // angles = angles + j_inv * spatial_twist;
        angles = angles + space_j.transpose() * spatial_twist;

        T_sb = forwardKinSpace(angles);

        tol_satisfied = tolerance_satisfied(
          T_sd, T_sb, position_tol, orientation_tol
        );

        i++;
      }
    }

    void inverseKinBody() {}

    void printSlist() {
      std::cout << "Screw Axis List\n";
      std::cout << Slist << std::endl;
    }
};
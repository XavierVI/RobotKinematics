#include <Eigen/Dense>
#include <iostream>

// J is the number of joints
template <int J> class RobotArm {
  private:
    // we're going to represent each screw axis as
    // a row vector, which will make it easier to construct
    // the matrix representation
    Eigen::Matrix<double, J, 6> Slist;
    // this is the home configuration
    Eigen::Matrix4d M;

  public:
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
      Eigen::ArithmeticSequence seq = Eigen::seq(0, 2);

      for (int i = J - 1; i >= 0; i--) {
        // update the rotational component
        Eigen::Matrix3d skew_symm {
          {0, -Slist[i, 2], Slist[i, 1]},
          {Slist[i, 2], 0, -Slist[i, 0]},
          {-Slist[i, 1], Slist[i, 0], 0}
        };

        T_sb(seq, seq) = 
            Eigen::Matrix3d::Identity() 
              + std::cos(angles[i])*skew_symm 
              + (1 - std::cos(angles[i]))*skew_symm*skew_symm;

        // update the position
        T_sb(seq, 3) = 
            (Eigen::Matrix3d::Identity()
              + (1 - std::cos(angles[i]))*skew_symm
              + (angles[i] - std::sin(angles[i]))*skew_symm*skew_symm
            )*Slist(2, 6);
      }

      return T_sb;
    }

    void forwardKinBody() {}

    void inverseKinSpace() {}

    void inverseKinBody() {}

    void printSlist() {}
};
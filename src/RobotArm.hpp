#include <Eigen/Dense>

// J is the number of joints
template <int J>
class RobotArm {
  private:
    // we're going to represent each screw axis as
    // a row vector, which will make it easier to construct
    // the matrix representation
    Eigen::Matrix<double, J, 6> Slist;
    // this is the home configuration
    Eigen::Matrix4d M;

    public:
      RobotArm(Eigen::Matrix4d M, Eigen::Matrix<double, J, 6>);

      void forwardSpaceKin();

      void forwardBodyKin();

      void inverseSpaceKin();

      void inverseBodyKin();
};
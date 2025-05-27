#include "RigidBody.hpp"
#include "RobotArm.hpp"

int main(int argc, char* argv[]) {
  //-----------------------------------------------
  // Example of a rotation for a planar rigid-body
  //-----------------------------------------------
  double theta = 0;

  // orientation of {b} wrt {s}
  Eigen::Matrix2d P {
    {std::cos(theta), -std::sin(theta)},
    {std::sin(theta), std::cos(theta)}
  };

  Eigen::Vector2d p {2, 2};

  PlanarRigidBody planarRigidBody (P, p);

  std::cout << "Before rotation\n";

  planarRigidBody.print_body_frame_position();

  std::cout << "After rotation\n";

  planarRigidBody.rotate_rigid_body(std::numbers::pi / 2);
  planarRigidBody.print_body_frame_position();

  //-----------------------------------------------
  // Example of a rotation for a rigid body in three
  // dimensions, using exponential coordinates
  //-----------------------------------------------
  // initial position and orientation of the
  // body frame represented as a transformation
  // matrix
  std::cout << "========================\n";
  std::cout << "Three dimensions\n";

  Eigen::Matrix4d T_sb {
    {1, 0, 0, 2},
    {0, 1, 0, 2},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  };
  theta = std::numbers::pi / 2;

  std::cout << "Before\n";
  std::cout << T_sb(Eigen::seq(0, 2), Eigen::seq(0, 2)) << std::endl;
  
  RigidBody rigidBody (T_sb);
  Eigen::Vector3d axis {0, 0, 1};
  rigidBody.rotate_rigid_body(axis, theta);
  std::cout << "After\n";
  rigidBody.print_body_frame();

  


  return 0;
}

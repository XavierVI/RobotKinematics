#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <numbers>

/**
 * This file is used to write code to help solidify concepts from the
 * modern robotics textbook.
 **/



class PlanarRigidBody {

  private:
    Eigen::Matrix2d space_frame;
    // Eigen::Matrix2d body_frame;

    // orientation and position of the body frame wrt the space frame
    Eigen::Matrix2d P;
    Eigen::Vector2d p;

    Eigen::Vector3d screw_axis;

  public:
    PlanarRigidBody(
      Eigen::Matrix2d P,
      Eigen::Vector2d p,
      Eigen::Vector3d screw_axis
    ) {
      space_frame = Eigen::Matrix2d {
        {1, 0},
        {0, 1}
      };

      this->P = P;
      this->p = p;
      this->screw_axis = screw_axis;
    }
};

int main(int argc, char* argv[]) {
  // creating an example of exponential coordinates
  // space frame
  Eigen::Matrix2d s_frame {
    {1, 0},
    {0, 1}
  };

  double theta = std::numbers::pi;

  // orientation of {b} wrt {s}
  Eigen::Matrix2d P {
    {std::cos(theta), -std::sin(theta)},
    {std::sin(theta), std::cos(theta)}
  };

  Eigen::Vector2d p {3, 1};

  Eigen::Vector3d screw_axis {std::numbers::pi / 2, std::numbers::pi, 0};

  PlanarRigidBody rigidBody (P, p, screw_axis);

  std::cout << s_frame << std::endl;
  return 0;
}

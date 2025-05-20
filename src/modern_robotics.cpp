#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <numbers>

/********************************************************************
 * This file is used to write code to help solidify concepts from the
 * modern robotics textbook.
 *******************************************************************/


class PlanarRigidBody {

  private:
    Eigen::Matrix2d space_frame;
    // Eigen::Matrix2d body_frame;

    // orientation and position of the body frame wrt the space frame
    Eigen::Matrix2d R_sb;
    Eigen::Vector2d p;

    Eigen::Vector3d screw_axis;

  public:
    PlanarRigidBody(
      Eigen::Matrix2d R_sb,
      Eigen::Vector2d p,
      Eigen::Vector3d screw_axis
    ) {
      space_frame = Eigen::Matrix2d {
        {1, 0},
        {0, 1}
      };

      this->R_sb = R_sb;
      this->p = p;
      this->screw_axis = screw_axis;
    }

    /**
     * Rotates the body frame. The method used to perform this rotation is
     * the following.
     * 
     * R_sb is a matrix which represents the orientation of {b} in {s}.
     * We create another matrix R_bb' which represents the new orientation of {b} ({b'}).
     * 
     * We compute R_sb' = R_sb * R_bb', yielding the new orientation of the {b} in {s}.
     * */
    void move_rigid_body(double theta) {
      Eigen::Matrix2d R_bb_prime {
        {std::cos(theta), -std::sin(theta)},
        {std::sin(theta), std::cos(theta)}
      };

      this->R_sb = this->R_sb * R_bb_prime;
    }

    void print_body_frame_position() {
      std::cout << "Position" << std::endl << this->p << std::endl;
      std::cout << "Orientation" << std::endl << this->R_sb << std::endl;
    }
};

int main(int argc, char* argv[]) {
  // creating an example of exponential coordinates
  // space frame
  Eigen::Matrix2d s_frame {
    {1, 0},
    {0, 1}
  };

  double theta = 0;

  // orientation of {b} wrt {s}
  Eigen::Matrix2d P {
    {std::cos(theta), -std::sin(theta)},
    {std::sin(theta), std::cos(theta)}
  };

  Eigen::Vector2d p {2, 2};

  Eigen::Vector3d screw_axis {1, 2, 0};

  PlanarRigidBody rigidBody (P, p, screw_axis);

  std::cout << "Before rotation\n";

  rigidBody.print_body_frame_position();

  std::cout << "After rotation\n";

  rigidBody.move_rigid_body(std::numbers::pi / 2);
  rigidBody.print_body_frame_position();

  return 0;
}

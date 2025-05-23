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

  public:
    PlanarRigidBody(
      Eigen::Matrix2d R_sb,
      Eigen::Vector2d p
    ) {
      space_frame = Eigen::Matrix2d {
        {1, 0},
        {0, 1}
      };

      this->R_sb = R_sb;
      this->p = p;
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
    void rotate_rigid_body(double theta) {
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

class RigidBody {
  /**
   * This class represents a three-dimensional rigid body.
   * 
   * The goal with this class is to better understand rotations which are
   * using exponential coordinates. These rotations specifically use 
   * Rodrigues' formula.
   */

  private:
    Eigen::Matrix3d space_frame;
    Eigen::Matrix3d body_frame;

  public:
    RigidBody(Eigen::Matrix3d body_frame) {
      this->space_frame = Eigen::Matrix3d {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
      };

      this->body_frame = body_frame;
    }

    /**
     * This method uses Rodrigues' formula to rotate the body frame
     * about an axis defined in the space frame. */
    void rotate_rigid_body(Eigen::Vector3d axis, double theta) {
      // forming the skew-symmetric representation of the axis
      Eigen::Matrix3d skew_symm_mat {
        {0,          -axis[2],   axis[1]},
        {axis[2],    0,          axis[0]},
        {-axis[1],   axis[0],    0}
      };

      Eigen::Matrix3d I {
        {1,0,0},
        {0,1,0},
        {0,0,1}
      };

      Eigen::Matrix3d R = I + std::sin(theta)*skew_symm_mat 
          + (1 - std::cos(theta))*skew_symm_mat*skew_symm_mat;

        body_frame = R * body_frame;
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

  PlanarRigidBody rigidBody (P, p);

  std::cout << "Before rotation\n";

  rigidBody.print_body_frame_position();

  std::cout << "After rotation\n";

  rigidBody.rotate_rigid_body(std::numbers::pi / 2);
  rigidBody.print_body_frame_position();

  return 0;
}

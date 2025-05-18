#include <iostream>
#include <Eigen/Dense>

/**
 * This file is used to write code to help solidify concepts from the
 * modern robotics textbook.
 **/



class PlanarRigidBody {

  private:
    Eigen::Matrix2d space_frame;
    Eigen::Matrix2d body_frame;

    // orientation and position of the body frame wrt the space frame
    Eigen::Matrix2d P;
    Eigen::Vector2d p;
    int i;

  public:
    PlanarRigidBody(Eigen::Matrix2d P, Eigen::Vector2d p) {
      space_frame = Eigen::Matrix2d {
        {1, 0},
        {0, 1}
      };

      this->P = P;
      this->p = p;
    }
};

int main(int argc, char* argv[]) {
  // creating an example of exponential coordinates
  // space frame
  Eigen::Matrix2d s_frame {
    {1, 0},
    {0, 1}
  };

  // orientation of {b} wrt {s}
  Eigen::Matrix2d P {
    {}
  };

  Eigen::Vector2d p {0, 0};

  std::cout << s_frame << std::endl;
  return 0;
}

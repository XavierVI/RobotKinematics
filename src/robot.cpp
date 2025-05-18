#include <vector>
#include <iostream>
#include <Eigen/Dense>

int main(int argc, char* argv[]) {
  std::vector<int> v (5, 0);

  std::cout << v[0] << std::endl;

  Eigen::Matrix3d matrix = Eigen::Matrix3d::Random();

    std::cout << matrix << std::endl;

  return 0;
}
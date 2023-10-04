#ifndef MOVEIT_OPW_KINEMATICS_TEST_UTILS_
#define MOVEIT_OPW_KINEMATICS_TEST_UTILS_

#include <Eigen/Geometry>

namespace moveit_opw_kinematics_plugin
{
namespace testing
{
const double TOLERANCE = 1e-6;  // absolute tolerance for EXPECT_NEAR checks

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Isometry>;

/** \brief Compare every element of two transforms.
 */
template <typename T>
void comparePoses(const Transform<T>& Ta, const Transform<T>& Tb)
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
  EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
  EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}
}  // namespace testing
}  // namespace moveit_opw_kinematics_plugin

#endif
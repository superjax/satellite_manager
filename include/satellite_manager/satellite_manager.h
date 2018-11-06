#include <vector>

#include "Eigen/Core"

extern "C" {
#include "rtklib.h"
}

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;

class SatelliteManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SatelliteManager();

  void addEphemeris(const eph_t& eph);
  void addEphemeris(const geph_t& eph);
  void addObservation(const obsd_t& obs);

  void getSatState(int sat_id, const gtime_t& t, Vector3d& posECEF, Vector3d& velECEF);
  void update();

private:
  std::vector<obsd_t> obs_;
  nav_t nav_;
  std::vector<double> rs_;
  std::vector<double> dts_;
  std::vector<double> var_;
  std::vector<int> svh_;
};

#include <vector>
#include <stdint.h>
#include <experimental/filesystem>
#include <fstream>

#include "Eigen/Core"

extern "C" {
#include "rtklib.h"
}

namespace fs = std::experimental::filesystem;

#define GPS_EPHEMERIS_ARRAY_SIZE 33
#define GLONASS_EPHEMERIS_ARRAY_SIZE (NSATGLO)
#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS

using namespace Eigen;

typedef Matrix<double, 7, 1> Vector7d;
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 8, 1> Vector8d;

class SatelliteManager
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SatelliteManager();
  ~SatelliteManager();

  void addEphemeris(const eph_t& eph);
  void addEphemeris(const geph_t& eph);
  void addObservation(const obsd_t& obs);
  void startLog(std::string file);

  uint8_t numSats() const { return obs_vec_.size(); }
  std::vector<uint8_t> satIds();
  bool getSatState(int sat_id, gtime_t &t, Vector8d& state, double& var, int& health);
  void update();

private:
  std::vector<obsd_t> obs_vec_;
  nav_t nav_;
  obs_t obs_;
  Matrix<double, 6, -1> rs_;
  Matrix<double, 2, -1> dts_;
  Matrix<double, 1, -1> var_;
  Matrix<int, 1, -1> svh_;
  gtime_t start_ = {};

  std::ofstream file_;
};


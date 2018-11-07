#include <vector>
#include <stdint.h>
#include <experimental/filesystem>
#include <fstream>

#include "Eigen/Core"

extern "C" {
#include "rtklib.h"
}

namespace fs = std::experimental::filesystem;

#define GPS_EPHEMERIS_ARRAY_SIZE NSATGPS
#define GLONASS_EPHEMERIS_ARRAY_SIZE (NSATGLO)
#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS

using namespace Eigen;

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

  /**
   * @brief lsPositioning
   * calculate position with least squares
   * @param state [pos, vel]
   */
  void lsPositioning(Vector6d& state, Matrix3d &Qp, Matrix3d &Qv, gtime_t& time);

  /**
   * @brief satIds
   * @return list of sats with valid positioning
   */
  const std::vector<uint8_t>& satIds() const { return current_sats_; }

  /**
   * @brief numSats
   * @return the number of satellites with valid positioning
   */
  uint8_t numSats() const { return current_sats_.size(); }

  /**
   * @brief update - updates the current sattelite estimates given the
   * supplied ephemeris and observation data at the provided time
   * @param t time to calculate satellite positions
   */
  void update(gtime_t t);

  /**
   * @brief getSatState
   * @param sat_id satellite id
   * @param state [pos[3], vel[3], bias, drift rate]
   * @param var variance (m^2)
   * @param health health flags
   * @return false if error
   */
  bool getSatState(int sat_id, Vector8d& state, double& var, int& health);


private:
  void log();
  std::vector<obsd_t> obs_vec_; // vector of most recent observations for each satellite
  nav_t nav_; // navigation data (used for calculating position, created from ephemeris)

  Matrix<double, 6, -1> rs_; // position and velocity of each sat [pos, vel] (m, m/s) ECEF
  Matrix<double, 2, -1> dts_; // bias and clock drift of each sat [bias, clock drift] (s, s/s)
  Matrix<double, 1, -1> var_; // variance (m^2)
  Matrix<int, 1, -1> svh_; // satellite health flag
  std::vector<uint8_t> current_sats_; // list of currently tracked satellites with valid position and velocity
  gtime_t start_ = {}; // start time (used in logging)
  gtime_t current_time_; // current time that all states are calculated wrt.

  std::ofstream file_; // output file
  prcopt_t opt_ = prcopt_default;
  sol_t sol_ = {};
};


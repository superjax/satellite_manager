#include <vector>
#include <stdint.h>
#include <experimental/filesystem>
#include <fstream>
#include <iostream>
#include <map>

#include "Eigen/Core"

extern "C" {
#include "rtklib.h"
}

namespace fs = std::experimental::filesystem;

#define GPS_EPHEMERIS_ARRAY_SIZE NSATGPS
#define GLONASS_EPHEMERIS_ARRAY_SIZE (NSATGLO)
#define SBAS_EPHEMERIS_ARRAY_SIZE NSATSBS

using namespace Eigen;
using namespace std;

typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 6> Matrix6d;
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
  double adjustPseudorange();

  // Speed of light
  static constexpr double C = 299792458.0; // m/s
  // Earth's rotation rate
  static constexpr double OMEGA_E = 7.2921151467e-5; //(rad/s);

  /**
   * @brief lsPositioning
   * calculate position with least squares
   * @param state [pos, vel]
   */
  bool recevierPos(Vector6d &state, Matrix3d& Qp, Matrix3d& Qv, gtime_t& time);

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

  void calcSatPosition(const eph_t& eph, const gtime_t& time, Vector3d& pos, Vector3d& vel, bool compute_harmonic_correction);


private:
  void log();
  void updateWavelen();
  void updateGlonassFrequencies();
  double calcPseudorange();
  nav_t nav_; // navigation data (used for calculating position, created from ephemeris)

  // buffer for RTKLIB (column indices aligned with obs_vec)
  std::vector<obsd_t> obs_vec_; // vector of most recent observations for each satellite
  Matrix<double, 6, -1> rs_; // position and velocity of each sat [pos, vel] (m, m/s) ECEF
  Matrix<double, 2, -1> dts_; // bias and clock drift of each sat [bias, clock drift] (s, s/s)
  Matrix<double, 1, -1> var_; // variance (m^2)
  Matrix<int, 1, -1> svh_; // satellite health flag

  // valid satellites only
  std::vector<uint8_t> current_sats_; // list of currently tracked satellites with valid position and velocity
  gtime_t start_ = {}; // start time (used in logging)
  gtime_t current_time_; // current time that all states are calculated wrt.

  std::ofstream file_; // output file
  prcopt_t opt_ = prcopt_default;
  sol_t sol_ = {};

  // matrices for only valid satellite information
  Matrix<double, 3, -1> xs_;
  Matrix<double, -1, 1> rho_;
  Matrix<double, 3, -1> e_; // unit vector to each satellite

  Map<Vector6d> x_; // [pos, vel]
  Matrix6d P_;
};


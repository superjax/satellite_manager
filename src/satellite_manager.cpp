#include "satellite_manager/satellite_manager.h"

#include <Eigen/Dense>

extern "C" {
#include "rtklib.h"
}



SatelliteManager::SatelliteManager() :
  x_(sol_.rr)
{
    nav_.eph = new eph_t[GPS_EPHEMERIS_ARRAY_SIZE];
    nav_.nmax = nav_.n = GPS_EPHEMERIS_ARRAY_SIZE;
    for (int i = 0; i < nav_.nmax; i++)
    {
        memset(&nav_.eph[i], 0, sizeof(eph_t));
        nav_.eph[i].iode = -1;
        nav_.eph[i].iodc = -1;
    }

    nav_.geph = new geph_t[GLONASS_EPHEMERIS_ARRAY_SIZE];
    nav_.ngmax = nav_.ng = GLONASS_EPHEMERIS_ARRAY_SIZE;
    for (int i = 0; i < nav_.ngmax; i++)
    {
        memset(&nav_.geph[i], 0, sizeof(geph_t));
        nav_.geph[i].iode = -1;
    }

    nav_.seph = new seph_t[SBAS_EPHEMERIS_ARRAY_SIZE];
    nav_.nsmax = nav_.ns = SBAS_EPHEMERIS_ARRAY_SIZE;
    for (int i = 0; i < nav_.nsmax; i++)
    {
        memset(&nav_.seph[i], 0, sizeof(seph_t));
    }
    P_.setZero();
    x_.setZero();
    sol_.rr[0] = -1798901.3;
    sol_.rr[1] = -4532226.61;
    sol_.rr[2] = 4099783.7;
    updateWavelen();
}

SatelliteManager::~SatelliteManager()
{
    if (file_.is_open())
        file_.close();
    delete nav_.geph;
    delete nav_.eph;
    delete nav_.seph;
}

void SatelliteManager::startLog(std::string file)
{
    if (!fs::is_directory("../logs") || !fs::exists("../logs"))
    {
        fs::create_directory("../logs");
    }

    file_.open("../logs/" + file);
}


void SatelliteManager::addEphemeris(const eph_t& eph)
{
    int idx = eph.sat;
    if (idx > NSATGPS)
    {
        // in 0 based indexes, it is gps, then glonass, then everything else. Glonass ephemeris is handled elsewhere, so we need to
        //  subtract the range of glonass indexes to get a 0 based non-Glonass epehmeris index.
        idx -= NSATGLO;
    }
    // prn is 1 based, make it 0 based for array
    idx--;
    if (idx >= 0)
    {
        if (nav_.eph[idx].iode != eph.iode || timediff(eph.toe, nav_.eph[idx].toe) > 0.0)
        {
            nav_.eph[idx] = eph;
            updateWavelen();
        }

    }
}

void SatelliteManager::addEphemeris(const geph_t& eph)
{
    int prn;
    satsys(eph.sat, &prn);
    int idx = prn - 1;
    if (idx >= 0)
    {
        if (nav_.geph[idx].iode != eph.iode)
        {
            nav_.geph[idx] = eph;
            updateWavelen();
            updateGlonassFrequencies();
        }
    }
}

void SatelliteManager::addObservation(const obsd_t& obs)
{
    bool new_sat = true;

    if (start_.time == 0)
    {
        start_.time = obs.time.time;
        start_.sec = obs.time.sec;
    }

    for (int i = 0; i < obs_vec_.size(); i++)
    {
        if (obs_vec_[i].sat == obs.sat && timediff(obs_vec_[i].time, obs.time) < 0)
        {
            obs_vec_[i] = obs;
            new_sat = false;
        }
    }
    if (new_sat)
    {
        obs_vec_.push_back(obs);
    }
}

void SatelliteManager::calcSatPosition(const eph_t &eph, const gtime_t &time, Vector3d &pos, Vector3d &vel, bool compute_harmonic_correction=false)
{

}

void SatelliteManager::update(gtime_t t)
{
    int n = obs_vec_.size();
    rs_.resize(6, n);
    dts_.resize(2, n);
    var_.resize(1, n);
    svh_.resize(1, n);
    satposs(t, obs_vec_.data(), obs_vec_.size(), &nav_, EPHOPT_BRDC,
            rs_.data(), dts_.data(), var_.data(), svh_.data());

    // make a list of valid sats
    current_sats_.clear();
    for (int i = 0; i < obs_vec_.size(); i++)
    {
        if ((rs_.block<3,1>(0, i).array() != 0).any())
            current_sats_.push_back(i);
    }

    adjustPseudorange();
    current_time_ = t;

    Matrix3d Pp, Pv;
    P_.block<3,3>(0,0) = Pp;
    P_.block<3,3>(3,3) = Pv;
    log();
}

void SatelliteManager::log()
{
    if (file_.is_open())
    {
        int i;
        int n = obs_vec_.size();
        double N = 2 + 11 * n;
        double t = timediff(current_time_, start_);
        file_.write((char*)&N, sizeof(double));
        file_.write((char*)&t, sizeof(double));
        for (i = 0; i < n; i++)
        {
            double sat = obs_vec_[i].sat;
            file_.write((char*)&sat, sizeof(double));
            file_.write((char*)(rs_.data()+6*i), sizeof(double)*6);
            file_.write((char*)(dts_.data()+2*i), sizeof(double)*2);
            file_.write((char*)(var_.data()+i), sizeof(double));
            double svh = svh_(i);
            file_.write((char*)(&svh), sizeof(double));
        }
        file_.flush();
    }
}

bool SatelliteManager::getSatState(int sat_id, Vector8d& state, double& var,
                                   int& health)
{
    int i;
    for (i = 0; i < obs_vec_.size(); i++)
    {
        if (sat_id == obs_vec_[i].sat)
            break;
    }

    if (i == obs_vec_.size())
        return false;

    if (rs_(0,i) == 0)
        return false;

    state.segment<6>(0) = rs_.col(i);
    state.segment<2>(6) = dts_.col(i);
    var = var_(i);
    health = svh_(i);
}

bool SatelliteManager::recevierPos(Vector6d &state, Matrix3d& Qp, Matrix3d& Qv,
                                   gtime_t& time)
{
  if (current_sats_.size() < 4)
    return false;

  MatrixXd A(current_sats_.size(), 4);
  MatrixXd b(current_sats_.size(), 1);
  MatrixXd est_psuedoranges(current_sats_.size(), 1);

  A.rightCols(1).setConstant(1.0);

  Map<const Vector3d> pos(sol_.rr);
  Vector4d x;
  x.segment<3>(0) = pos;
  x(3,0) = CLIGHT*sol_.dtr[0];

  Vector4d xprev;
  xprev.setZero();
  while ((x - xprev).norm() > 1e-3)
  {
    xprev = x;
    adjustPseudorange();
    A.leftCols(3) = -e_.transpose();
    for (int i = 0; i < current_sats_.size(); i++)
      est_psuedoranges(i,0) = (x.segment<3>(0) - xs_.col(i)).norm();
    b = rho_ - est_psuedoranges;
    x = A.colPivHouseholderQr().solve(b);

  }
  return true;
}

double SatelliteManager::adjustPseudorange()
{
  rho_.resize(current_sats_.size(), 1);
  e_.resize(3, current_sats_.size());
  xs_.resize(3, current_sats_.size());

  Vector3d lla;
  ecef2pos(x_.data(), lla.data());
  for (int i = 0; i < current_sats_.size(); i++)
  {
    int obs_id = current_sats_[i];
    Vector2d azel;
    Vector2d zazel{0.0,90.0*D2R};
    xs_.col(i) = rs_.block<3,1>(0, obs_id);

    double rhat = geodist(xs_.data()+3*i, x_.data(), e_.data()+3*i);
    if (rhat < 0.0)
      continue;

    double elevation = satazel(lla.data(), e_.data()+3*i, azel.data());
    if (elevation < opt_.elmin)
      continue;

    rhat += -CLIGHT*dts_(0,i);

    double zhd = tropmodel(obs_vec_[0].time, lla.data(), zazel.data(), 0.0);
    double trop_comp = tropmapf(obs_vec_[obs_id].time, lla.data(), azel.data(), NULL) * zhd;
    rhat += trop_comp;
    rho_(i, 0) = rhat;
  }
}

void SatelliteManager::updateWavelen()
{
	for (int i = 0; i < MAXSAT; i++)
	{
		nav_.lam[i][0] = satwavelen(i + 1, 0, &nav_);
	}
}

void SatelliteManager::updateGlonassFrequencies()
{
	for (int i = 0; i < MAXPRNGLO; i++)
	{
		int sat = satno(SYS_GLO, i + 1);
		int frq;

		for (int j = 0, frq = -999; j < 3; j++)
		{
			if (nav_.geph[i].sat != sat)
			{
				continue;
			}
			frq = nav_.geph[i].frq;
		}
		if (frq < -7 || frq > 6)
		{
			continue;
		}

		for (int j = 0; j < 3; j++)
		{
			if (nav_.geph[i].sat == sat)
			{
				continue;
			}
			nav_.geph[i].sat = sat;
			nav_.geph[i].frq = frq;
		}
	}
}


#include "satellite_manager/satellite_manager.h"

extern "C" {
#include "rtklib.h"
}

SatelliteManager::SatelliteManager()
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
}

SatelliteManager::~SatelliteManager()
{
    if (file_.is_open())
        file_.close();
    delete nav_.geph;
    delete nav_.eph;
    delete nav_.seph;
    delete obs_.data;
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
			nav_.eph[idx] = eph;
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

void SatelliteManager::update(gtime_t t)
{
  int n = obs_vec_.size();
  rs_.resize(6, n);
  dts_.resize(2, n);
  var_.resize(1, n);
  svh_.resize(1, n);
  satposs(t, obs_vec_.data(), obs_vec_.size(), &nav_, EPHOPT_BRDC,
      rs_.data(), dts_.data(), var_.data(), svh_.data());
  current_time_ = t;

  // make a list of valid sats
  current_sats_.clear();
  for (int i = 0; i < obs_vec_.size(); i++)
  {
      if ((rs_.block<3,1>(0, i).array() != 0).any())
          current_sats_.push_back(obs_vec_[i].sat);
  }

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

bool SatelliteManager::getSatState(int sat_id, Vector8d& state, double& var, int& health)
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

    t = obs_vec_[i].time;
    state.segment<6>(0) = rs_.col(i);
    state.segment<2>(6) = dts_.col(i);
    var = var_(i);
    health = svh_(i);
}


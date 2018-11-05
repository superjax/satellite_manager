#include "satellite_manager/satellite_manager.h"

SatelliteManager::SatelliteManager()
{

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
  for (int i = 0; i < obs_.size(); i++)
  {
    if (obs_[i].sat == obs.sat && timediff(obs_[i].time, obs.time) < 0)
    {
      obs_[i] = obs;
      new_sat = false;
    }
  }
  if (new_sat)
  {
    obs_.push_back(obs);
  }
}

void SatelliteManager::update()
{
  gtime_t now;
  int n = obs_.size();
  rs_.resize(6*n);
  dts_.resize(2*n);
  var_.resize(n);
  svh_.resize(MAXOBS * 2);
  satposs(obs_[0].time, obs_.data(), obs_.size(), &nav_, EPHOPT_BRDC,
      rs_.data(), dts_.data(), var_.data(), svh_.data());

}

void SatelliteManager::getSatState(int sat_id, const gtime_t& t, Vector3d& posECEF, Vector3d& velECEF)
{

}


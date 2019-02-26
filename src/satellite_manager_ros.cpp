#include "satellite_manager/satellite_manager_ros.h"


ros::Time gtime2ROS(const gtime_t& time)
{
  ros::Time out;
  out.sec = time.time;
  out.nsec = time.sec * 1e9;
  return out;
}

gtime_t ros2gtime(const ros::Time& time)
{
  gtime_t out;
  out.sec = time.nsec*1e-9;
  out.time = time.sec;
  return out;
}

SatelliteManagerROS::SatelliteManagerROS()
{
  sat_pub_ = nh_.advertise<satellite_manager::SatellitePosition>("sat_pos", 25);
  rec_pub_ = nh_.advertise<satellite_manager::PositionVelocity>("rec_pos", 1);
  prev_time_.sec = 0;
  prev_time_.time = 0;

  obs_sub_ = nh_.subscribe("gps/obs", 50, &SatelliteManagerROS::obsCallback, this);
  eph_sub_ = nh_.subscribe("gps/eph", 50, &SatelliteManagerROS::ephCallback, this);
  geph_sub_ = nh_.subscribe("gps/geph", 50, &SatelliteManagerROS::gephCallback, this);
}

void SatelliteManagerROS::ephCallback(const inertial_sense::GNSSEphemerisConstPtr &msg)
{
  eph_t eph;
  eph.sat = msg->sat;
  eph.iode = msg->iode;
  eph.iodc = msg->iodc;
  eph.sva = msg->sva;
  eph.svh = msg->svh;
  eph.week = msg->week;
  eph.code = msg->code;
  eph.flag = msg->flag;
  eph.toe.sec = msg->toe.sec;
  eph.toc.sec = msg->toc.sec;
  eph.ttr.sec = msg->ttr.sec;
  eph.toe.time = msg->toe.time;
  eph.toc.time = msg->toc.time;
  eph.ttr.time = msg->ttr.time;
  eph.A = msg->A;
  eph.e = msg->e;
  eph.i0 = msg->i0;
  eph.OMG0 = msg->OMG0;
  eph.omg = msg->omg;
  eph.M0 = msg->M0;
  eph.deln = msg->deln;
  eph.OMGd = msg->OMGd;
  eph.idot = msg->idot;
  eph.crc = msg->crc;
  eph.crs = msg->crs;
  eph.cuc = msg->cuc;
  eph.cus = msg->cus;
  eph.cic = msg->cic;
  eph.cis = msg->cis;
  eph.toes = msg->toes;
  eph.fit = msg->fit;
  eph.f0 = msg->f0;
  eph.f1 = msg->f1;
  eph.f2 = msg->f2;
  eph.tgd[0] = msg->tgd[0];
  eph.tgd[1] = msg->tgd[1];
  eph.tgd[2] = msg->tgd[2];
  eph.tgd[3] = msg->tgd[3];
  eph.Adot = msg->Adot;
  eph.ndot = msg->ndot;
  sat_manager_.addEphemeris(eph);
}

void SatelliteManagerROS::gephCallback(const inertial_sense::GlonassEphemerisConstPtr &msg)
{
  geph_t geph;
  geph.sat = msg->sat;
  geph.iode = msg->iode;
  geph.frq = msg->frq;
  geph.svh = msg->svh;
  geph.sva = msg->sva;
  geph.age = msg->age;
  geph.toe.time = msg->toe.time;
  geph.toe.sec = msg->toe.sec;
  geph.tof.time = msg->tof.time;
  geph.tof.sec = msg->tof.sec;
  geph.pos[0] = msg->pos[0];
  geph.vel[1] = msg->vel[1];
  geph.acc[2] = msg->acc[2];
  geph.taun = msg->taun;
  geph.gamn = msg->gamn;
  geph.dtaun = msg->dtaun;
  sat_manager_.addEphemeris(geph);
}

void SatelliteManagerROS::obsCallback(const inertial_sense::GNSSObservationConstPtr &msg)
{
  obsd_t obsd;
  obsd.time.sec = msg->time.sec;
  obsd.time.time = msg->time.time;
  obsd.eventime.sec = msg->time.sec;
  obsd.eventime.time = msg->time.time;
  obsd.timevalid = true;
  obsd.sat = msg->sat;
  obsd.rcv = msg->rcv;
  obsd.SNR[0] = msg->SNR;
  obsd.LLI[0] = msg->LLI;
  obsd.code[0] = msg->code;
  obsd.qualL[0] = msg->qualL;
  obsd.qualP[0] = msg->qualP;
  obsd.L[0] = msg->L;
  obsd.P[0] = msg->P;
  obsd.D[0] = msg->D;
  if (timediff(obsd.time, prev_time_) > 0.001)
  {
    update(gtime2ROS(prev_time_));
    prev_time_.sec = obsd.time.sec;
    prev_time_.time = obsd.time.time;
  }
  sat_manager_.addObservation(obsd);
}

void SatelliteManagerROS::update(const ros::Time& t)
{
  // publish state of satellites
  Vector8d state;

  double var;
  int svh;
  sat_manager_.update(ros2gtime(t));
  std::vector<uint8_t> sat_ids = sat_manager_.satIds();
  for (int i = 0; i < sat_ids.size(); i++)
  {
    if (sat_manager_.getSatState(sat_ids[i], state, var, svh))
    {
      satellite_manager::SatellitePosition sat_pos_msg;
      sat_pos_msg.header.stamp = t;
      sat_pos_msg.sat_id = sat_ids[i];
      sat_pos_msg.position.x = state(0);
      sat_pos_msg.position.y = state(1);
      sat_pos_msg.position.z = state(2);
      sat_pos_msg.velocity.x = state(3);
      sat_pos_msg.velocity.y = state(4);
      sat_pos_msg.velocity.z = state(5);
      sat_pos_msg.clock_bias = state(6);
      sat_pos_msg.clock_drift = state(7);
      sat_pub_.publish(sat_pos_msg);
    }
  }
  Vector6d receiver_pos_vel;
  Matrix3d Qp, Qv;
  gtime_t trec;
  if(sat_manager_.recevierPos(receiver_pos_vel, Qp, Qv, trec))
  {
    satellite_manager::PositionVelocity pos_vel_msg;
    pos_vel_msg.header.stamp = gtime2ROS(trec);
    pos_vel_msg.posECEF.x = receiver_pos_vel(0);
    pos_vel_msg.posECEF.y = receiver_pos_vel(1);
    pos_vel_msg.posECEF.z = receiver_pos_vel(2);
    pos_vel_msg.velECEF.x = receiver_pos_vel(3);
    pos_vel_msg.velECEF.y = receiver_pos_vel(4);
    pos_vel_msg.velECEF.x = receiver_pos_vel(5);

    Map<Matrix6d> cov(pos_vel_msg.cov.data());
    cov.setZero();
    cov.block<3,3>(0,0) = Qp;
    cov.block<3,3>(3,3) = Qv;
    rec_pub_.publish(pos_vel_msg);
  }
}


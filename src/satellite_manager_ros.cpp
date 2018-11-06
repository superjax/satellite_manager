#include "satellite_manager/satellite_manager_ros.h"


SatelliteManagerROS::SatelliteManagerROS(){}

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
    update();
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
    geph.pos[3] = msg->pos[3];
    geph.vel[3] = msg->vel[3];
    geph.acc[3] = msg->acc[3];
    geph.taun = msg->taun;
    geph.gamn = msg->gamn;
    geph.dtaun = msg->dtaun;
    sat_manager_.addEphemeris(geph);
    update();
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
    sat_manager_.addObservation(obsd);
    update();
}

void SatelliteManagerROS::update()
{
    sat_manager_.update();

    // publish state of satellites
}



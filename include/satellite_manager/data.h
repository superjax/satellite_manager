#include <stdint.h>

/** time struct */
typedef struct
{
    /** time (s) expressed by standard time_t */
    int64_t time;

    /** fraction of second under 1 s */
    double sec;
} gtime_t;

/** non-Glonass ephemeris data */
typedef struct
{
    /** satellite number */
    int32_t sat;

    /** IODE Issue of Data, Ephemeris (ephemeris version) */
    int32_t iode;

    /** IODC Issue of Data, Clock (clock version) */
    int32_t iodc;

    /** SV accuracy (URA index) IRN-IS-200H p.97 */
    int32_t sva;

    /** SV health GPS/QZS (0:ok) */
    int32_t svh;

    /** GPS/QZS: gps week, GAL: galileo week */
    int32_t week;

    /** GPS/QZS: code on L2
     * (00=Invalid, 01 = P Code ON, 11 = C/A code ON, 11 = Invalid)
     * GAL/CMP: data sources */
    int32_t code;

    /** GPS/QZS: L2 P data flag (indicates that the NAV data stream was commanded OFF on the P-code of the in-phase component of the L2 channel)
     *
     *  CMP: nav type */
    int32_t flag;

    /** Toe */
    gtime_t toe;

    /** clock data reference time (s) (20.3.4.5) */
    gtime_t toc;

    /** T_trans */
    gtime_t ttr;

    /** Semi-Major Axis m */
    double A;

    /** Eccentricity (no units)  */
    double e;

    /** Inclination Angle at Reference Time (rad) */
    double i0;

    /** Longitude of Ascending Node of Orbit Plane at Weekly Epoch (rad) */
    double OMG0;

    /** Argument of Perigee (rad) */
    double omg;

    /** Mean Anomaly at Reference Time (rad) */
    double M0;

    /** Mean Motion Difference From Computed Value (rad) */
    double deln;

    /** Rate of Right Ascension (rad/s) */
    double OMGd;

    /** Rate of Inclination Angle (rad/s) */
    double idot;

    /** Amplitude of the Cosine Harmonic Correction Term to the Orbit Radius */
    double crc;

    /** Amplitude of the Sine Harmonic Correction Term to the Orbit Radius (m) */
    double crs;

    /** Amplitude of the Cosine Harmonic Correction Term to the Argument of Latitude (rad)  */
    double cuc;

    /** Amplitude of the Sine Harmonic Correction Term to the Argument of Latitude (rad) */
    double cus;

    /** Amplitude of the Cosine Harmonic Correction Term to the Angle of Inclination (rad) */
    double cic;

    /** Amplitude of the Sine Harmonic Correction Term to the Angle of Inclination (rad) */
    double cis;

    /** Reference Time Ephemeris in week (s) */
    double toes;

    /** fit interval (h) (0: 4 hours, 1:greater than 4 hours) */
    double fit;

    /** SV clock parameters - af0 */
    double f0;

    /** SV clock parameters - af1 */
    double f1;

    /** SV clock parameters - af2 */
    double f2;

    /** group delay parameters
    * GPS/QZS:tgd[0]=TGD (IRN-IS-200H p.103)
    * GAL    :tgd[0]=BGD E5a/E1,tgd[1]=BGD E5b/E1
    * CMP    :tgd[0]=BGD1,tgd[1]=BGD2
    */
    double tgd[4];

    /** Adot for CNAV */
    double Adot;

    /** ndot for CNAV */
    double ndot;
} eph_t;

/** Glonass ephemeris data */
typedef struct
{
    /** satellite number */
    int32_t sat;

    /** IODE (0-6 bit of tb field) */
    int32_t iode;

    /** satellite frequency number */
    int32_t frq;

    /** satellite health */
    int32_t svh;

    /** satellite accuracy */
    int32_t sva;

    /** satellite age of operation */
    int32_t age;

    /** epoch of epherides (gpst) */
    gtime_t toe;

    /** message frame time (gpst) */
    gtime_t tof;

    /** satellite position (ecef) (m) */
    double pos[3];

    /** satellite velocity (ecef) (m/s) */
    double vel[3];

    /** satellite acceleration (ecef) (m/s^2) */
    double acc[3];

    /** SV clock bias (s) */
    double taun;

    /** relative freq bias */
    double gamn;

    /** delay between L1 and L2 (s) */
    double dtaun;
} geph_t;

typedef struct {        /* SBAS ephemeris type */
    int sat;            /* satellite number */
    gtime_t t0;         /* reference epoch time (GPST) */
    gtime_t tof;        /* time of message frame (GPST) */
    int sva;            /* SV accuracy (URA index) */
    int svh;            /* SV health (0:ok) */
    double pos[3];      /* satellite position (m) (ecef) */
    double vel[3];      /* satellite velocity (m/s) (ecef) */
    double acc[3];      /* satellite acceleration (m/s^2) (ecef) */
    double af0,af1;     /* satellite clock-offset/drift (s,s/s) */
} seph_t;

typedef struct {        /* earth rotation parameter data type */
    double mjd;         /* mjd (days) */
    double xp,yp;       /* pole offset (rad) */
    double xpr,ypr;     /* pole offset rate (rad/day) */
    double ut1_utc;     /* ut1-utc (s) */
    double lod;         /* length of day (s/day) */
} erpd_t;

typedef struct {        /* earth rotation parameter type */
    int n,nmax;         /* number and max number of data */
    erpd_t *data;       /* earth rotation parameter data */
} erp_t;

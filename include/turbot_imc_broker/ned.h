#ifndef INCLUDE_TURBOT_IMC_BROKER_NED_H_
#define INCLUDE_TURBOT_IMC_BROKER_NED_H_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>

Eigen::Vector3d getRPY(const Eigen::Matrix3d& rotation);

Eigen::Quaterniond euler2Quaternion(const double roll, const double pitch, const double yaw);

double dms2Deg(const double degree_minutes, const char hemisphere);

double dms2DegInt(const double degree_minutes, const int hemisphere);

double rad2Deg(const double radians);

double deg2Rad(const double degrees);

double normalizeAngle(const double angle);

static const double a = 6378137;
static const double b = 6356752.3142;
static const double esq = 6.69437999014 * 0.001;
static const double e1sq = 6.73949674228 * 0.001;
static const double f = 1 / 298.257223563;

class Ned
{
 public:
    Ned(const double lat, const double lon, const double height);

    void geodetic2Ecef(const double lat, const double lon, const double height,
                       double& x, double& y, double& z);

    void ecef2Geodetic(const double x, const double y, const double z,
                       double& lat, double& lon, double& height);

    void ecef2Ned(const double x, const double y, const double z,
                  double& north, double& east, double& depth);

    void ned2Ecef(const double north, const double east, const double depth,
                  double& x, double& y, double& z);

    void geodetic2Ned(const double lat, const double lon, const double height,
                      double& north, double& east, double& depth);

    void ned2Geodetic(const double north, const double east, const double depth,
                      double& lat, double& lon, double& height);

    inline double getLat() {return rad2Deg(_init_lat);}
    inline double getLon() {return rad2Deg(_init_lon);}

 private:
    double _init_lat;
    double _init_lon;
    double _init_h;
    double _init_ecef_x;
    double _init_ecef_y;
    double _init_ecef_z;
    Eigen::Matrix3d _ecef_to_ned_matrix;
    Eigen::Matrix3d _ned_to_ecef_matrix;

    double __cbrt__(const double x);
    Eigen::Matrix3d __nRe__(const double lat_rad, const double lon_rad);
};

#endif  // INCLUDE_TURBOT_IMC_BROKER_NED_H_

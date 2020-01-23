#ifndef __PC_ATPS_HEADER__
#define __PC_ATPS_HEADER__

/***********************************************************************
PC_ATPS
created:	2019/06
author:		xz_zhu
purpose:	PC_ATPS.dll
************************************************************************/



/*
   example of useage:
   -----------------------------------------------------------------------------------------------
   std::vector<ATPS::SVPoint3d> points;
   *read the input points into [points]

   ATPS::ATPS_Plane ATPS_plane(kappa_t, delta_t, tau_t, gamma_t, epsilon_t, theta_t, iter_t);
   
   double res = 0.0;
   res = ATPS_plane.get_res(points);
   ATPS_plane.set_parameters(res);

   std::vector<std::vector<ATPS::SVPoint3d>> planar_points;
   std::vector<ATPS::SVPoint3d> nonplanar_points;
   std::vector<std::vector<double>> model_coefficients;

   ATPS_plane.ATPS_PlaneSegmentation(points, planar_points, nonplanar_points, model_coefficients);
   -----------------------------------------------------------------------------------------------
*/



/*!
 * \file PC_ATPS.h
 * \brief plane segmentation dll header file
 *
 */

#include <vector>

#ifndef ATPS_NAMESPACE_BEGIN
#define ATPS_NAMESPACE_BEGIN namespace ATPS {
#endif
#ifndef ATPS_NAMESPACE_END
#define ATPS_NAMESPACE_END }
#endif

ATPS_NAMESPACE_BEGIN

#ifndef SVPOINT3D
#define SVPOINT3D
class SVPoint3d
{
public:
	SVPoint3d()
	{
		x = 0.0;
		y = 0.0;
		z = 0.0;
		r = 0;
		g = 0;
		b = 0;
	}
	~SVPoint3d(){}
	SVPoint3d(double _x, double _y, double _z) :
		x(_x), y(_y), z(_z) {}
	SVPoint3d(double _x, double _y, double _z, unsigned int _r, unsigned int _g, unsigned int _b) :
		x(_x), y(_y), z(_z), r(_r), g(_g), b(_b) {}
	double X() { return x; }
	double Y() { return y; }
	double Z() { return z; }

public:
	double x;
	double y;
	double z;
	uint8_t r;
	uint8_t g;
	uint8_t b;
};
#endif

#define USE_ATPS_AS_DLL

#if defined(USE_ATPS_AS_DLL) && defined(_WIN32)
#ifdef PC_ATPS_EXPORTS
#define PC_ATPS_API __declspec(dllexport)
#else
#define PC_ATPS_API __declspec(dllimport)
#endif
#else
#define PC_ATPS_API
#endif


/*!
 * \class ATPS_Plane
 * \brief Interface to Class ATPS_Plane
 * \usage 
 *			ATPS_Plane ATPS_plane;
 *
 *			ATPS_plane.SetPoints(); 
 *			ATPS_plane.PlaneSegmentation();
 *			...
 *
 */
class PC_ATPS_API ATPS_Plane
{
public:
	ATPS_Plane();

	ATPS_Plane(int kappa_, double delta_, double tau_, double gamma_, double epsilon_, double theta_, int iter_);

	ATPS_Plane(const double res);

	~ATPS_Plane();


	int get_kappa() const;
	double get_delta() const;
	double get_tau() const;
	double get_gamma() const;
	double get_epsilon() const;
	double get_theta() const;
	int get_iter() const;

	double get_res(const std::vector<SVPoint3d> points) const;


	void set_parameters(const int kappa_, const double delta_, const double tau_, const double gamma_, const double epsilon_, const double theta_, const int iter_);
	void set_parameters(const double res);
	void set_kappa(const int kappa_);
	void set_delta(const double delta_);
	void set_tau(const double tau_);
	void set_gamma(const double gamma_);
	void set_epsilon(const double epsilon_);
	void set_theta(const double theta_);
	void set_iter(const int iter_);


	bool set_points(const std::string pc_path, std::vector<SVPoint3d>& points);


    // segment the input point cloud into planes
	bool ATPS_PlaneSegmentation(
		const std::vector<SVPoint3d> points,
		std::vector<std::vector<SVPoint3d>>& planar_points,
		std::vector<SVPoint3d>& nonplanar_points, 
		std::vector<std::vector<double>>& model_coefficients);

    // segment the input point cloud into planar supervoxels
	bool ATPS_RPSGeneration(
		const std::vector<SVPoint3d> points, 
		std::vector<std::vector<SVPoint3d>>& RPS_points, 
		std::vector<SVPoint3d>& IP_points, 
		std::vector<std::vector<double>>& model_coefficients);


	bool write_planes_ply(
		std::string ply_path,
		const std::vector<std::vector<SVPoint3d>> planar_points,
		const std::vector<SVPoint3d> nonplanar_points);


private:
	// parameters

	double r_max;
	double r_min;
	double r_delta;

	// threashold

	int kappa_t;
	double delta_t;
	double tau_t;
	double gamma_t;
	double epsilon_t;
	double theta_t;
	int iter_t;

	/*
	*---description of parameters---*
	kappa_t: the minimum point number for a valid plane. (1.0/res)
	delta_t: the threshold of curvature for multi-scale supervoxel segmentation. (0.05)
	tau_t: the threshold of distance tolerance value for point-to-plane and plane-to-plane. (0.1)
	gamma_t: the threshold of neighborhood for point-to-plane and plane-to-plane. (res*7.0)
	epsilon_t: the threshold of NFA tolerance value for a-contrario rigorous planar supervoxel generation. (0.0)
	theta_t: the threshold of normal vector angle for hybrid region growing. (10.0)
	iter_t: the iteration type of independent points reclassification. (0/1)
	*/
};

ATPS_NAMESPACE_END

#endif // __PC_ATPS_HEADER__
#include "ParKine.hpp"
#include "ParallelAnkleSolver.hpp"
#include <limits>
#include <math.h>

//parallel kinematic parameters for jaka pi
#define L 27.0
#define x1 7.5
#define y1 11.5
#define z1 -40.55
#define z2 -21.1
#define x3 -200.0
#define x4 -117.0
#define L1 209.0
#define L2 127.0
#define ITER_THRESHOLD  1e-10
#define MAX_ITER  10
#define PI 3.14159265358979323846

// Stateless bounded assembly solve shared by policy feedback and the sim SDK.
// Keep the public C ABI: the packaged simulation SDK calls this through its PLT.
// Failure outputs are deliberately invalid; callers must stop instead of using
// fabricated zero angles or a stale estimate.
int AB2PitchRoll(double a, double b, double da, double db,
                 double *pitch, double *roll, double *dpitch, double *droll,
                 double J[2][2])
{
    if (!pitch || !roll || !dpitch || !droll || !J)
        return static_cast<int>(jaka::AnkleSolveStatus::InvalidInput);
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    *pitch = *roll = *dpitch = *droll = invalid;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) J[i][j] = invalid;
    const auto result = jaka::solve_parallel_ankle(a, b, da, db);
    if (!result.ok()) return static_cast<int>(result.status);
    *pitch = result.pitch; *roll = result.roll;
    *dpitch = result.dpitch; *droll = result.droll;
    for (int i = 0; i < 2; ++i)
        for (int j = 0; j < 2; ++j) J[i][j] = result.J[i][j];
    return result.iterations;
}

void PitchRoll2AB(double pitch,double roll, double* a,double* b)
{
    const double sin_pitch=sin(pitch);
    const double cos_pitch=cos(pitch);
    const double sin_roll=sin(roll);
    const double cos_roll=cos(roll);

    double sin_term=- 2*L*y1*cos_roll - 2*L*x1*sin_roll;
    double cos_term=2*L*x3 + 2*L*z1*sin_pitch - 2*L*x1*cos_pitch*cos_roll + 2*L*y1*cos_pitch*sin_roll;
    double offset=  L*L + x1*x1 + x3*x3 + y1*y1 + z1*z1 + z2*z2 - 2*z1*z2*cos_pitch + 2*x3*z1*sin_pitch - 2*x1*x3*cos_pitch*cos_roll \
        + 2*x3*y1*cos_pitch*sin_roll - 2*x1*z2*cos_roll*sin_pitch + 2*y1*z2*sin_pitch*sin_roll - L1*L1;


    double phi=atan2(sin_term,cos_term);
    double sin_val=-offset/sqrt(sin_term*sin_term+cos_term*cos_term);
    if (sin_val>1.0)
        sin_val=1.0;
    else if (sin_val<-1.0)
        sin_val=-1.0;
    *a=-asin(sin_val)-phi-PI;
    if(*a<-PI)
        (*a)+=2*PI;

    sin_term=2*L*x1*sin_roll - 2*L*y1*cos_roll;
    cos_term=2*L*x1*cos_pitch*cos_roll- 2*L*z1*sin_pitch - 2*L*x4 + 2*L*y1*cos_pitch*sin_roll;
    offset=L*L + x1*x1 + x4*x4 + y1*y1 + z1*z1 + z2*z2 - 2*z1*z2*cos_pitch + 2*x4*z1*sin_pitch - 2*x1*x4*cos_pitch*cos_roll \
        - 2*x4*y1*cos_pitch*sin_roll - 2*x1*z2*cos_roll*sin_pitch - 2*y1*z2*sin_pitch*sin_roll-L2*L2;

    phi=atan2(sin_term,cos_term);
    sin_val=-offset/sqrt(sin_term*sin_term+cos_term*cos_term);
    if (sin_val>1.0)
        sin_val=1.0;
    else if (sin_val<-1.0)
        sin_val=-1.0;
    *b=asin(sin_val)-phi;

    return;
}

void Q2RPY(const double* quat,double* rpy)
{
    const double w=quat[0];
    const double x=quat[1];
    const double y=quat[2];
    const double z=quat[3];

    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    rpy[0] = atan2(t0, t1);


    t0 = 2.0 * (w * y - z * x);
    if(t0>1.0)
        t0=1.0;
    else if(t0<-1.0)
        t0=-1.0;
    rpy[1] = asin(t0);

    t0 = 2.0 * (w * z + x * y);
    t1 = 1.0 - 2.0 * (y * y + z * z);
    rpy[2] = atan2(t0, t1);

    return;
}


#define SIGN(x) ((x)>0?1.0:((x)<0?-1.0:0.0))
/************************* TD IMPLEMENTATION ******************************/
static inline void nlf2_step(void* obj,double x,double v)
{
	NLF2* tmp=(NLF2*)obj;
	double errx = tmp->x0-x;
	double errv=tmp->v0-v;
	double d=tmp->a_max*tmp->Ts;
	double d0=d*tmp->Ts;
	double y=errx+errv*tmp->Ts;
	double a;
	if (fabs(y)>d0)
	{
		a=errv+(sqrt(d*d+8*tmp->a_max*fabs(y))-d)/2.0*SIGN(y);
	}
	else
	{
		a=errv+y/tmp->Ts;
	}
	double fhan=a/d;
	if(fhan>1.0)
		fhan=1.0;
	else if(fhan<-1.0)
		fhan=-1.0;
	fhan*=-tmp->a_max;

	tmp->x0+=tmp->v0*tmp->Ts;
	double temp=tmp->v0+fhan*tmp->Ts;

	if(temp>tmp->v_max)
		temp=tmp->v_max;
	else if(temp<-tmp->v_max)
		temp=-tmp->v_max;

	tmp->v0=temp;

    return;
}


void NLF2Init(NLF2* nlf2,double Ts, double a_max,double v_max,double x0,double v0)
{
	nlf2->Ts=Ts;
	nlf2->a_max=a_max;
	nlf2->v_max=v_max;

	nlf2->x0=x0;
	nlf2->v0=v0;
	nlf2->step=&nlf2_step;

    return;
}






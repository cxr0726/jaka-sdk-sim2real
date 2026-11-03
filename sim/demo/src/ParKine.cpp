#include "ParKine.hpp"
#include "ParallelAnkleSolver.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <math.h>

namespace
{
constexpr double L=27., x1=7.5, kRodY1=11.5, z1=-40.55, z2=-21.1;
constexpr double x3=-200., x4=-117., L1=209., L2=127.;
constexpr double PI=3.14159265358979323846;
void invalidate(double *pitch,double *roll,double *dpitch,double *droll,double J[2][2])
{
    const double nan=std::numeric_limits<double>::quiet_NaN();
    if(pitch)*pitch=nan;
    if(roll)*roll=nan;
    if(dpitch)*dpitch=nan;
    if(droll)*droll=nan;
    if(J)for(int i=0;i<2;++i)for(int j=0;j<2;++j)J[i][j]=nan;
}
}

int AB2PitchRoll(double a,double b,double da,double db,double *pitch,double *roll,
                 double *dpitch,double *droll,double J[2][2])
{
    if(!pitch||!roll||!dpitch||!droll||!J)
    {
        invalidate(pitch,roll,dpitch,droll,J);
        return static_cast<int>(jaka::AnkleSolveStatus::InvalidInput);
    }
    const auto result=jaka::solve_parallel_ankle(a,b,da,db);
    if(!result.ok())
    {
        invalidate(pitch,roll,dpitch,droll,J);
        return static_cast<int>(result.status);
    }
    *pitch=result.pitch;*roll=result.roll;*dpitch=result.dpitch;*droll=result.droll;
    for(int i=0;i<2;++i)for(int j=0;j<2;++j)J[i][j]=result.J[i][j];
    return result.iterations;
}

int PitchRoll2ABChecked(double pitch,double roll,double *a,double *b)
{
    const auto fail=[&](jaka::AnkleSolveStatus status)
    {
        const double nan=std::numeric_limits<double>::quiet_NaN();
        if(a)*a=nan;
        if(b)*b=nan;
        return static_cast<int>(status);
    };
    if(!a||!b||!std::isfinite(pitch)||!std::isfinite(roll))
        return fail(jaka::AnkleSolveStatus::InvalidInput);
    const jaka::AnkleSolveOptions assembly;
    if(pitch<assembly.pitch_min||pitch>assembly.pitch_max||roll<assembly.roll_min||roll>assembly.roll_max)
        return fail(jaka::AnkleSolveStatus::OutsideAssembly);
    const double sp=std::sin(pitch),cp=std::cos(pitch),sr=std::sin(roll),cr=std::cos(roll);
    double sin_term=-2*L*kRodY1*cr-2*L*x1*sr;
    double cos_term=2*L*x3+2*L*z1*sp-2*L*x1*cp*cr+2*L*kRodY1*cp*sr;
    double offset=L*L+x1*x1+x3*x3+kRodY1*kRodY1+z1*z1+z2*z2-2*z1*z2*cp
        +2*x3*z1*sp-2*x1*x3*cp*cr+2*x3*kRodY1*cp*sr-2*x1*z2*cr*sp
        +2*kRodY1*z2*sp*sr-L1*L1;
    auto asin_argument=[](double offset,double sine,double cosine,double &value)
    {
        const double denominator=std::hypot(sine,cosine);
        if(!std::isfinite(denominator)||denominator<=1.e-12)return false;
        value=-offset/denominator;
        // Permit only floating-point roundoff at an exactly reachable boundary.
        constexpr double roundoff=64.*std::numeric_limits<double>::epsilon();
        if(!std::isfinite(value)||std::abs(value)>1.+roundoff)return false;
        value=std::clamp(value,-1.,1.);return true;
    };
    double argument;
    if(!asin_argument(offset,sin_term,cos_term,argument))
        return fail(jaka::AnkleSolveStatus::OutsideAssembly);
    double qa=-std::asin(argument)-std::atan2(sin_term,cos_term)-PI;
    if(qa<-PI)qa+=2*PI;
    sin_term=2*L*x1*sr-2*L*kRodY1*cr;
    cos_term=2*L*x1*cp*cr-2*L*z1*sp-2*L*x4+2*L*kRodY1*cp*sr;
    offset=L*L+x1*x1+x4*x4+kRodY1*kRodY1+z1*z1+z2*z2-2*z1*z2*cp
        +2*x4*z1*sp-2*x1*x4*cp*cr-2*x4*kRodY1*cp*sr-2*x1*z2*cr*sp
        -2*kRodY1*z2*sp*sr-L2*L2;
    if(!asin_argument(offset,sin_term,cos_term,argument))
        return fail(jaka::AnkleSolveStatus::OutsideAssembly);
    const double qb=std::asin(argument)-std::atan2(sin_term,cos_term);
    if(!std::isfinite(qa)||!std::isfinite(qb))return fail(jaka::AnkleSolveStatus::InvalidInput);
    // Analytic inverse branches are only accepted when the checked forward
    // equation recovers the requested assembly. No projection into reachability.
    const auto check=jaka::solve_parallel_ankle(qa,qb,0.,0.);
    if(!check.ok())return fail(check.status);
    if(std::abs(check.pitch-pitch)>1.e-7||std::abs(check.roll-roll)>1.e-7)
        return fail(jaka::AnkleSolveStatus::OutsideAssembly);
    *a=qa;*b=qb;return 0;
}

void PitchRoll2AB(double pitch,double roll,double *a,double *b)
{
    (void)PitchRoll2ABChecked(pitch,roll,a,b);
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






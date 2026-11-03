#include "ParKine.hpp"
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

static void R(double pitch, double roll, double Rot[3][3], double Rpitch[3][3], double Rroll[3][3])
{
	const double sp = sin(pitch);
	const double cp = cos(pitch);
	const double sr = sin(roll);
	const double cr = cos(roll);

	Rot[0][0] = cp * cr;
	Rot[0][1] = -cp * sr;
	Rot[0][2] = -sp;
	Rot[1][0] = cr * sp;
	Rot[1][1] = -sp * sr;
	Rot[1][2] = cp;
	Rot[2][0] = -sr;
	Rot[2][1] = -cr;
	Rot[2][2] = 0.0;

	Rpitch[0][0] = -Rot[1][0];
	Rpitch[0][1] = -Rot[1][1];
	Rpitch[0][2] = -cp;
	Rpitch[1][0] = Rot[0][0];
	Rpitch[1][1] = Rot[0][1];
	Rpitch[1][2] = -sp;
	Rpitch[2][0] = 0.0;
	Rpitch[2][1] = 0.0;
	Rpitch[2][2] = 0.0;

	Rroll[0][0] = Rot[0][1];
	Rroll[0][1] = -Rot[0][0];
	Rroll[0][2] = 0.0;
	Rroll[1][0] = Rot[1][1];
	Rroll[1][1] = -Rot[1][0];
	Rroll[1][2] = 0.0;
	Rroll[2][0] = -cr;
	Rroll[2][1] = sr;
	Rroll[2][2] = 0.0;

}

static inline double pTRp(const double p1[3], double R[3][3], const double p2[3])
{
	double tmp[3];
	tmp[0] = p1[0] * R[0][0] + p1[1] * R[1][0] + p1[2] * R[2][0];
	tmp[1] = p1[0] * R[0][1] + p1[1] * R[1][1] + p1[2] * R[2][1];
	tmp[2] = p1[0] * R[0][2] + p1[1] * R[1][2] + p1[2] * R[2][2];

	double ret = tmp[0] * p2[0] + tmp[1] * p2[1] + tmp[2] * p2[2];
	return ret;
}

static inline double dot3(const double p1[3], const double p2[3])
{
	double ret = p1[0] * p2[0] + p1[1] * p2[1] + p1[2] * p2[2];
	return ret;
}

static inline int inv2(double J[2][2])
{
	double det = J[0][0] * J[1][1] - J[0][1] * J[1][0];
	if (fabs(det) > 1e-5)
	{
		double tmp = J[0][0];
		J[0][0] = J[1][1] / det;
		J[1][1] = tmp / det;
		J[0][1] = -J[0][1] / det;
		J[1][0] = -J[1][0] / det;
		return 0;
	}
	else
	{
		return -1;
	}
}

int  AB2PitchRoll(double a, double b, double da, double db, double* pitch, double* roll, double* dpitch, double* droll, double J[2][2])
{
	double sum = a + b;
	if (fabs(sum) > 1.0) // range check
	{
		sum = sum > 0.0 ? 1.0 : -1.0;
		b = sum - a;
	}
	
	const double sa = sin(a);
	const double ca = cos(a);
	const double sb = sin(b);
	const double cb = cos(b);

	const double p1[3] = { x1, y1, z1 };
	const double p2[3] = { x1, -y1, z1 };
	const double p3[3] = {x3 + L * sa, z2, -L * ca};
	const double p4[3] = {x4 - L * sb, z2, L * cb};

	const double p1_dot_p3 = (dot3(p1,p1) + dot3(p3,p3) - L1 * L1) / 2.0;
	const double p2_dot_p4 = (dot3(p2,p2) + dot3(p4,p4) - L2 * L2) / 2.0;

	*pitch = (a - b) / 2.0;
	*roll = -(a + b) / 2.0;
	double Rot[3][3], Rpitch[3][3], Rroll[3][3];
	R(*pitch, *roll, Rot, Rpitch, Rroll);

	double Jac[2][2],y[2], delta[2];
	y[0] = p1_dot_p3 - pTRp(p3, Rot, p1);
	y[1] = p2_dot_p4 - pTRp(p4, Rot, p2);

	Jac[0][0] = pTRp(p3, Rpitch, p1);
	Jac[0][1] = pTRp(p3, Rroll, p1);
	Jac[1][0] = pTRp(p4, Rpitch, p2);
	Jac[1][1] = pTRp(p4, Rroll, p2);
	if (!inv2(Jac))
	{
		delta[0] = Jac[0][0] * y[0] + Jac[0][1] * y[1];
		delta[1] = Jac[1][0] * y[0] + Jac[1][1] * y[1];
	}
	else
	{
		//singular
		*dpitch = 0.0;
		*droll = 0.0;
		J[0][0] = 1.0;
		J[0][1] = 0.0;
		J[1][0] = 0.0;
		J[1][1] = 1.0;
		return -1;
	}
	int count = 0;

	while ((delta[0]*delta[0]+delta[1]*delta[1]) > ITER_THRESHOLD && count < MAX_ITER)
	{
		*pitch += delta[0];
		*roll += delta[1];

		R(*pitch, *roll, Rot, Rpitch, Rroll);

		y[0] = p1_dot_p3 - pTRp(p3, Rot, p1);
		y[1] = p2_dot_p4 - pTRp(p4, Rot, p2);

		Jac[0][0] = pTRp(p3, Rpitch, p1);
		Jac[0][1] = pTRp(p3, Rroll, p1);
		Jac[1][0] = pTRp(p4, Rpitch, p2);
		Jac[1][1] = pTRp(p4, Rroll, p2);

		if (!inv2(Jac))
		{
			delta[0] = Jac[0][0] * y[0] + Jac[0][1] * y[1];
			delta[1] = Jac[1][0] * y[0] + Jac[1][1] * y[1];
		}
		else
		{
			//singular
			*dpitch = 0.0;
			*droll = 0.0;
			J[0][0] = 1.0;
			J[0][1] = 0.0;
			J[1][0] = 0.0;
			J[1][1] = 1.0;
			return -1;
		}
		count++;
	}

	const double p3_dot[3] = {L * ca, 0.0, L * sa};
	const double p4_dot[3] = {-L * cb, 0.0, -L * sb};

	double M[2][2];
	M[0][0] = pTRp(p3_dot, Rot, p1) - dot3(p3, p3_dot);
	M[1][1] = pTRp(p4_dot, Rot, p2) - dot3(p4, p4_dot);
	M[0][1] = M[1][0] = 0.0;
	J[0][0] = -(Jac[0][0] * M[0][0] + Jac[0][1] * M[1][0]);
	J[0][1] = -(Jac[0][0] * M[0][1] + Jac[0][1] * M[1][1]);
	J[1][0] = -(Jac[1][0] * M[0][0] + Jac[1][1] * M[1][0]);
	J[1][1] = -(Jac[1][0] * M[0][1] + Jac[1][1] * M[1][1]);

	*dpitch = J[0][0] * da + J[0][1] * db;
	*droll = J[1][0] * da + J[1][1] * db;

	return count;
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






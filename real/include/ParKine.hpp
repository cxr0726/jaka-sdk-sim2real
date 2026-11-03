#ifndef __JK_PARKINE_H__
#define __JK_PARKINE_H__

#ifdef __cplusplus
extern "C" {
#endif

    //forward/inverse kinematics
    int  AB2PitchRoll(double a, double b, double da, double db, double* pitch, double* roll, double* dpitch, double* droll, double J[2][2]);
    void PitchRoll2AB(double pitch,double roll, double* a,double* b);

    //quatnion to euler angle, quat in [w,x,y,z] form
    void Q2RPY(const double* quat,double* rpy);

    //second order easy planner
    typedef void(*__2i0o__)(void*  obj,double x,double v);
    typedef struct{
    	double Ts;
    	double a_max;
    	double v_max;
    
    	double x0;
    	double v0;
    	__2i0o__ step;
    }NLF2;
    void NLF2Init(NLF2* nlf2,double Ts, double a_max,double v_max,double x0,double v0);

#ifdef __cplusplus
}
#endif

#endif
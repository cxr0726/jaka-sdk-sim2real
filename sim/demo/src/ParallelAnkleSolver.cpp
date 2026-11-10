#include "ParallelAnkleSolver.hpp"

#include <algorithm>
#include <cmath>

namespace jaka
{
namespace
{
// Original JAKA parallel-ankle geometry, in millimetres.
constexpr double L=27., x1=7.5, y1=11.5, z1=-40.55, z2=-21.1;
constexpr double x3=-200., x4=-117., L1=209., L2=127.;

void rotation(double p,double r,double R[3][3],double Rp[3][3],double Rr[3][3]) noexcept
{
    const double sp=std::sin(p),cp=std::cos(p),sr=std::sin(r),cr=std::cos(r);
    R[0][0]=cp*cr;R[0][1]=-cp*sr;R[0][2]=-sp;
    R[1][0]=cr*sp;R[1][1]=-sp*sr;R[1][2]=cp;
    R[2][0]=-sr;R[2][1]=-cr;R[2][2]=0.;
    Rp[0][0]=-R[1][0];Rp[0][1]=-R[1][1];Rp[0][2]=-cp;
    Rp[1][0]=R[0][0];Rp[1][1]=R[0][1];Rp[1][2]=-sp;
    Rp[2][0]=Rp[2][1]=Rp[2][2]=0.;
    Rr[0][0]=R[0][1];Rr[0][1]=-R[0][0];Rr[0][2]=0.;
    Rr[1][0]=R[1][1];Rr[1][1]=-R[1][0];Rr[1][2]=0.;
    Rr[2][0]=-cr;Rr[2][1]=sr;Rr[2][2]=0.;
}
double dot(const double a[3],const double b[3]) noexcept
{return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];}
double product(const double a[3],const double R[3][3],const double b[3]) noexcept
{
    return (a[0]*R[0][0]+a[1]*R[1][0]+a[2]*R[2][0])*b[0]
         +(a[0]*R[0][1]+a[1]*R[1][1]+a[2]*R[2][1])*b[1]
         +(a[0]*R[0][2]+a[1]*R[1][2]+a[2]*R[2][2])*b[2];
}
struct Geometry
{
    double p1[3]{x1,y1,z1},p2[3]{x1,-y1,z1};
    double p3[3],p4[3],c1,c2;
    Geometry(double a,double b) noexcept
    {
        p3[0]=x3+L*std::sin(a);p3[1]=z2;p3[2]=-L*std::cos(a);
        p4[0]=x4-L*std::sin(b);p4[1]=z2;p4[2]=L*std::cos(b);
        c1=(dot(p1,p1)+dot(p3,p3)-L1*L1)/2.;
        c2=(dot(p2,p2)+dot(p4,p4)-L2*L2)/2.;
    }
};
struct State { double p{},r{},f[2]{},A[2][2]{},loss{}; };
State evaluate(double p,double r,const Geometry &g) noexcept
{
    double R[3][3],Rp[3][3],Rr[3][3];rotation(p,r,R,Rp,Rr);
    State s;s.p=p;s.r=r;
    s.f[0]=g.c1-product(g.p3,R,g.p1);s.f[1]=g.c2-product(g.p4,R,g.p2);
    s.A[0][0]=product(g.p3,Rp,g.p1);s.A[0][1]=product(g.p3,Rr,g.p1);
    s.A[1][0]=product(g.p4,Rp,g.p2);s.A[1][1]=product(g.p4,Rr,g.p2);
    s.loss=s.f[0]*s.f[0]+s.f[1]*s.f[1];return s;
}
bool invert(const double A[2][2],double inverse[2][2],double &condition,double max_condition) noexcept
{
    const double scale=std::max({std::abs(A[0][0]),std::abs(A[0][1]),std::abs(A[1][0]),std::abs(A[1][1])});
    if(!std::isfinite(scale)||scale==0.)return false;
    const double a=A[0][0]/scale,b=A[0][1]/scale,c=A[1][0]/scale,d=A[1][1]/scale;
    const double det=a*d-b*c,trace=a*a+b*b+c*c+d*d;
    const double largest=(trace+std::sqrt(std::max(0.,trace*trace-4.*det*det)))/2.;
    if(!std::isfinite(det)||det==0.||!std::isfinite(largest))return false;
    // sigma_max / sigma_min = sigma_max^2 / |det(A)|, after scaling.
    condition=largest/std::abs(det);
    if(!std::isfinite(condition)||condition>max_condition)return false;
    const double denominator=scale*det;
    inverse[0][0]=d/denominator;inverse[0][1]=-b/denominator;
    inverse[1][0]=-c/denominator;inverse[1][1]=a/denominator;return true;
}
bool in_branch(double p,double r,const AnkleSolveOptions &o) noexcept
{return p>=o.pitch_min&&p<=o.pitch_max&&r>=o.roll_min&&r<=o.roll_max;}
bool valid_options(const AnkleSolveOptions &o) noexcept
{
    return std::isfinite(o.pitch_min)&&std::isfinite(o.pitch_max)&&o.pitch_min<o.pitch_max
        && std::isfinite(o.roll_min)&&std::isfinite(o.roll_max)&&o.roll_min<o.roll_max
        && std::isfinite(o.step_limit_rad)&&o.step_limit_rad>0.
        && std::isfinite(o.residual_norm_limit)&&o.residual_norm_limit>0.
        && std::isfinite(o.max_condition)&&o.max_condition>=1.&&o.max_iterations>=0&&o.max_iterations<=1000;
}
} // namespace

const char *ankle_status_name(AnkleSolveStatus status) noexcept
{
    switch(status)
    {
    case AnkleSolveStatus::Ok:return "ok";
    case AnkleSolveStatus::InvalidInput:return "invalid input";
    case AnkleSolveStatus::NoConvergence:return "no convergence";
    case AnkleSolveStatus::Singular:return "singular or ill-conditioned";
    case AnkleSolveStatus::OutsideAssembly:return "outside assembly branch";
    }
    return "unknown";
}

AnkleSolveResult solve_parallel_ankle(double a,double b,double da,double db,
    const AnkleSolveOptions &o,const AnkleSolveResult *previous) noexcept
{
    AnkleSolveResult result;
    if(!std::isfinite(a)||!std::isfinite(b)||!std::isfinite(da)||!std::isfinite(db)||!valid_options(o))
    {result.status=AnkleSolveStatus::InvalidInput;return result;}
    const Geometry geometry(a,b);
    const double p0=std::clamp((a-b)/2.,o.pitch_min,o.pitch_max);
    const double r0=std::clamp(-(a+b)/2.,o.roll_min,o.roll_max);
    // The previous valid state is preferred but never returned as a fallback.
    double seeds[10][2]={{p0,r0},{0.,0.},{-.5,0.},{.5,0.},{0.,-.5},{0.,.5},{-.5,-.5},{.5,.5},{.5,-.5},{0.,0.}};
    int count=9;
    if(previous&&previous->ok()&&std::isfinite(previous->pitch)&&std::isfinite(previous->roll)
        &&in_branch(previous->pitch,previous->roll,o))
    {
        for(int i=9;i>0;--i){seeds[i][0]=seeds[i-1][0];seeds[i][1]=seeds[i-1][1];}
        seeds[0][0]=previous->pitch;seeds[0][1]=previous->roll;count=10;
    }
    State best;best.loss=std::numeric_limits<double>::infinity();int best_iterations=0;
    const double limit_squared=o.residual_norm_limit*o.residual_norm_limit;
    const double target=std::min(1.e-12,limit_squared);
    bool saw_singular=false;
    for(int seed=0;seed<count;++seed)
    {
        const double p=std::clamp(seeds[seed][0],o.pitch_min,o.pitch_max);
        const double r=std::clamp(seeds[seed][1],o.roll_min,o.roll_max);
        State s=evaluate(p,r,geometry);int iteration=0;
        for(;iteration<o.max_iterations&&s.loss>target;++iteration)
        {
            double inverse[2][2],condition=0.;
            if(!invert(s.A,inverse,condition,o.max_condition)){saw_singular=true;break;}
            double dp=inverse[0][0]*s.f[0]+inverse[0][1]*s.f[1];
            double dr=inverse[1][0]*s.f[0]+inverse[1][1]*s.f[1];
            const double largest=std::max(std::abs(dp),std::abs(dr));
            if(!std::isfinite(largest))break;
            if(largest>o.step_limit_rad){dp*=o.step_limit_rad/largest;dr*=o.step_limit_rad/largest;}
            bool accepted=false;
            for(double alpha=1.;alpha>=1./4096.;alpha*=.5)
            {
                const double p_next=s.p+alpha*dp,r_next=s.r+alpha*dr;
                if(!in_branch(p_next,r_next,o))continue;
                const State candidate=evaluate(p_next,r_next,geometry);
                if(std::isfinite(candidate.loss)&&candidate.loss<s.loss)
                {s=candidate;accepted=true;break;}
            }
            if(!accepted)break;
        }
        if(s.loss<best.loss){best=s;best_iterations=iteration;}
        if(s.loss<=target)break;
    }
    result.residual_norm=std::sqrt(best.loss);result.iterations=best_iterations;
    if(!std::isfinite(best.loss)||best.loss>limit_squared)
    {
        const double edge=1.e-9;
        const bool at_edge=std::abs(best.p-o.pitch_min)<edge||std::abs(best.p-o.pitch_max)<edge
            ||std::abs(best.r-o.roll_min)<edge||std::abs(best.r-o.roll_max)<edge;
        result.status=at_edge?AnkleSolveStatus::OutsideAssembly:
            (saw_singular?AnkleSolveStatus::Singular:AnkleSolveStatus::NoConvergence);
        return result;
    }
    if(!in_branch(best.p,best.r,o)){result.status=AnkleSolveStatus::OutsideAssembly;return result;}
    double inverse[2][2];
    if(!invert(best.A,inverse,result.condition,o.max_condition))
    {result.status=AnkleSolveStatus::Singular;return result;}
    double R[3][3],Rp[3][3],Rr[3][3];rotation(best.p,best.r,R,Rp,Rr);
    const double p3dot[3]={L*std::cos(a),0.,L*std::sin(a)};
    const double p4dot[3]={-L*std::cos(b),0.,-L*std::sin(b)};
    const double m0=product(p3dot,R,geometry.p1)-dot(geometry.p3,p3dot);
    const double m1=product(p4dot,R,geometry.p2)-dot(geometry.p4,p4dot);
    const double J[2][2]={{-inverse[0][0]*m0,-inverse[0][1]*m1},
                          {-inverse[1][0]*m0,-inverse[1][1]*m1}};
    const double dp=J[0][0]*da+J[0][1]*db,dr=J[1][0]*da+J[1][1]*db;
    if(!std::isfinite(dp)||!std::isfinite(dr)||!std::isfinite(J[0][0])||!std::isfinite(J[0][1])
        ||!std::isfinite(J[1][0])||!std::isfinite(J[1][1]))
    {result.status=AnkleSolveStatus::Singular;return result;}
    result.pitch=best.p;result.roll=best.r;result.dpitch=dp;result.droll=dr;
    for(int i=0;i<2;++i)for(int j=0;j<2;++j)result.J[i][j]=J[i][j];
    result.status=AnkleSolveStatus::Ok;return result;
}

AnkleSolveResult ParallelAnkleSolver::solve(double a,double b,double da,double db) noexcept
{
    AnkleSolveResult result=solve_parallel_ankle(a,b,da,db,options_,have_last_?&last_:nullptr);
    if(result.ok()){last_=result;have_last_=true;}
    return result;
}
} // namespace jaka

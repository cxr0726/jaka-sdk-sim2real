#include "ParKine.hpp"
#include "ParallelAnkleSolver.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {
void check(bool condition,const std::string &message)
{if(!condition)throw std::runtime_error(message);}
void close(double a,double b,double tolerance,const std::string &message)
{check(std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tolerance,message);}
struct Result {int code;double p,r,dp,dr,J[2][2];};
Result forward(double a,double b,double da=0.,double db=0.) {
    Result x{};x.code=AB2PitchRoll(a,b,da,db,&x.p,&x.r,&x.dp,&x.dr,x.J);return x;
}
void nominal_geometry_and_jacobian() {
    // Avoid combinations projected by the unchanged legacy inverse.
    for(int pi=-4;pi<=5;++pi)for(int ri=-7;ri<=7;++ri) {
        const double p=.1*pi,r=.1*ri;double a,b;PitchRoll2AB(p,r,&a,&b);
        const auto x=forward(a,b,1.7,-.9);check(x.code>=0,"nominal forward failed");
        close(x.p,p,1.e-8,"pitch roundtrip");close(x.r,r,1.e-8,"roll roundtrip");
    }
    double a,b;PitchRoll2AB(.1,-.05,&a,&b);const auto x=forward(a,b,1.7,-.9);
    close(x.dp,x.J[0][0]*1.7-x.J[0][1]*.9,1.e-12,"pitch velocity convention");
    close(x.dr,x.J[1][0]*1.7-x.J[1][1]*.9,1.e-12,"roll velocity convention");
    constexpr double epsilon=1.e-5;
    for(int column=0;column<2;++column) {
        const auto plus=forward(a+(column==0?epsilon:0.),b+(column==1?epsilon:0.));
        const auto minus=forward(a-(column==0?epsilon:0.),b-(column==1?epsilon:0.));
        check(plus.code>=0&&minus.code>=0,"Jacobian neighbor failed");
        close((plus.p-minus.p)/(2.*epsilon),x.J[0][column],1.e-6,"pitch finite-difference Jacobian");
        close((plus.r-minus.r)/(2.*epsilon),x.J[1][column],1.e-6,"roll finite-difference Jacobian");
    }
}
void recorded_dance_regressions() {
    // First SDK 1kHz failure: old count=10, PR=(-25.7345,-87.7443).
    const auto first=forward(-1.6221196396336162,1.0576655247598854,-7.921380633610002,-2.40221612481025);
    check(first.code>=0,"recorded first SDK input rejected");
    close(first.p,-.6016659070179456,1.e-8,"first SDK pitch branch");
    close(first.r,.2206868403526281,1.e-8,"first SDK roll branch");
    // Twelve ms later the old solver returned count=6, PR=(-6.5407,-34.6567).
    // Native closed-chain PR is independent evidence for the physical branch;
    // finite equality compliance means it need not exactly match rod equations.
    const auto policy=forward(-1.665803491124356,.9763095563801714,-.8263821406032572,-5.19277709118868);
    check(policy.code>=0,"recorded policy input rejected");
    close(policy.p,-.5878794492566483,.03,"policy pitch assembly branch");
    close(policy.r,.2698384684351259,.03,"policy roll assembly branch");
}
void failures_are_explicit() {
    const double nan=std::numeric_limits<double>::quiet_NaN();
    for(const auto &x:{forward(nan,0.),forward(0.,0.,nan,0.),forward(1.5,1.5)}) {
        check(x.code<0,"invalid/unreachable input accepted");
        check(!std::isfinite(x.p)&&!std::isfinite(x.r)&&!std::isfinite(x.dp)&&!std::isfinite(x.dr),"failure fabricated usable state");
        check(!std::isfinite(x.J[0][0])&&!std::isfinite(x.J[1][1]),"failure fabricated usable Jacobian");
    }
    jaka::AnkleSolveOptions options;options.max_iterations=0;
    const auto exhausted=jaka::solve_parallel_ankle(.2,-.3,0.,0.,options);
    check(!exhausted.ok(),"exhausted iterations reported success");
    double p,r,dp,dr,J[2][2];
    check(AB2PitchRoll(0.,0.,0.,0.,nullptr,&r,&dp,&dr,J)<0,"null output accepted");
    check(AB2PitchRoll(0.,0.,0.,0.,&p,&r,&dp,&dr,nullptr)<0,"null Jacobian accepted");
}
void benchmark() {
    const auto start=std::chrono::steady_clock::now();double sum=0.;constexpr int samples=10000;
    for(int i=0;i<samples;++i) {
        double a,b;PitchRoll2AB(.4*std::sin(i*.003),.6*std::cos(i*.002),&a,&b);
        const auto x=forward(a,b);check(x.code>=0,"benchmark workspace rejected");sum+=x.p+x.r;
    }
    const auto elapsed=std::chrono::duration<double,std::micro>(std::chrono::steady_clock::now()-start).count();
    std::cout<<"forward mean "<<elapsed/samples<<" us over "<<samples<<" calls; checksum "<<sum<<'\n';
}
}
int main() {try {
    nominal_geometry_and_jacobian();recorded_dance_regressions();failures_are_explicit();benchmark();
    std::cout<<"bounded ankle: recorded wrong-branch regressions, geometry/Jacobian and failure contract passed\n";return 0;
} catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}}

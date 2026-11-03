#include "ParallelAnkleSolver.hpp"
#include "ParKine.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace
{
void check(bool condition,const std::string &message)
{if(!condition)throw std::runtime_error(message);}
void close(double a,double b,double tolerance,const std::string &message)
{check(std::isfinite(a)&&std::isfinite(b)&&std::abs(a-b)<=tolerance,message);}

void test_workspace_and_jacobian()
{
    jaka::ParallelAnkleSolver warm;
    int reachable=0;
    for(int pi=0;pi<=50;++pi)for(int ri=0;ri<=50;++ri)
    {
        const double p=-.5236+pi*(.6108+.5236)/50.;
        const double r=-.349+ri*.698/50.;
        double a,b;
        check(PitchRoll2ABChecked(p,r,&a,&b)>=0,"reachable physical PR workspace rejected by IK");
        const auto cold=jaka::solve_parallel_ankle(a,b,1.7,-.9);
        const auto cached=warm.solve(a,b,1.7,-.9);
        check(cold.ok()&&cached.ok(),"forward solve failed inside physical PR workspace");
        close(cold.pitch,p,1.e-8,"pitch roundtrip");close(cold.roll,r,1.e-8,"roll roundtrip");
        close(cached.pitch,cold.pitch,1.e-8,"warm/cold pitch branch differs");
        close(cached.roll,cold.roll,1.e-8,"warm/cold roll branch differs");
        check(cold.residual_norm<=1.e-5,"accepted closure residual above limit");
        close(cold.dpitch,cold.J[0][0]*1.7-cold.J[0][1]*.9,1.e-12,"pitch velocity map");
        close(cold.droll,cold.J[1][0]*1.7-cold.J[1][1]*.9,1.e-12,"roll velocity map");
        if(pi%5==0&&ri%5==0)
        {
            constexpr double eps=1.e-6;
            for(int column=0;column<2;++column)
            {
                const auto plus=jaka::solve_parallel_ankle(a+(column==0?eps:0.),b+(column==1?eps:0.),0.,0.);
                const auto minus=jaka::solve_parallel_ankle(a-(column==0?eps:0.),b-(column==1?eps:0.),0.,0.);
                check(plus.ok()&&minus.ok(),"finite-difference neighbor solve failed");
                close((plus.pitch-minus.pitch)/(2.*eps),cold.J[0][column],2.e-6,"pitch analytic Jacobian");
                close((plus.roll-minus.roll)/(2.*eps),cold.J[1][column],2.e-6,"roll analytic Jacobian");
            }
        }
        ++reachable;
    }
    check(reachable==2601,"workspace coverage changed");
}

void test_recorded_wrong_roots()
{
    // Recorded full-precision inputs that made the old solver return positive
    // iteration counts while producing wrong roots (including -1475 rad).
    const double examples[][8]={
        {-.7943700221369686,1.6945090391329518,-8.52789290904364,1.3718991549630228,
         -.5464455886072532,-.3828205069150595,-2.3105571453802134,7.347152748946591},
        {-1.0806399969147658,1.6468979123351752,-4.116228762912707,6.8168391057000015,
         -.6091147123508757,-.16666810940757307,-.7920394304202802,2.1949981347969434},
        {-1.4083161533228334,1.533408447835432,-14.034463394402362,-8.414186094663386,
         -.6450210949330455,-.0021859897329066734,-.6011347123687059,5.404183349127628},
        {-1.6637234588510597,1.2935632648146638,-39.23601235996652,-10.15450268441606,
         -.6347467794561583,.10022292063805173,1.3835562797303913,2.8824034900800797},
        {1.5676964868556393,-1.2189129459192467,-4.094664780630148,-7.989245359391221,
         .8546468308148155,-.024805810238618616,1.761282365420165,4.422124116070136}};
    jaka::ParallelAnkleSolver warm;
    for(const auto &e:examples)
    {
        const auto x=jaka::solve_parallel_ankle(e[0],e[1],e[2],e[3]);
        const auto y=warm.solve(e[0],e[1],e[2],e[3]);
        check(x.ok()&&y.ok(),"recorded wrong-root input no longer solved");
        close(x.pitch,e[4],1.e-8,"recorded pitch root");close(x.roll,e[5],1.e-8,"recorded roll root");
        close(x.dpitch,e[6],1.e-6,"recorded pitch velocity");close(x.droll,e[7],1.e-6,"recorded roll velocity");
        close(y.pitch,x.pitch,1.e-8,"recorded cached pitch");close(y.roll,x.roll,1.e-8,"recorded cached roll");
    }
}

void test_failure_is_explicit_and_cache_is_transactional()
{
    double a,b;check(PitchRoll2ABChecked(.2,-.1,&a,&b)>=0,"fixture IK");
    jaka::ParallelAnkleSolver left,right;
    const auto good=left.solve(a,b,.2,.3);check(good.ok(),"fixture FK");
    check(!right.has_last_valid(),"leg contexts share hidden state");
    const double nan=std::numeric_limits<double>::quiet_NaN();
    const double inf=std::numeric_limits<double>::infinity();
    for(int k=0;k<4;++k)
    {
        double input[4]={a,b,.2,.3};input[k]=k%2?inf:nan;
        const auto bad=left.solve(input[0],input[1],input[2],input[3]);
        check(bad.status==jaka::AnkleSolveStatus::InvalidInput,"nonfinite input status");
        check(!bad.ok()&&std::isnan(bad.pitch)&&std::isnan(bad.roll),"failed result appears valid");
        check(left.has_last_valid(),"failure removed previous valid seed");
        close(left.last_valid().pitch,good.pitch,0.,"failure corrupted previous valid pitch");
        close(left.last_valid().roll,good.roll,0.,"failure corrupted previous valid roll");
    }
    check(left.solve(a,b,0.,0.).ok(),"solver did not recover after invalid input");
    left.reset();check(!left.has_last_valid(),"reset retains seed");
    jaka::AnkleSolveOptions no_iterations;no_iterations.max_iterations=0;
    check(!jaka::solve_parallel_ankle(a,b,0.,0.,no_iterations).ok(),"no-convergence accepted as success");
    jaka::AnkleSolveOptions strict_branch;strict_branch.pitch_min=.7;strict_branch.pitch_max=.9;
    strict_branch.roll_min=.6;strict_branch.roll_max=.8;
    check(!jaka::solve_parallel_ankle(a,b,0.,0.,strict_branch).ok(),"wrong assembly accepted");
    jaka::AnkleSolveOptions ill_conditioned;ill_conditioned.max_condition=1.;
    check(!jaka::solve_parallel_ankle(a,b,0.,0.,ill_conditioned).ok(),"condition limit ignored");
    double p=4.,r=4.,dp=4.,dr=4.,J[2][2];
    check(AB2PitchRoll(nan,b,0.,0.,&p,&r,&dp,&dr,J)<0,"C ABI failure code");
    check(std::isnan(p)&&std::isnan(r)&&std::isnan(dp)&&std::isnan(dr)&&std::isnan(J[0][0]),
          "C ABI failure silently fabricated zero/identity feedback");
    check(AB2PitchRoll(a,b,0.,0.,nullptr,&r,&dp,&dr,J)<0,"C ABI null pointer");
    check(PitchRoll2ABChecked(nan,0.,&a,&b)<0&&std::isnan(a)&&std::isnan(b),"nonfinite IK accepted");
    check(PitchRoll2ABChecked(2.,0.,&a,&b)<0&&std::isnan(a)&&std::isnan(b),"outside assembly IK projected");
    check(PitchRoll2ABChecked(1.35,1.15,&a,&b)<0,"unreachable IK silently clamped");
    check(PitchRoll2ABChecked(0.,0.,nullptr,&b)<0,"IK null pointer");
}
}

int main()
{
    try
    {
        test_workspace_and_jacobian();test_recorded_wrong_roots();
        test_failure_is_explicit_and_cache_is_transactional();
        std::cout<<"parallel ankle solver: workspace, recorded wrong roots, Jacobian and failures passed\n";
        return 0;
    }
    catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}

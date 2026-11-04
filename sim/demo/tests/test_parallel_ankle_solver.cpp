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
struct Result {int code;double p,r,dp,dr,J[2][2];};
Result forward(double a,double b,double da=0.,double db=0.)
{
    Result x{};
    x.code=AB2PitchRoll(a,b,da,db,&x.p,&x.r,&x.dp,&x.dr,x.J);
    return x;
}

void test_nominal_geometry_and_jacobian()
{
    // Exercise the original C ABI in the nominal workspace. The old solver's
    // step-based tolerance is intentionally less strict than the removed one.
    for(int pi=-2;pi<=2;++pi)for(int ri=-2;ri<=2;++ri)
    {
        const double p=.1*pi,r=.05*ri;
        double a,b;PitchRoll2AB(p,r,&a,&b);
        const auto x=forward(a,b,1.7,-.9);
        check(x.code>=0,"nominal legacy forward solve failed");
        close(x.p,p,2.e-5,"nominal pitch roundtrip");
        close(x.r,r,2.e-5,"nominal roll roundtrip");
    }
    double a,b;PitchRoll2AB(.1,-.05,&a,&b);
    const auto x=forward(a,b,1.7,-.9);
    close(x.dp,x.J[0][0]*1.7-x.J[0][1]*.9,1.e-12,"pitch velocity convention");
    close(x.dr,x.J[1][0]*1.7-x.J[1][1]*.9,1.e-12,"roll velocity convention");
    constexpr double epsilon=1.e-4;
    for(int column=0;column<2;++column)
    {
        const auto plus=forward(a+(column==0?epsilon:0.),b+(column==1?epsilon:0.));
        const auto minus=forward(a-(column==0?epsilon:0.),b-(column==1?epsilon:0.));
        check(plus.code>=0&&minus.code>=0,"legacy Jacobian neighbor solve failed");
        close((plus.p-minus.p)/(2.*epsilon),x.J[0][column],2.e-5,"legacy pitch Jacobian");
        close((plus.r-minus.r)/(2.*epsilon),x.J[1][column],2.e-5,"legacy roll Jacobian");
    }
}

void test_original_abi_return_and_projection_contract()
{
    // Compatibility checks record the requested rollback semantics, rather
    // than claiming the original solver has bounded roots or residual checks.
    const auto projected=forward(1.5,1.5,.2,-.3);
    const auto original_sum_limit=forward(1.5,-.5,.2,-.3);
    check(projected.code>=0&&original_sum_limit.code>=0,"legacy sum-limit fixture failed");
    close(projected.p,original_sum_limit.p,0.,"legacy AB sum projection changed");
    close(projected.r,original_sum_limit.r,0.,"legacy AB sum projection changed");
    close(projected.dp,original_sum_limit.dp,0.,"legacy projected velocity changed");
    close(projected.dr,original_sum_limit.dr,0.,"legacy projected velocity changed");

    const auto failed=forward(std::numeric_limits<double>::quiet_NaN(),0.);
    check(failed.code<0,"original negative-return fixture changed");
    close(failed.dp,0.,0.,"legacy failure pitch velocity");
    close(failed.dr,0.,0.,"legacy failure roll velocity");
    close(failed.J[0][0],1.,0.,"legacy failure Jacobian");
    close(failed.J[1][1],1.,0.,"legacy failure Jacobian");
}
}

int main()
{
    try
    {
        test_nominal_geometry_and_jacobian();
        test_original_abi_return_and_projection_contract();
        std::cout<<"original parallel ankle ABI: nominal geometry/Jacobian and legacy return/projection semantics passed\n";
        return 0;
    }
    catch(const std::exception &error){std::cerr<<error.what()<<'\n';return 1;}
}

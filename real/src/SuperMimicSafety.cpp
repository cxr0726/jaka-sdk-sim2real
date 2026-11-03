#include "SuperMimicSafety.hpp"
#include "ParallelAnkleSolver.hpp"
#include "RLController.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

namespace RLController
{
    namespace
    {
        constexpr int kSuperJoints = 27;
        constexpr std::array<int, 2> kAnkleIndex = {4, 16};
        // One cache per leg and per controller consumer; never shared with
        // legacy modes or hidden inside the old stateless C entry points.
        std::array<jaka::ParallelAnkleSolver, 2> ankle_observers;

        constexpr double L=27., x1=7.5, kRodY1=11.5, z1=-40.55, z2=-21.1;
        constexpr double x3=-200., x4=-117., L1=209., L2=127.;
        constexpr double PI=3.14159265358979323846;

        // File-local copy of the simulation's checked inverse. In particular,
        // the second rod's -2*kRodY1*z2*sp*sr sign is intentional. This defines
        // no PitchRoll2AB/AB2PitchRoll symbol and cannot replace legacy ABI.
        int checked_pitch_roll_to_ab(double pitch,double roll,double *a,double *b)
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
            const auto check=jaka::solve_parallel_ankle(qa,qb,0.,0.);
            if(!check.ok())return fail(check.status);
            if(std::abs(check.pitch-pitch)>1.e-7||std::abs(check.roll-roll)>1.e-7)
                return fail(jaka::AnkleSolveStatus::OutsideAssembly);
            *a=qa;*b=qb;return 0;
        }

        bool fault(const char *where, const char *reason)
        {
            latch_super_fault(where, reason);
            return false;
        }

        bool valid_joint_array(const double *q)
        {
            return q && mNumAction == kSuperJoints &&
                std::all_of(q, q+kSuperJoints, [](double value) { return std::isfinite(value); });
        }

        bool finite_result(const jaka::AnkleSolveResult &result)
        {
            return result.ok() && std::isfinite(result.pitch) && std::isfinite(result.roll) &&
                std::isfinite(result.dpitch) && std::isfinite(result.droll);
        }
    } // namespace

    void latch_super_fault(const char *where, const char *reason)
    {
        std::fprintf(stderr, "[SuperMimic fault] %s: %s; disabling PD and latching ERR_JOINT\n",
                     where ? where : "unknown", reason ? reason : "invalid input");
        mErrCode = ERR_JOINT;
        if (mRobot)
        {
            mRobot->enable_pd(false);
            mRobot->do_enable(false);
        }
    }

    void reset_super_parallel_ankles()
    {
        for (auto &observer : ankle_observers) observer.reset();
    }

    bool decode_super_parallel_ankles(double *q, double *qd, const char *where)
    {
        if (!valid_joint_array(q) || !valid_joint_array(qd))
            return fault(where, "invalid/nonfinite 27-joint feedback");
        // Check both legs before committing either measurements or observer
        // caches. A bad right leg must not leave a converted left leg behind.
        auto next_observers = ankle_observers;
        std::array<jaka::AnkleSolveResult, 2> result;
        for (int side=0; side<2; ++side)
        {
            const int i=kAnkleIndex[side];
            const double sign=side==0 ? 1. : -1.;
            const auto canonical=jaka::solve_parallel_ankle(sign*q[i], sign*q[i+1], sign*qd[i], sign*qd[i+1]);
            const auto observed=next_observers[side].solve(sign*q[i], sign*q[i+1], sign*qd[i], sign*qd[i+1]);
            if (!finite_result(canonical) || !finite_result(observed))
            {
                char reason[160];
                const auto status=!canonical.ok() ? canonical.status : observed.status;
                std::snprintf(reason, sizeof(reason), "%s ankle forward solve: %s",
                              side==0 ? "left" : "right", jaka::ankle_status_name(status));
                return fault(where, reason);
            }
            if (std::max(std::abs(observed.pitch-canonical.pitch),
                         std::abs(observed.roll-canonical.roll)) > 1.e-7)
                return fault(where, "ambiguous ankle assembly root");
            result[side]=canonical;
        }
        for (int side=0; side<2; ++side)
        {
            const int i=kAnkleIndex[side];
            q[i]=result[side].pitch; q[i+1]=result[side].roll;
            qd[i]=result[side].dpitch; qd[i+1]=result[side].droll;
        }
        ankle_observers=next_observers;
        return true;
    }

    bool encode_super_parallel_ankle_targets(double *q, const char *where)
    {
        if (!valid_joint_array(q))
            return fault(where, "invalid/nonfinite 27-joint target");
        double a[2],b[2];
        for (int side=0; side<2; ++side)
        {
            const int i=kAnkleIndex[side];
            const int status=checked_pitch_roll_to_ab(q[i],q[i+1],&a[side],&b[side]);
            if (status<0 || !std::isfinite(a[side]) || !std::isfinite(b[side]))
            {
                char reason[160];
                std::snprintf(reason, sizeof(reason), "%s ankle target: %s", side==0 ? "left" : "right",
                              jaka::ankle_status_name(static_cast<jaka::AnkleSolveStatus>(status)));
                return fault(where, reason);
            }
        }
        q[4]=a[0];q[5]=b[0];q[16]=-a[1];q[17]=-b[1];
        return true;
    }
} // namespace RLController

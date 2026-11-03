#include "RLController.hpp" // Test-only stub, selected by this target's include path.
#include "SuperMimicSafety.hpp"
#include "ParallelAnkleSolver.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace RLController
{
    std::unique_ptr<StubRobot> mRobot;
    int mNumAction = 27;
    ERRCODE mErrCode = ERR_NONE;
}

namespace
{
    using JointArray = std::array<double, 27>;

    void require(bool ok, const char *why)
    {
        if (!ok) throw std::runtime_error(why);
    }

    bool identical(const JointArray &a, const JointArray &b)
    {
        // Bitwise comparison also preserves the payload of a NaN fixture.
        return std::memcmp(a.data(), b.data(), sizeof(a)) == 0;
    }

    void require_shutdown(int expected)
    {
        using namespace RLController;
        require(mErrCode == ERR_JOINT, "fault did not latch ERR_JOINT");
        require(mRobot && mRobot->pd_off == expected && mRobot->enable_off == expected,
                "fault did not disable PD and robot enable");
    }

    JointArray raw_fixture()
    {
        JointArray q{};
        q[4] = .1; q[5] = -.05;
        q[16] = -.2; q[17] = .1;
        require(RLController::encode_super_parallel_ankle_targets(q.data(), "test fixture"),
                "fixture inverse rejected");
        return q;
    }

    double test_workspace_and_signs()
    {
        using namespace RLController;
        reset_super_parallel_ankles();
        double max_error = 0.;
        int poses = 0;
        for (int pi = 0; pi <= 50; ++pi)
        {
            for (int ri = 0; ri <= 50; ++ri)
            {
                const double pitch = -.5236 + pi * (.6108 + .5236) / 50.;
                const double roll = -.349 + ri * .698 / 50.;
                JointArray q{}, velocity{};
                q[4] = pitch; q[5] = roll;
                q[16] = -.4 * pitch; q[17] = -.7 * roll;
                q[8] = .123;
                const auto expected = q;
                require(encode_super_parallel_ankle_targets(q.data(), "workspace inverse"),
                        "physical workspace inverse rejected");
                velocity[4] = .2; velocity[5] = -.3;
                velocity[16] = .4; velocity[17] = -.5;
                const auto left = jaka::solve_parallel_ankle(q[4], q[5], velocity[4], velocity[5]);
                const auto right = jaka::solve_parallel_ankle(-q[16], -q[17], -velocity[16], -velocity[17]);
                require(left.ok() && right.ok(), "encoded AB rejected by independent forward solver");
                // Independently fix the sign convention, so matching sign
                // mistakes in encode and decode cannot cancel in a roundtrip.
                require(std::abs(left.pitch - expected[4]) < 1.e-8 &&
                        std::abs(left.roll - expected[5]) < 1.e-8 &&
                        std::abs(right.pitch - expected[16]) < 1.e-8 &&
                        std::abs(right.roll - expected[17]) < 1.e-8,
                        "target AB sign convention changed");
                require(decode_super_parallel_ankles(q.data(), velocity.data(), "workspace feedback"),
                        "physical workspace feedback rejected");
                for (int joint = 0; joint < 27; ++joint)
                {
                    max_error = std::max(max_error, std::abs(q[joint] - expected[joint]));
                    require(std::abs(q[joint] - expected[joint]) < 1.e-8,
                            "roundtrip or nonankle joint order changed");
                }
                require(std::abs(velocity[4] - left.dpitch) < 1.e-12 &&
                        std::abs(velocity[5] - left.droll) < 1.e-12 &&
                        std::abs(velocity[16] - right.dpitch) < 1.e-12 &&
                        std::abs(velocity[17] - right.droll) < 1.e-12,
                        "velocity mapping/right sign changed");
                ++poses;
            }
        }
        require(poses == 2601, "physical workspace coverage changed");
        require(mErrCode == ERR_NONE && mRobot->pd_off == 0 && mRobot->enable_off == 0,
                "healthy conversion disabled the robot");
        return max_error;
    }

    void test_recorded_wrong_root()
    {
        using namespace RLController;
        auto q = raw_fixture();
        JointArray velocity{};
        q[16] = .7943700221369686;
        q[17] = -1.6945090391329518;
        require(decode_super_parallel_ankles(q.data(), velocity.data(), "recorded wrong-root"),
                "recorded wrong-root input rejected");
        require(std::abs(q[16] + .5464455886072532) < 1.e-8 &&
                std::abs(q[17] + .3828205069150595) < 1.e-8,
                "recorded right ankle returned wrong assembly root");
    }

    void test_transactional_failures()
    {
        using namespace RLController;
        auto q = raw_fixture();
        JointArray velocity{};
        velocity[17] = std::numeric_limits<double>::quiet_NaN();
        auto before = q, before_velocity = velocity;
        require(!decode_super_parallel_ankles(q.data(), velocity.data(), "bad right velocity"),
                "NaN feedback accepted");
        require(identical(q, before) && identical(velocity, before_velocity),
                "failed feedback partially changed either leg");
        require_shutdown(1);

        // Both inputs are finite: exercise the failure after the left leg has
        // solved, rather than only the up-front nonfinite-input rejection.
        mErrCode = ERR_NONE;
        q = raw_fixture(); velocity.fill(0.);
        require(!jaka::solve_parallel_ankle(1.5, 1.5, 0., 0.).ok(),
                "finite invalid-assembly fixture changed");
        q[16] = -1.5; q[17] = -1.5;
        before = q; before_velocity = velocity;
        require(!decode_super_parallel_ankles(q.data(), velocity.data(), "finite bad right assembly"),
                "finite invalid assembly accepted");
        require(identical(q, before) && identical(velocity, before_velocity),
                "right solve failure left partial left conversion");
        require_shutdown(2);

        mErrCode = ERR_NONE;
        q.fill(0.); q[16] = 2.; before = q;
        require(!encode_super_parallel_ankle_targets(q.data(), "unreachable right target"),
                "unreachable inverse accepted");
        require(identical(q, before), "right inverse failure left partial left target");
        require_shutdown(3);

        mErrCode = ERR_NONE;
        q.fill(0.); velocity.fill(0.);
        q[8] = std::numeric_limits<double>::infinity();
        before = q; before_velocity = velocity;
        require(!decode_super_parallel_ankles(q.data(), velocity.data(), "nonankle infinite feedback"),
                "nonankle Inf accepted");
        require(identical(q, before) && identical(velocity, before_velocity),
                "invalid nonankle feedback changed arrays");
        require_shutdown(4);

        // A failure cannot be consumed as a successful stale/zero result.
        // Once the external fault is cleared, valid input can be solved again.
        mErrCode = ERR_NONE;
        q = raw_fixture(); velocity.fill(0.);
        require(decode_super_parallel_ankles(q.data(), velocity.data(), "valid recovery"),
                "cache did not recover after a failed leg");
        require(std::abs(q[4] - .1) < 1.e-8 && std::abs(q[16] + .2) < 1.e-8,
                "failure corrupted independent leg caches");
        require(mErrCode == ERR_NONE && mRobot->pd_off == 4 && mRobot->enable_off == 4,
                "healthy recovery altered fault/enable state");

        mNumAction = 26;
        require(!encode_super_parallel_ankle_targets(q.data(), "bad dimensions"),
                "dimension mismatch accepted");
        require_shutdown(5);
        mNumAction = 27;
        mRobot.reset(); mErrCode = ERR_NONE;
        require(!decode_super_parallel_ankles(nullptr, velocity.data(), "null robot and feedback"),
                "null feedback accepted");
        require(mErrCode == ERR_JOINT, "null-robot fault not latched");
        reset_super_parallel_ankles();
    }
} // namespace

int main()
{
    try
    {
        RLController::mRobot = std::make_unique<RLController::StubRobot>();
        const double max_error = test_workspace_and_signs();
        test_recorded_wrong_root();
        test_transactional_failures();
        std::cout << "SuperMimic safety: 2601 poses, max roundtrip error=" << max_error
                  << "; signs/velocity, wrong root, transactional faults and disable passed; no hardware\n";
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}

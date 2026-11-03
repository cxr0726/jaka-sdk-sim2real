#pragma once
#include <limits>

namespace jaka
{
enum class AnkleSolveStatus : int
{
    Ok = 0,
    InvalidInput = -1,
    NoConvergence = -2,
    Singular = -3,
    OutsideAssembly = -4
};

struct AnkleSolveOptions
{
    // Assembly-root bounds, deliberately wider than physical joint limits.
    double pitch_min = -1.4, pitch_max = 1.4;
    double roll_min = -1.2, roll_max = 1.2;
    double step_limit_rad = .2;
    double residual_norm_limit = 1.e-5; // squared rod-length residual, mm^2
    double max_condition = 1.e6;
    int max_iterations = 60;
};

struct AnkleSolveResult
{
    AnkleSolveStatus status = AnkleSolveStatus::NoConvergence;
    double pitch = std::numeric_limits<double>::quiet_NaN();
    double roll = std::numeric_limits<double>::quiet_NaN();
    double dpitch = std::numeric_limits<double>::quiet_NaN();
    double droll = std::numeric_limits<double>::quiet_NaN();
    double J[2][2] = {{0., 0.}, {0., 0.}};
    double residual_norm = std::numeric_limits<double>::infinity();
    double condition = std::numeric_limits<double>::infinity();
    int iterations = 0;
    bool ok() const noexcept { return status == AnkleSolveStatus::Ok; }
};

const char *ankle_status_name(AnkleSolveStatus status) noexcept;
AnkleSolveResult solve_parallel_ankle(double a, double b, double da, double db,
    const AnkleSolveOptions &options = {}, const AnkleSolveResult *seed = nullptr) noexcept;

// Own one instance per physical ankle/consumer. A failed solve leaves last_valid
// unchanged; callers must still handle the failed result, never consume it as
// a current measurement or silently substitute last_valid forever.
class ParallelAnkleSolver
{
public:
    explicit ParallelAnkleSolver(AnkleSolveOptions options = {}) : options_(options) {}
    AnkleSolveResult solve(double a, double b, double da, double db) noexcept;
    void reset() noexcept { have_last_ = false; last_ = {}; }
    bool has_last_valid() const noexcept { return have_last_; }
    const AnkleSolveResult &last_valid() const noexcept { return last_; }
private:
    AnkleSolveOptions options_;
    bool have_last_ = false;
    AnkleSolveResult last_;
};
} // namespace jaka

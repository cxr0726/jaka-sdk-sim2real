#pragma once

#include <mujoco/mujoco.h>

struct GLFWwindow;

class MujocoViewer
{
public:
    MujocoViewer() = default;
    ~MujocoViewer();

    MujocoViewer(const MujocoViewer &) = delete;
    MujocoViewer &operator=(const MujocoViewer &) = delete;

    bool open(mjModel *model, mjData *data, const char *title);
    bool is_open() const;
    void render(mjModel *model, mjData *data);
    void close();

private:
    static void mouse_button_callback(GLFWwindow *window, int button, int action, int mods);
    static void cursor_pos_callback(GLFWwindow *window, double xpos, double ypos);
    static void scroll_callback(GLFWwindow *window, double xoffset, double yoffset);

    GLFWwindow *window_{nullptr};
    mjModel *model_{nullptr};
    mjvCamera cam_{};
    mjvOption opt_{};
    mjvScene scene_{};
    mjrContext context_{};
    bool context_ready_{false};
    bool mouse_left_{false};
    bool mouse_middle_{false};
    bool mouse_right_{false};
    double last_x_{0.0};
    double last_y_{0.0};
};

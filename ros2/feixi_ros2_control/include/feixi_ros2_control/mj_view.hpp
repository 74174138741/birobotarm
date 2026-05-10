#pragma once

#include <GLFW/glfw3.h>

#if __has_include(<mujoco/mujoco.h>)
#include <mujoco/mujoco.h>
#else
#include <mujoco.h>
#endif

enum class MjCameraPreset { Default, Side, Front, Top, Count };

class MjView {
   public:
    MjView() = default;
    ~MjView();

    MjView(MjView const&) = delete;
    MjView& operator=(MjView const&) = delete;

    bool init(mjModel* model, int width, int height, char const* title);
    void shutdown();

    void install_interaction(mjModel* model);

    void init_camera(MjCameraPreset preset);
    void cycle_camera_preset();

    GLFWwindow* window{};
    mjvScene scn{};
    mjvCamera cam{};
    mjrContext con{};
    mjvOption opt{};

   private:
    static MjView* self_from(GLFWwindow* window);
    static void cb_mouse_button(GLFWwindow* window, int button, int action, int mods);
    static void cb_cursor_pos(GLFWwindow* window, double xpos, double ypos);
    static void cb_scroll(GLFWwindow* window, double xoffset, double yoffset);
    static void cb_key(GLFWwindow* window, int key, int scancode, int action, int mods);

    void sync_mouse_buttons(GLFWwindow* window);
    void on_cursor_pos(GLFWwindow* window, double xpos, double ypos);
    void on_scroll(double xoffset, double yoffset);
    void on_key(int key, int action);

    bool viz_ok_{};
    mjModel* model_{};
    bool mouse_left_{};
    bool mouse_middle_{};
    bool mouse_right_{};
    double last_mouse_x_{};
    double last_mouse_y_{};
    MjCameraPreset camera_preset_{MjCameraPreset::Default};
};

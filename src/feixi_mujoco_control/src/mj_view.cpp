#include <cstdio>

#include "feixi_mujoco_control/mj_view.hpp"

MjView* MjView::self_from(GLFWwindow* window) {
    return static_cast<MjView*>(glfwGetWindowUserPointer(window));
}

void MjView::cb_mouse_button(GLFWwindow* window, int button, int action, int mods) {
    (void)button;
    (void)action;
    (void)mods;
    MjView* self = self_from(window);
    if (self) {
        self->sync_mouse_buttons(window);
    }
}

void MjView::cb_cursor_pos(GLFWwindow* window, double xpos, double ypos) {
    MjView* self = self_from(window);
    if (self) {
        self->on_cursor_pos(window, xpos, ypos);
    }
}

void MjView::cb_scroll(GLFWwindow* window, double xoffset, double yoffset) {
    MjView* self = self_from(window);
    if (self) {
        self->on_scroll(xoffset, yoffset);
    }
}

void MjView::cb_key(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode;
    (void)mods;
    MjView* self = self_from(window);
    if (self) {
        self->on_key(key, action);
    }
}

MjView::~MjView() {
    shutdown();
}

bool MjView::init(mjModel* model, int width, int height, char const* title) {
    if (!glfwInit()) {
        char const* msg = nullptr;
        glfwGetError(&msg);
        if (msg) {
            fputs(msg, stderr);
            fputc('\n', stderr);
        }
        return false;
    }

    window = glfwCreateWindow(width, height, title, nullptr, nullptr);
    if (!window) {
        char const* msg = nullptr;
        glfwGetError(&msg);
        if (msg) {
            fputs(msg, stderr);
            fputc('\n', stderr);
        }
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultScene(&scn);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    opt.geomgroup[3] = 0;

    mjr_defaultContext(&con);

    mjv_makeScene(model, &scn, 1000);
    mjr_makeContext(model, &con, mjFONTSCALE_100);

    viz_ok_ = true;
    return true;
}

void MjView::init_camera(MjCameraPreset preset) {
    if (preset == MjCameraPreset::Count) {
        preset = MjCameraPreset::Default;
    }
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FREE;
    cam.lookat[0] = 0.0;
    cam.lookat[1] = 0.0;
    cam.lookat[2] = 0.45;

    switch (preset) {
        case MjCameraPreset::Default:
            cam.distance = 3.0;
            cam.azimuth = 90.0;
            cam.elevation = -20.0;
            break;
        case MjCameraPreset::Side:
            cam.distance = 2.8;
            cam.azimuth = 0.0;
            cam.elevation = -15.0;
            break;
        case MjCameraPreset::Front:
            cam.distance = 2.8;
            cam.azimuth = -90.0;
            cam.elevation = -18.0;
            break;
        case MjCameraPreset::Top:
            cam.distance = 3.2;
            cam.azimuth = 90.0;
            cam.elevation = -85.0;
            break;
        case MjCameraPreset::Count:
            break;
    }
    camera_preset_ = preset;
}

void MjView::cycle_camera_preset() {
    int const n = static_cast<int>(MjCameraPreset::Count);
    int i = static_cast<int>(camera_preset_);
    i = (i + 1) % n;
    init_camera(static_cast<MjCameraPreset>(i));
}

void MjView::install_interaction(mjModel* model) {
    model_ = model;
    glfwSetWindowUserPointer(window, this);
    glfwSetMouseButtonCallback(window, &MjView::cb_mouse_button);
    glfwSetCursorPosCallback(window, &MjView::cb_cursor_pos);
    glfwSetScrollCallback(window, &MjView::cb_scroll);
    glfwSetKeyCallback(window, &MjView::cb_key);
}

void MjView::sync_mouse_buttons(GLFWwindow* w) {
    mouse_left_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    mouse_middle_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    mouse_right_ = (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(w, &last_mouse_x_, &last_mouse_y_);
}

void MjView::on_cursor_pos(GLFWwindow* window, double xpos, double ypos) {
    if (!model_) {
        return;
    }
    if (!mouse_left_ && !mouse_middle_ && !mouse_right_) {
        return;
    }

    double const dx = xpos - last_mouse_x_;
    double const dy = ypos - last_mouse_y_;
    last_mouse_x_ = xpos;
    last_mouse_y_ = ypos;

    int width = 0;
    int height = 0;
    glfwGetWindowSize(window, &width, &height);
    if (height <= 0) {
        return;
    }

    bool const mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                            glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

    mjtMouse action = mjMOUSE_NONE;
    if (mouse_right_) {
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    } else if (mouse_left_) {
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    } else {
        action = mjMOUSE_ZOOM;
    }

    mjv_moveCamera(model_, action, static_cast<mjtNum>(dx / height),
                   static_cast<mjtNum>(dy / height), &scn, &cam);
}

void MjView::on_scroll(double xoffset, double yoffset) {
    (void)xoffset;
    if (!model_) {
        return;
    }
    mjv_moveCamera(model_, mjMOUSE_ZOOM, 0, static_cast<mjtNum>(-0.05 * yoffset), &scn, &cam);
}

void MjView::on_key(int key, int action) {
    if (action != GLFW_PRESS) {
        return;
    }
    if (key == GLFW_KEY_C) {
        cycle_camera_preset();
        return;
    }
    if (key >= GLFW_KEY_1 && key <= GLFW_KEY_4) {
        int const i = key - GLFW_KEY_1;
        if (i < static_cast<int>(MjCameraPreset::Count)) {
            init_camera(static_cast<MjCameraPreset>(i));
        }
    }
}

void MjView::shutdown() {
    if (!viz_ok_) {
        return;
    }
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    viz_ok_ = false;

    if (window) {
        glfwDestroyWindow(window);
        window = nullptr;
    }
    glfwTerminate();
}

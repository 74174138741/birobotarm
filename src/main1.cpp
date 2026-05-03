#include <algorithm>
#include <cstddef>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <unistd.h>

#include <GLFW/glfw3.h>

#if __has_include(<mujoco/mujoco.h>)
#include <mujoco/mujoco.h>
#else
#include <mujoco.h>
#endif

namespace fs = std::filesystem;

// 关节空间 PD（motor → 力矩），按 actuator 顺序与 q_des[0..n_des) 对齐
static void apply_motor_pd(mjModel* m, mjData* d, const double* q_des, int n_des, double kp, double kd) {
    const int n = std::min(m->nu, n_des);
    for (int i = 0; i < n; ++i) {
        const int jid = m->actuator_trnid[2 * i];
        if (jid < 0) {
            continue;
        }
        const int qa = m->jnt_qposadr[jid];
        const int va = m->jnt_dofadr[jid];
        double tau = kp * (q_des[i] - d->qpos[qa]) - kd * d->qvel[va];
        tau = mju_clip(tau, m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
        d->ctrl[i] = tau;
    }
}

static fs::path resolve_model_path() {
    char buf[4096];
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0) {
        buf[n] = '\0';
        const fs::path from_exe = fs::path(buf).parent_path() / "feixi/feixi_model.xml";
        if (fs::exists(from_exe)) {
            return fs::absolute(from_exe);
        }
    }
    for (const char* rel : {"models/feixi/feixi_model.xml", "feixi/feixi_model.xml"}) {
        if (fs::exists(rel)) {
            return fs::absolute(rel);
        }
    }
    return {};
}

int main() {
    const fs::path model_path = resolve_model_path();
    if (model_path.empty()) {
        std::cerr << "Model not found (expected feixi/feixi_model.xml next to the binary, or "
                     "models/feixi/feixi_model.xml from cwd).\n";
        return 1;
    }

    char error[1024] = {0};
    mjModel* model = mj_loadXML(model_path.string().c_str(), nullptr, error, sizeof(error));
    if (!model) {
        std::cerr << error << std::endl;
        return 1;
    }

    mjData* data = mj_makeData(model);

    const double q_init[] = {0.10, -0.30, 0.20, -0.10, 0.15, 0.40, -0.20};
    constexpr int k_dof = static_cast<int>(sizeof(q_init) / sizeof(q_init[0]));
    const int ncopy = std::min(k_dof, model->nq);
    std::memcpy(data->qpos, q_init, static_cast<size_t>(ncopy) * sizeof(double));
    mj_forward(model, data);

    if (!glfwInit()) {
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    GLFWwindow* window = glfwCreateWindow(800, 600, "MuJoCo", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }
    glfwMakeContextCurrent(window);

    mjvScene scn{};
    mjvCamera cam{};
    mjrContext con{};
    mjvOption opt{};

    mjv_defaultScene(&scn);
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);

    mjv_makeScene(model, &scn, 1000);
    mjr_makeContext(model, &con, mjFONTSCALE_100);

    cam.distance = 3.0;
    cam.azimuth = 90;
    cam.elevation = -20;

    constexpr double kp = 1200.0;
    constexpr double kd = 90.0;

    while (!glfwWindowShouldClose(window)) {
        apply_motor_pd(model, data, q_init, k_dof, kp, kd);
        mj_step(model, data);

        int w = 0;
        int h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        const mjrRect viewport = {0, 0, w, h};

        mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwDestroyWindow(window);
    glfwTerminate();
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}

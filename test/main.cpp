#include <algorithm>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include "mj_view.hpp"

static void apply_motor_pd(mjModel* m, mjData* d, double const* q_des, int n_des, double kp,

                           double kd) {
    for (int i = 0; i < m->nu; ++i) {
        d->ctrl[i] = 0;
    }
    int const n = std::min(m->nu, n_des);
    for (int i = 0; i < n; ++i) {
        if (m->actuator_trntype[i] != mjTRN_JOINT) {
            continue;
        }
        int const jid = m->actuator_trnid[2 * i];
        if (jid < 0) {
            continue;
        }
        int const qa = m->jnt_qposadr[jid];
        int const va = m->jnt_dofadr[jid];
        double tau = kp * (q_des[i] - d->qpos[qa]) - kd * d->qvel[va];
        tau = mju_clip(tau, m->actuator_ctrlrange[2 * i], m->actuator_ctrlrange[2 * i + 1]);
        d->ctrl[i] = tau;
    }
}


int main() {
    char err[1024];
    std::memset(err, 0, sizeof(err));
    // Absolute main MJCF path avoids MuJoCo doubling the scene directory with nested includes.
    std::filesystem::path const xml =
        std::filesystem::absolute(std::filesystem::path("scenes/arm_with_gripper.xml"));
    mjModel* model = mj_loadXML(xml.string().c_str(), nullptr, err, sizeof(err));
    if (!model) {
        fputs(err, stderr);
        fputc('\n', stderr);
        return 1;
    }

    mjData* data = mj_makeData(model);

    double const q_init[] = {0.10, -0.30, 0.20, -0.10, 0.15, 0.40, -0.20};
    constexpr int k_dof = static_cast<int>(sizeof(q_init) / sizeof(q_init[0]));
    int const ncopy = std::min(k_dof, model->nq);
    std::memcpy(data->qpos, q_init, static_cast<std::size_t>(ncopy) * sizeof(q_init[0]));
    mj_forward(model, data);

    MjView view;
    if (!view.init(model, 800, 600, "MuJoCo")) {
        fputs("GLFW init / window failed (need local graphics: DISPLAY, not plain SSH)\n", stderr);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 1;
    }

    view.install_interaction(model);
    view.init_camera(MjCameraPreset::Default);

    constexpr double kp = 1200.0;
    constexpr double kd = 90.0;

    while (!glfwWindowShouldClose(view.window)) {
        apply_motor_pd(model, data, q_init, k_dof, kp, kd);
        mj_step(model, data);

        int w = 0;
        int h = 0;

        glfwGetFramebufferSize(view.window, &w, &h);
        mjrRect const viewport = {0, 0, w, h};

        mjv_updateScene(model, data, &view.opt, nullptr, &view.cam, mjCAT_ALL, &view.scn);
        mjr_render(viewport, &view.scn, &view.con);


        glfwSwapBuffers(view.window);
        glfwPollEvents();
    }

    view.shutdown();
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}

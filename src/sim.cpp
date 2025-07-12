#include <memory>
#include <iostream>

#include "RigidBody.hpp"
#include "RobotArm.hpp"

#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"

/*************************************************************************
*
*                           Global Variables
*
*************************************************************************/
mjModel* model = NULL;
mjData* data = NULL;

mjvCamera camera;   // abstract camera
mjvOption option;   // visualization options
mjvScene scene;     // abstract scene
mjrContext context; // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


/*************************************************************************
*
*                             Callbacks
*
*************************************************************************/
// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  // backspace: reset simulation
  if (act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE) {
    mj_resetData(model, data);
    mj_forward(model, data);
  }
}


// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  // update button state
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

  // update mouse position
  glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  // no buttons down: nothing to do
  if (!button_left && !button_middle && !button_right) {
    return;
  }

  // compute mouse displacement, save
  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  // get current window size
  int width, height;
  glfwGetWindowSize(window, &width, &height);

  // get shift key state
  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

  // determine action based on mouse button
  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  // move camera
  mjv_moveCamera(model, action, dx/height, dy/height, &scene, &camera);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  // emulate vertical mouse motion = 5% of window height
  mjv_moveCamera(model, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scene, &camera);
}

GLFWwindow* initializeGLFW() {
  // create window, make OpenGL context current, request v-sync
  GLFWwindow* window = glfwCreateWindow(1200, 900, "Robot Arm Demo", NULL, NULL);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // initialize visualization data structures
  mjv_defaultCamera(&camera);
  mjv_defaultOption(&option);
  mjv_defaultScene(&scene);
  mjr_defaultContext(&context);

  // create scene and context
  mjv_makeScene(model, &scene, 2000);
  mjr_makeContext(model, &context, mjFONTSCALE_150);

  // install GLFW mouse and keyboard callbacks
  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  return window;
}


void start_sim() {
  char error[1000] = "Could not load scene.xml";
  model = mj_loadXML("models/scene.xml", NULL, error, 1000);
  
  if (!model) {
    mju_error("Load model error: %s", error);
  }
  
  data = mj_makeData(model);

  // init GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }
  
  GLFWwindow* window = initializeGLFW();

  Eigen::Matrix4d M {
    {1.0, 0.0, 0.0, 0.536494},
    {0.0, 1.0, 0.0, 0.0     },
    {0.0, 0.0, 1.0, 0.42705 },
    {0.0, 0.0, 0.0, 1.0 }
  };

  Eigen::Matrix<double, 6, 6> Slist {
    {0.0,   0.0,   1.0,      0.0,     0.0,     0.0  },
    {0.0,   1.0,   0.0,   -0.12705,   0.0,     0.0  },
    {0.0,   1.0,   0.0,   -0.42705,   0.0,     0.05955 },
    {1.0,   0.0,   0.0,      0.0,   0.42705,   0.0 },
    {0.0,   1.0,   0.0,   -0.42705,   0.0,     0.35955  },
    {1.0,   0.0,   0.0,      0.0,   0.42705,   0.0  },
  };

  RobotArm<6> robotArm(M, Slist);

  Eigen::Matrix4d T_desired {
    {0.0, -1.0, 0.0, 0.0},
    {1.0,  0.0, 0.0, 0.4},
    {0.0,  0.0, 1.0, 0.4},
    {0.0,  0.0, 0.0, 1.0}
  };

  Eigen::Vector<double, 6> angles {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  robotArm.inverseKinSpace(
    T_desired, angles
  );

  // run main loop, target real-time simulation and 60 fps rendering
  while (!glfwWindowShouldClose(window)) {
    for (int i = 0; i < 6; i++) {
      data->ctrl[i] = angles(i);
    }
    // advance interactive simulation for 1/60 sec
    //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
    //  this loop will finish on time for the next frame to be rendered at 60 fps.
    //  Otherwise add a cpu timer and exit this loop when it is time to render.
    mjtNum simstart = data->time;
    while (data->time - simstart < 1.0 / 60.0) {
      mj_step(model, data);
    }

    // get framebuffer viewport
    mjrRect viewport = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    // update scene and render
    mjv_updateScene(model, data, &option, NULL, &camera, mjCAT_ALL, &scene);
    mjr_render(viewport, &scene, &context);

    // swap OpenGL buffers (blocking call due to v-sync)
    glfwSwapBuffers(window);

    // process pending GUI events, call GLFW callbacks
    glfwPollEvents();
  }

  //free visualization storage
  mjv_freeScene(&scene);
  mjr_freeContext(&context);

  // deallocate existing mjModel
  mj_deleteModel(model);

  // deallocate existing mjData
  mj_deleteData(data);
}

int main(int argc, char* argv[]) {
  start_sim();
  return 0;
}
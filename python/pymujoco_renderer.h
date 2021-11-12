#ifndef PY_MUJOCO_RENDERER_H
#define PY_MUJOCO_RENDERER_H

struct _mjModel;
typedef struct _mjModel mjModel;
struct _mjData;
typedef struct _mjData mjData;

void py_mjv_init();
void py_mjv_exit();
bool py_mjv_render(mjModel* model,  mjData* data);

#endif //PY_MUJOCO_RENDERER_H
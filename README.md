# pymujoco [![Software License](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0) 

`pymujoco`-python bindings for [MuJoCo](http://mujoco.org/) simulator generated using pybind11.

There is also a preliminary [colab](https://colab.research.google.com/drive/1jJfwcBDRcY3oXLawX2HLDFTDOIyHNdKQ?usp=sharing) that allows headless rendering using EGL (cannot restart the EGL context at the moment).

## Usage

```
from pymujoco import *
r = mjv_create_renderer()

#help(p)
m = mj_loadXML("humanoid.xml")
#print(m.qpos0)
#print(m.nq)
#print(m.nv)
d = mj_makeData(m)

exit_requested=False
while not exit_requested:
  mj_step(m,d)
  
  print(d.qpos)
  exit_requested = r.render(m,d)
  
names = ''.join([row.tostring().decode('UTF-8') for row in m.names]).split('\x00')
print("names=",names)
mj_printModel(m, "humanoid2.txt")
mj_deleteData(d)
mj_deleteModel(m)

mjv_delete_renderer(r)
```

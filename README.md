# pymujoco [![Software License](https://img.shields.io/hexpm/l/plug.svg)](https://www.apache.org/licenses/LICENSE-2.0) 

`pymujoco`-python bindings for [MuJoCo](http://mujoco.org/) simulator generated using pybind11.

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

## [Changelog](#GrocerEase)

Major releases, PR's, and Issues are listed here.

| Added/Modified      | Author(s)    |                      Details                     | Date        |
| :-----------------: | :----------: | :----------------------------------------------: | :---------: |     
| Release 0.0.1       | Erwin Coumans| Compiles on `Windows` with Python 3.7            | 11/11/21    |
| Release 0.0.2       | Erwin Coumans| Compiles on `Windows` with Python 3.8            | 16/11/21    |


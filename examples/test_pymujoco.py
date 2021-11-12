from pymujoco import *
r = mjv_create_renderer()

#help(p)
m = mj_loadXML("humanoid.xml")
print(m.nq)
print(m.nv)
d = mj_makeData(m)

exit_requested=False
while not exit_requested:
  mj_step(m,d)
  exit_requested = r.render(m,d)
  
mj_printModel(m, "humanoid2.txt")
mj_deleteData(d)
mj_deleteModel(m)

mjv_delete_renderer(r)
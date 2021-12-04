from pymujoco import *
r = mjv_create_renderer()
import numpy as np
np.set_printoptions(suppress=True, precision=3)
#help(p)
m = mj_loadXML("humanoid100.xml")
#print(m.qpos0)
#print(m.nq)
#print(m.nv)
d = mj_makeData(m)

exit_requested=False
while not exit_requested:
  mj_step(m,d)
  
  #print(d.qpos)
  print("number of contacts:",d.ncon)
  for c in range(int(d.ncon[0])):
    ct = d.get_contact(c)
    print("contact[",c,"].dist=",ct.dist)
    print("contact[",c,"].pos=",ct.pos)
    
  exit_requested = r.render(m,d)
  
names = ''.join([row.tostring().decode('UTF-8') for row in m.names]).split('\x00')
print("names=",names)
mj_printModel(m, "humanoid2.txt")
mj_deleteData(d)
mj_deleteModel(m)

mjv_delete_renderer(r)
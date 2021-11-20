// Copyright 2020 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "pymujoco_includes.h"
#include "pymujoco_renderer.h"

//#include "D:\dev\VisualLeakDetector\include\vld.h"

// pymujoco is work in progress, only tested on Windows (Linux and MacOS will follow soon)
// need to run some script to autogenerate py::array_t bindings for below (and mjdata structures)
// also need to finish the API binding wrappers in pymujoco.inl
// here is an example script that should work
// the mjv_create_renderer/mjv_delete_renderer_r and r.render is optional

#if 0
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

#endif


namespace py = pybind11;

struct PyMjModel
{
    mjModel* m_;

    int nq;                         // number of generalized coordinates = dim(qpos)
    int nv;                         // number of degrees of freedom = dim(qvel)
    int nu;                         // number of actuators/controls = dim(ctrl)
    int na;                         // number of activation states = dim(act)
    int nbody;                      // number of bodies
    int njnt;                       // number of joints
    int ngeom;                      // number of geoms
    int nsite;                      // number of sites
    int ncam;                       // number of cameras
    int nlight;                     // number of lights
    int nmesh;                      // number of meshes
    int nmeshvert;                  // number of vertices in all meshes
    int nmeshtexvert;               // number of vertices with texcoords in all meshes
    int nmeshface;                  // number of triangular faces in all meshes
    int nmeshgraph;                 // number of ints in mesh auxiliary data
    int nskin;                      // number of skins
    int nskinvert;                  // number of vertices in all skins
    int nskintexvert;               // number of vertiex with texcoords in all skins
    int nskinface;                  // number of triangular faces in all skins
    int nskinbone;                  // number of bones in all skins
    int nskinbonevert;              // number of vertices in all skin bones
    int nhfield;                    // number of heightfields
    int nhfielddata;                // number of data points in all heightfields
    int ntex;                       // number of textures
    int ntexdata;                   // number of bytes in texture rgb data
    int nmat;                       // number of materials
    int npair;                      // number of predefined geom pairs
    int nexclude;                   // number of excluded geom pairs
    int neq;                        // number of equality constraints
    int ntendon;                    // number of tendons
    int nwrap;                      // number of wrap objects in all tendon paths
    int nsensor;                    // number of sensors
    int nnumeric;                   // number of numeric custom fields
    int nnumericdata;               // number of mjtNums in all numeric fields
    int ntext;                      // number of text custom fields
    int ntextdata;                  // number of mjtBytes in all text fields
    int ntuple;                     // number of tuple custom fields
    int ntupledata;                 // number of objects in all tuple fields
    int nkey;                       // number of keyframes
    int nmocap;                     // number of mocap bodies
    int nuser_body;                 // number of mjtNums in body_user
    int nuser_jnt;                  // number of mjtNums in jnt_user
    int nuser_geom;                 // number of mjtNums in geom_user
    int nuser_site;                 // number of mjtNums in site_user
    int nuser_cam;                  // number of mjtNums in cam_user
    int nuser_tendon;               // number of mjtNums in tendon_user
    int nuser_actuator;             // number of mjtNums in actuator_user
    int nuser_sensor;               // number of mjtNums in sensor_user
    int nnames;                     // number of chars in all names

    // sizes set after mjModel construction (only affect mjData)
    int nM;                         // number of non-zeros in sparse inertia matrix
    int nemax;                      // number of potential equality-constraint rows
    int njmax;                      // number of available rows in constraint Jacobian
    int nconmax;                    // number of potential contacts in contact list
    int nstack;                     // number of fields in mjData stack
    int nuserdata;                  // number of extra fields in mjData
    int nsensordata;                // number of fields in sensor data vector
    int nbuffer;                    // number of bytes in buffer

    py::array_t<float> qpos0;       // qpos values at default pose              (nq x 1)
    py::array_t<float> qpos_spring; // reference pose for springs               (nq x 1)
    py::array_t<int> body_parentid; // id of body's parent                      (nbody x 1)
    py::array_t<int> body_rootid;   // id of root above body                    (nbody x 1)
    py::array_t<int> body_weldid;   // id of body that this body is welded to   (nbody x 1)
    py::array_t<int> body_mocapid;  // id of mocap data; -1: none               (nbody x 1)
    py::array_t<int> body_jntnum;   // number of joints for this body           (nbody x 1)
    py::array_t<int> body_jntadr;   // start addr of joints; -1: no joints      (nbody x 1)
    py::array_t<int> body_dofnum;   // number of motion degrees of freedom      (nbody x 1)
    py::array_t<int> body_dofadr;   // start addr of dofs; -1: no dofs          (nbody x 1)
    py::array_t<int> body_geomnum;  // number of geoms                          (nbody x 1)
    py::array_t<int> body_geomadr;  // start addr of geoms; -1: no geoms        (nbody x 1)
        
    py::array_t<mjtByte>  body_simple;          // body is simple (has diagonal M)          (nbody x 1)
    py::array_t<mjtByte>  body_sameframe;       // inertial frame is same as body frame     (nbody x 1)
    py::array_t<mjtNum>   body_pos;             // position offset rel. to parent body      (nbody x 3)
    py::array_t<mjtNum>   body_quat;            // orientation offset rel. to parent body   (nbody x 4)
    py::array_t<mjtNum>   body_ipos;            // local position of center of mass         (nbody x 3)
    py::array_t<mjtNum>   body_iquat;           // local orientation of inertia ellipsoid   (nbody x 4)
    py::array_t<mjtNum>   body_mass;            // mass                                     (nbody x 1)
    py::array_t<mjtNum>   body_subtreemass;     // mass of subtree starting at this body    (nbody x 1)
    py::array_t<mjtNum>   body_inertia;         // diagonal inertia in ipos/iquat frame     (nbody x 3)
    py::array_t<mjtNum>   body_invweight0;      // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
    py::array_t<mjtNum>   body_user;            // user data                                (nbody x nuser_body)

    // joints
    py::array_t<int>      jnt_type;             // type of joint (mjtJoint)                 (njnt x 1)
    py::array_t<int>      jnt_qposadr;          // start addr in 'qpos' for joint's data    (njnt x 1)
    py::array_t<int>      jnt_dofadr;           // start addr in 'qvel' for joint's data    (njnt x 1)
    py::array_t<int>      jnt_bodyid;           // id of joint's body                       (njnt x 1)
    py::array_t<int>      jnt_group;            // group for visibility                     (njnt x 1)
    py::array_t<mjtByte>  jnt_limited;          // does joint have limits                   (njnt x 1)
    py::array_t<mjtNum>   jnt_solref;           // constraint solver reference: limit       (njnt x mjNREF)
    py::array_t<mjtNum>   jnt_solimp;           // constraint solver impedance: limit       (njnt x mjNIMP)
    py::array_t<mjtNum>   jnt_pos;              // local anchor position                    (njnt x 3)
    py::array_t<mjtNum>   jnt_axis;             // local joint axis                         (njnt x 3)
    py::array_t<mjtNum>   jnt_stiffness;        // stiffness coefficient                    (njnt x 1)
    py::array_t<mjtNum>   jnt_range;            // joint limits                             (njnt x 2)
    py::array_t<mjtNum>   jnt_margin;           // min distance for limit detection         (njnt x 1)
    py::array_t<mjtNum>   jnt_user;             // user data                                (njnt x nuser_jnt)

    // dofs
    py::array_t<int>      dof_bodyid;           // id of dof's body                         (nv x 1)
    py::array_t<int>      dof_jntid;            // id of dof's joint                        (nv x 1)
    py::array_t<int>      dof_parentid;         // id of dof's parent; -1: none             (nv x 1)
    py::array_t<int>      dof_Madr;             // dof address in M-diagonal                (nv x 1)
    py::array_t<int>      dof_simplenum;        // number of consecutive simple dofs        (nv x 1)
    py::array_t<mjtNum>   dof_solref;           // constraint solver reference:frictionloss (nv x mjNREF)
    py::array_t<mjtNum>   dof_solimp;           // constraint solver impedance:frictionloss (nv x mjNIMP)
    py::array_t<mjtNum>   dof_frictionloss;     // dof friction loss                        (nv x 1)
    py::array_t<mjtNum>   dof_armature;         // dof armature inertia/mass                (nv x 1)
    py::array_t<mjtNum>   dof_damping;          // damping coefficient                      (nv x 1)
    py::array_t<mjtNum>   dof_invweight0;       // diag. inverse inertia in qpos0           (nv x 1)
    py::array_t<mjtNum>   dof_M0;               // diag. inertia in qpos0                   (nv x 1)

    // geoms
    py::array_t<int>      geom_type;            // geometric type (mjtGeom)                 (ngeom x 1)
    py::array_t<int>      geom_contype;         // geom contact type                        (ngeom x 1)
    py::array_t<int>      geom_conaffinity;     // geom contact affinity                    (ngeom x 1)
    py::array_t<int>      geom_condim;          // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
    py::array_t<int>      geom_bodyid;          // id of geom's body                        (ngeom x 1)
    py::array_t<int>      geom_dataid;          // id of geom's mesh/hfield (-1: none)      (ngeom x 1)
    py::array_t<int>      geom_matid;           // material id for rendering                (ngeom x 1)
    py::array_t<int>      geom_group;           // group for visibility                     (ngeom x 1)
    py::array_t<int>      geom_priority;        // geom contact priority                    (ngeom x 1)
    py::array_t<mjtByte>  geom_sameframe;       // same as body frame (1) or iframe (2)     (ngeom x 1)
    py::array_t<mjtNum>   geom_solmix;          // mixing coef for solref/imp in geom pair  (ngeom x 1)
    py::array_t<mjtNum>   geom_solref;          // constraint solver reference: contact     (ngeom x mjNREF)
    py::array_t<mjtNum>   geom_solimp;          // constraint solver impedance: contact     (ngeom x mjNIMP)
    py::array_t<mjtNum>   geom_size;            // geom-specific size parameters            (ngeom x 3)
    py::array_t<mjtNum>   geom_rbound;          // radius of bounding sphere                (ngeom x 1)
    py::array_t<mjtNum>   geom_pos;             // local position offset rel. to body       (ngeom x 3)
    py::array_t<mjtNum>   geom_quat;            // local orientation offset rel. to body    (ngeom x 4)
    py::array_t<mjtNum>   geom_friction;        // friction for (slide, spin, roll)         (ngeom x 3)
    py::array_t<mjtNum>   geom_margin;          // detect contact if dist<margin            (ngeom x 1)
    py::array_t<mjtNum>   geom_gap;             // include in solver if dist<margin-gap     (ngeom x 1)
    py::array_t<mjtNum>   geom_user;            // user data                                (ngeom x nuser_geom)
    py::array_t<float>    geom_rgba;            // rgba when material is omitted            (ngeom x 4)


    // sites
    py::array_t<int>      site_type;            // geom type for rendering (mjtGeom)        (nsite x 1)
    py::array_t<int>      site_bodyid;          // id of site's body                        (nsite x 1)
    py::array_t<int>      site_matid;           // material id for rendering                (nsite x 1)
    py::array_t<int>      site_group;           // group for visibility                     (nsite x 1)
    py::array_t<mjtByte>  site_sameframe;       // same as body frame (1) or iframe (2)     (nsite x 1)
    py::array_t<mjtNum>   site_size;            // geom size for rendering                  (nsite x 3)
    py::array_t<mjtNum>   site_pos;             // local position offset rel. to body       (nsite x 3)
    py::array_t<mjtNum>   site_quat;            // local orientation offset rel. to body    (nsite x 4)
    py::array_t<mjtNum>   site_user;            // user data                                (nsite x nuser_site)
    py::array_t<float>    site_rgba;            // rgba when material is omitted            (nsite x 4)

#if 0
    // cameras
    py::array_t<int>      cam_mode;             // camera tracking mode (mjtCamLight)       (ncam x 1)
    py::array_t<int>      cam_bodyid;           // id of camera's body                      (ncam x 1)
    py::array_t<int>      cam_targetbodyid;     // id of targeted body; -1: none            (ncam x 1)
    py::array_t<mjtNum>   cam_pos;              // position rel. to body frame              (ncam x 3)
    py::array_t<mjtNum>   cam_quat;             // orientation rel. to body frame           (ncam x 4)
    py::array_t<mjtNum>   cam_poscom0;          // global position rel. to sub-com in qpos0 (ncam x 3)
    py::array_t<mjtNum>   cam_pos0;             // global position rel. to body in qpos0    (ncam x 3)
    py::array_t<mjtNum>   cam_mat0;             // global orientation in qpos0              (ncam x 9)
    py::array_t<mjtNum>   cam_fovy;             // y-field of view (deg)                    (ncam x 1)
    py::array_t<mjtNum>   cam_ipd;              // inter-pupilary distance                  (ncam x 1)
    py::array_t<mjtNum>   cam_user;             // user data                                (ncam x nuser_cam)

    // lights
    py::array_t<int>      light_mode;           // light tracking mode (mjtCamLight)        (nlight x 1)
    py::array_t<int>      light_bodyid;         // id of light's body                       (nlight x 1)
    py::array_t<int>      light_targetbodyid;   // id of targeted body; -1: none            (nlight x 1)
    py::array_t<mjtByte>  light_directional;    // directional light                        (nlight x 1)
    py::array_t<mjtByte>  light_castshadow;     // does light cast shadows                  (nlight x 1)
    py::array_t<mjtByte>  light_active;         // is light on                              (nlight x 1)
    py::array_t<mjtNum>   light_pos;            // position rel. to body frame              (nlight x 3)
    py::array_t<mjtNum>   light_dir;            // direction rel. to body frame             (nlight x 3)
    py::array_t<mjtNum>   light_poscom0;        // global position rel. to sub-com in qpos0 (nlight x 3)
    py::array_t<mjtNum>   light_pos0;           // global position rel. to body in qpos0    (nlight x 3)
    py::array_t<mjtNum>   light_dir0;           // global direction in qpos0                (nlight x 3)
    py::array_t<float>    light_attenuation;    // OpenGL attenuation (quadratic model)     (nlight x 3)
    py::array_t<float>    light_cutoff;         // OpenGL cutoff                            (nlight x 1)
    py::array_t<float>    light_exponent;       // OpenGL exponent                          (nlight x 1)
    py::array_t<float>    light_ambient;        // ambient rgb (alpha=1)                    (nlight x 3)
    py::array_t<float>    light_diffuse;        // diffuse rgb (alpha=1)                    (nlight x 3)
    py::array_t<float>    light_specular;       // specular rgb (alpha=1)                   (nlight x 3)

    // meshes
    py::array_t<int>      mesh_vertadr;         // first vertex address                     (nmesh x 1)
    py::array_t<int>      mesh_vertnum;         // number of vertices                       (nmesh x 1)
    py::array_t<int>      mesh_texcoordadr;     // texcoord data address; -1: no texcoord   (nmesh x 1)
    py::array_t<int>      mesh_faceadr;         // first face address                       (nmesh x 1)
    py::array_t<int>      mesh_facenum;         // number of faces                          (nmesh x 1)
    py::array_t<int>      mesh_graphadr;        // graph data address; -1: no graph         (nmesh x 1)
    py::array_t<float>    mesh_vert;            // vertex positions for all meshe           (nmeshvert x 3)
    py::array_t<float>    mesh_normal;          // vertex normals for all meshes            (nmeshvert x 3)
    py::array_t<float>    mesh_texcoord;        // vertex texcoords for all meshes          (nmeshtexvert x 2)
    py::array_t<int>      mesh_face;            // triangle face data                       (nmeshface x 3)
    py::array_t<int>      mesh_graph;           // convex graph data                        (nmeshgraph x 1)

    // skins
    py::array_t<int>      skin_matid;           // skin material id; -1: none               (nskin x 1)
    py::array_t<float>    skin_rgba;            // skin rgba                                (nskin x 4)
    py::array_t<float>    skin_inflate;         // inflate skin in normal direction         (nskin x 1)
    py::array_t<int>      skin_vertadr;         // first vertex address                     (nskin x 1)
    py::array_t<int>      skin_vertnum;         // number of vertices                       (nskin x 1)
    py::array_t<int>      skin_texcoordadr;     // texcoord data address; -1: no texcoord   (nskin x 1)
    py::array_t<int>      skin_faceadr;         // first face address                       (nskin x 1)
    py::array_t<int>      skin_facenum;         // number of faces                          (nskin x 1)
    py::array_t<int>      skin_boneadr;         // first bone in skin                       (nskin x 1)
    py::array_t<int>      skin_bonenum;         // number of bones in skin                  (nskin x 1)
    py::array_t<float>    skin_vert;            // vertex positions for all skin meshes     (nskinvert x 3)
    py::array_t<float>    skin_texcoord;        // vertex texcoords for all skin meshes     (nskintexvert x 2)
    py::array_t<int>      skin_face;            // triangle faces for all skin meshes       (nskinface x 3)
    py::array_t<int>      skin_bonevertadr;     // first vertex in each bone                (nskinbone x 1)
    py::array_t<int>      skin_bonevertnum;     // number of vertices in each bone          (nskinbone x 1)
    py::array_t<float>    skin_bonebindpos;     // bind pos of each bone                    (nskinbone x 3)
    py::array_t<float>    skin_bonebindquat;    // bind quat of each bone                   (nskinbone x 4)
    py::array_t<int>      skin_bonebodyid;      // body id of each bone                     (nskinbone x 1)
    py::array_t<int>      skin_bonevertid;      // mesh ids of vertices in each bone        (nskinbonevert x 1)
    py::array_t<float>    skin_bonevertweight;  // weights of vertices in each bone         (nskinbonevert x 1)

    // height fields
    py::array_t<mjtNum>   hfield_size;          // (x, y, z_top, z_bottom)                  (nhfield x 4)
    py::array_t<int>      hfield_nrow;          // number of rows in grid                   (nhfield x 1)
    py::array_t<int>      hfield_ncol;          // number of columns in grid                (nhfield x 1)
    py::array_t<int>      hfield_adr;           // address in hfield_data                   (nhfield x 1)
    py::array_t<float>    hfield_data;          // elevation data                           (nhfielddata x 1)

    // textures
    py::array_t<int>      tex_type;             // texture type (mjtTexture)                (ntex x 1)
    py::array_t<int>      tex_height;           // number of rows in texture image          (ntex x 1)
    py::array_t<int>      tex_width;            // number of columns in texture image       (ntex x 1)
    py::array_t<int>      tex_adr;              // address in rgb                           (ntex x 1)
    py::array_t<mjtByte>  tex_rgb;              // rgb (alpha = 1)                          (ntexdata x 1)

    // materials
    py::array_t<int>      mat_texid;            // texture id; -1: none                     (nmat x 1)
    py::array_t<mjtByte>  mat_texuniform;       // make texture cube uniform                (nmat x 1)
    py::array_t<float>    mat_texrepeat;        // texture repetition for 2d mapping        (nmat x 2)
    py::array_t<float>    mat_emission;         // emission (x rgb)                         (nmat x 1)
    py::array_t<float>    mat_specular;         // specular (x white)                       (nmat x 1)
    py::array_t<float>    mat_shininess;        // shininess coef                           (nmat x 1)
    py::array_t<float>    mat_reflectance;      // reflectance (0: disable)                 (nmat x 1)
    py::array_t<float>    mat_rgba;             // rgba                                     (nmat x 4)


    // predefined geom pairs for collision detection; has precedence over exclude
    py::array_t<int>      pair_dim;             // contact dimensionality                   (npair x 1)
    py::array_t<int>      pair_geom1;           // id of geom1                              (npair x 1)
    py::array_t<int>      pair_geom2;           // id of geom2                              (npair x 1)
    py::array_t<int>      pair_signature;       // (body1+1)<<16 + body2+1                  (npair x 1)
    py::array_t<mjtNum>   pair_solref;          // constraint solver reference: contact     (npair x mjNREF)
    py::array_t<mjtNum>   pair_solimp;          // constraint solver impedance: contact     (npair x mjNIMP)
    py::array_t<mjtNum>   pair_margin;          // detect contact if dist<margin            (npair x 1)
    py::array_t<mjtNum>   pair_gap;             // include in solver if dist<margin-gap     (npair x 1)
    py::array_t<mjtNum>   pair_friction;        // tangent1, 2, spin, roll1, 2              (npair x 5)

    // excluded body pairs for collision detection
    py::array_t<int>      exclude_signature;    // (body1+1)<<16 + body2+1                  (nexclude x 1)

    // equality constraints
    py::array_t<int>      eq_type;              // constraint type (mjtEq)                  (neq x 1)
    py::array_t<int>      eq_obj1id;            // id of object 1                           (neq x 1)
    py::array_t<int>      eq_obj2id;            // id of object 2                           (neq x 1)
    py::array_t<mjtByte>  eq_active;            // enable/disable constraint                (neq x 1)
    py::array_t<mjtNum>   eq_solref;            // constraint solver reference              (neq x mjNREF)
    py::array_t<mjtNum>   eq_solimp;            // constraint solver impedance              (neq x mjNIMP)
    py::array_t<mjtNum>   eq_data;              // numeric data for constraint              (neq x mjNEQDATA)

    // tendons
    py::array_t<int>      tendon_adr;           // address of first object in tendon's path (ntendon x 1)
    py::array_t<int>      tendon_num;           // number of objects in tendon's path       (ntendon x 1)
    py::array_t<int>      tendon_matid;         // material id for rendering                (ntendon x 1)
    py::array_t<int>      tendon_group;         // group for visibility                     (ntendon x 1)
    py::array_t<mjtByte>  tendon_limited;       // does tendon have length limits           (ntendon x 1)
    py::array_t<mjtNum>   tendon_width;         // width for rendering                      (ntendon x 1)
    py::array_t<mjtNum>   tendon_solref_lim;    // constraint solver reference: limit       (ntendon x mjNREF)
    py::array_t<mjtNum>   tendon_solimp_lim;    // constraint solver impedance: limit       (ntendon x mjNIMP)
    py::array_t<mjtNum>   tendon_solref_fri;    // constraint solver reference: friction    (ntendon x mjNREF)
    py::array_t<mjtNum>   tendon_solimp_fri;    // constraint solver impedance: friction    (ntendon x mjNIMP)
    py::array_t<mjtNum>   tendon_range;         // tendon length limits                     (ntendon x 2)
    py::array_t<mjtNum>   tendon_margin;        // min distance for limit detection         (ntendon x 1)
    py::array_t<mjtNum>   tendon_stiffness;     // stiffness coefficient                    (ntendon x 1)
    py::array_t<mjtNum>   tendon_damping;       // damping coefficient                      (ntendon x 1)
    py::array_t<mjtNum>   tendon_frictionloss;  // loss due to friction                     (ntendon x 1)
    py::array_t<mjtNum>   tendon_lengthspring;  // tendon length in qpos_spring             (ntendon x 1)
    py::array_t<mjtNum>   tendon_length0;       // tendon length in qpos0                   (ntendon x 1)
    py::array_t<mjtNum>   tendon_invweight0;    // inv. weight in qpos0                     (ntendon x 1)
    py::array_t<mjtNum>   tendon_user;          // user data                                (ntendon x nuser_tendon)
    py::array_t<float>    tendon_rgba;          // rgba when material is omitted            (ntendon x 4)

    // list of all wrap objects in tendon paths
    py::array_t<int>      wrap_type;            // wrap object type (mjtWrap)               (nwrap x 1)
    py::array_t<int>      wrap_objid;           // object id: geom, site, joint             (nwrap x 1)
    py::array_t<mjtNum>   wrap_prm;             // divisor, joint coef, or site id          (nwrap x 1)

    // actuators
    py::array_t<int>      actuator_trntype;     // transmission type (mjtTrn)               (nu x 1)
    py::array_t<int>      actuator_dyntype;     // dynamics type (mjtDyn)                   (nu x 1)
    py::array_t<int>      actuator_gaintype;    // gain type (mjtGain)                      (nu x 1)
    py::array_t<int>      actuator_biastype;    // bias type (mjtBias)                      (nu x 1)
    py::array_t<int>      actuator_trnid;       // transmission id: joint, tendon, site     (nu x 2)
    py::array_t<int>      actuator_group;       // group for visibility                     (nu x 1)
    py::array_t<mjtByte>  actuator_ctrllimited; // is control limited                       (nu x 1)
    py::array_t<mjtByte>  actuator_forcelimited;// is force limited                         (nu x 1)
    py::array_t<mjtNum>   actuator_dynprm;      // dynamics parameters                      (nu x mjNDYN)
    py::array_t<mjtNum>   actuator_gainprm;     // gain parameters                          (nu x mjNGAIN)
    py::array_t<mjtNum>   actuator_biasprm;     // bias parameters                          (nu x mjNBIAS)
    py::array_t<mjtNum>   actuator_ctrlrange;   // range of controls                        (nu x 2)
    py::array_t<mjtNum>   actuator_forcerange;  // range of forces                          (nu x 2)
    py::array_t<mjtNum>   actuator_gear;        // scale length and transmitted force       (nu x 6)
    py::array_t<mjtNum>   actuator_cranklength; // crank length for slider-crank            (nu x 1)
    py::array_t<mjtNum>   actuator_acc0;        // acceleration from unit force in qpos0    (nu x 1)
    py::array_t<mjtNum>   actuator_length0;     // actuator length in qpos0                 (nu x 1)
    py::array_t<mjtNum>   actuator_lengthrange; // feasible actuator length range           (nu x 2)
    py::array_t<mjtNum>   actuator_user;        // user data                                (nu x nuser_actuator)

    // sensors
    py::array_t<int>      sensor_type;          // sensor type (mjtSensor)                  (nsensor x 1)
    py::array_t<int>      sensor_datatype;      // numeric data type (mjtDataType)          (nsensor x 1)
    py::array_t<int>      sensor_needstage;     // required compute stage (mjtStage)        (nsensor x 1)
    py::array_t<int>      sensor_objtype;       // type of sensorized object (mjtObj)       (nsensor x 1)
    py::array_t<int>      sensor_objid;         // id of sensorized object                  (nsensor x 1)
    py::array_t<int>      sensor_dim;           // number of scalar outputs                 (nsensor x 1)
    py::array_t<int>      sensor_adr;           // address in sensor array                  (nsensor x 1)
    py::array_t<mjtNum>   sensor_cutoff;        // cutoff for real and positive; 0: ignore  (nsensor x 1)
    py::array_t<mjtNum>   sensor_noise;         // noise standard deviation                 (nsensor x 1)
    py::array_t<mjtNum>   sensor_user;          // user data                                (nsensor x nuser_sensor)

    // custom numeric fields
    py::array_t<int>      numeric_adr;          // address of field in numeric_data         (nnumeric x 1)
    py::array_t<int>      numeric_size;         // size of numeric field                    (nnumeric x 1)
    py::array_t<mjtNum>   numeric_data;         // array of all numeric fields              (nnumericdata x 1)

    // custom text fields
    py::array_t<int>      text_adr;             // address of text in text_data             (ntext x 1)
    py::array_t<int>      text_size;            // size of text field (strlen+1)            (ntext x 1)
    py::array_t<char>     text_data;            // array of all text fields (0-terminated)  (ntextdata x 1)

    // custom tuple fields
    py::array_t<int>      tuple_adr;            // address of text in text_data             (ntuple x 1)
    py::array_t<int>      tuple_size;           // number of objects in tuple               (ntuple x 1)
    py::array_t<int>      tuple_objtype;        // array of object types in all tuples      (ntupledata x 1)
    py::array_t<int>      tuple_objid;          // array of object ids in all tuples        (ntupledata x 1)
    py::array_t<mjtNum>   tuple_objprm;         // array of object params in all tuples     (ntupledata x 1)

    // keyframes
    py::array_t<mjtNum>   key_time;             // key time                                 (nkey x 1)
    py::array_t<mjtNum>   key_qpos;             // key position                             (nkey x nq)
    py::array_t<mjtNum>   key_qvel;             // key velocity                             (nkey x nv)
    py::array_t<mjtNum>   key_act;              // key activation                           (nkey x na)
    py::array_t<mjtNum>   key_mpos;             // key mocap position                       (nkey x 3*nmocap)
    py::array_t<mjtNum>   key_mquat;            // key mocap quaternion                     (nkey x 4*nmocap)
#endif

    // names
    py::array_t<int>      name_bodyadr;         // body name pointers                       (nbody x 1)
    py::array_t<int>      name_jntadr;          // joint name pointers                      (njnt x 1)
    py::array_t<int>      name_geomadr;         // geom name pointers                       (ngeom x 1)
    py::array_t<int>      name_siteadr;         // site name pointers                       (nsite x 1)
    py::array_t<int>      name_camadr;          // camera name pointers                     (ncam x 1)
    py::array_t<int>      name_lightadr;        // light name pointers                      (nlight x 1)
    py::array_t<int>      name_meshadr;         // mesh name pointers                       (nmesh x 1)
    py::array_t<int>      name_skinadr;         // skin name pointers                       (nskin x 1)
    py::array_t<int>      name_hfieldadr;       // hfield name pointers                     (nhfield x 1)
    py::array_t<int>      name_texadr;          // texture name pointers                    (ntex x 1)
    py::array_t<int>      name_matadr;          // material name pointers                   (nmat x 1)
    py::array_t<int>      name_pairadr;         // geom pair name pointers                  (npair x 1)
    py::array_t<int>      name_excludeadr;      // exclude name pointers                    (nexclude x 1)
    py::array_t<int>      name_eqadr;           // equality constraint name pointers        (neq x 1)
    py::array_t<int>      name_tendonadr;       // tendon name pointers                     (ntendon x 1)
    py::array_t<int>      name_actuatoradr;     // actuator name pointers                   (nu x 1)
    py::array_t<int>      name_sensoradr;       // sensor name pointers                     (nsensor x 1)
    py::array_t<int>      name_numericadr;      // numeric name pointers                    (nnumeric x 1)
    py::array_t<int>      name_textadr;         // text name pointers                       (ntext x 1)
    py::array_t<int>      name_tupleadr;        // tuple name pointers                      (ntuple x 1)
    py::array_t<int>      name_keyadr;          // keyframe name pointers                   (nkey x 1)
    py::array_t<char>     names;                // names of all objects, 0-terminated       (nnames x 1)

    py::capsule buffer_handle;

    PyMjModel(mjModel* m)
        :m_(m),
        buffer_handle(py::capsule ([](){}))
    {
        nq = m->nq;
        nv = m->nv;
        nu = m->nu;
        na = m->na;
        nbody = m->nbody;
        njnt = m->njnt;
        ngeom = m->ngeom;            
        nsite = m->nsite;          
        ncam = m->ncam;
        nlight = m->nlight;
        nmesh = m->nmesh;
        nmeshvert = m->nmeshvert;
        nmeshtexvert = m->nmeshtexvert;
        nmeshface = m->nmeshface;
        nmeshgraph = m->nmeshgraph;
        nskin = m->nskin;
        nskinvert = m->nskinvert;
        nskintexvert = m->nskintexvert;
        nskinface = m->nskinface;
        nskinbone = m->nskinbone;
        nskinbonevert = m->nskinbonevert;
        nhfield = m->nhfield;
        nhfielddata = m->nhfielddata;
        ntex = m->ntex;
        ntexdata = m->ntexdata;
        nmat = m->nmat;
        npair = m->npair;
        nexclude = m->nexclude;                  
        neq = m->neq;                       
        ntendon = m->ntendon;                   
        nwrap = m->nwrap;                     
        nsensor = m->nsensor;                   
        nnumeric = m->nnumeric;                  
        nnumericdata = m->nnumericdata;              
        ntext = m->ntext;                     
        ntextdata = m->ntextdata;                 
        ntuple = m->ntuple;                    
        ntupledata = ntupledata;
        nkey = m->nkey;                      
        nmocap = m->nmocap;                    
        nuser_body = m->nuser_body;                
        nuser_jnt = m->nuser_jnt;                 
        nuser_geom = m->nuser_geom;                
        nuser_site = m->nuser_site;                
        nuser_cam = m->nuser_cam;                 
        nuser_tendon = m->nuser_tendon;              
        nuser_actuator = m->nuser_actuator;            
        nuser_sensor = m->nuser_sensor;              
        nnames = m->nnames;

        // sizes set after mjModel construction (only affect mjData)
        nM = m->nM;                         // number of non-zeros in sparse inertia matrix
        nemax= m->nemax;                      // number of potential equality-constraint rows
        njmax= m->njmax;                      // number of available rows in constraint Jacobian
        nconmax= m->nconmax;                    // number of potential contacts in contact list
        nstack= m->nstack;                     // number of fields in mjData stack
        nuserdata= m->nuserdata;                  // number of extra fields in mjData
        nsensordata= m->nsensordata;                // number of fields in sensor data vector
        nbuffer= m->nbuffer;                    // number of bytes in buffer

        
        // create a mapping between C-arrays and Numpy without copying data
        qpos0 = py::array_t<mjtNum>(size_t(m_->nq), m_->qpos0, buffer_handle);
        qpos_spring = py::array_t<mjtNum>(size_t(m_->nq), m_->qpos_spring, buffer_handle);
        body_parentid = py::array_t<int>(size_t(m_->nbody), m_->body_parentid, buffer_handle);
        body_rootid = py::array_t<int>(size_t(m_->nbody), m_->body_rootid, buffer_handle);
        body_weldid = py::array_t<int>(size_t(m_->nbody), m_->body_weldid, buffer_handle);
        body_mocapid = py::array_t<int>(size_t(m_->nbody), m_->body_mocapid, buffer_handle);
        body_jntnum = py::array_t<int>(size_t(m_->nbody), m_->body_jntnum, buffer_handle);
        body_jntadr = py::array_t<int>(size_t(m_->nbody), m_->body_jntadr, buffer_handle);
        body_dofnum = py::array_t<int>(size_t(m_->nbody), m_->body_dofnum, buffer_handle);
        body_dofadr = py::array_t<int>(size_t(m_->nbody), m_->body_dofadr, buffer_handle);
        body_geomnum = py::array_t<int>(size_t(m_->nbody), m_->body_geomnum, buffer_handle);
        body_geomadr = py::array_t<int>(size_t(m_->nbody), m_->body_geomadr, buffer_handle);
        
        body_simple = py::array_t<mjtByte>(size_t(m_->nbody), m_->body_simple, buffer_handle);
        body_sameframe = py::array_t<mjtByte>(size_t(m_->nbody), m_->body_sameframe, buffer_handle);
        body_pos = py::array_t<mjtNum>(size_t(m_->nbody*3), m_->body_pos, buffer_handle);
        body_quat = py::array_t<mjtNum>(size_t(m_->nbody*4), m_->body_quat, buffer_handle);
        body_ipos = py::array_t<mjtNum>(size_t(m_->nbody*3), m_->body_ipos, buffer_handle);
        body_iquat = py::array_t<mjtNum>(size_t(m_->nbody*4), m_->body_iquat, buffer_handle);
        body_mass = py::array_t<mjtNum>(size_t(m_->nbody*1), m_->body_mass, buffer_handle);
        body_subtreemass = py::array_t<mjtNum>(size_t(m_->nbody*1), m_->body_subtreemass, buffer_handle);
        body_inertia = py::array_t<mjtNum>(size_t(m_->nbody*3), m_->body_inertia, buffer_handle);
        body_invweight0 = py::array_t<mjtNum>(size_t(m_->nbody*2), m_->body_invweight0, buffer_handle);
        body_user = py::array_t<mjtNum>(size_t(m_->nbody*m_->nuser_body), m_->body_user, buffer_handle);
    // joints    
        jnt_type = py::array_t<int>(size_t(m_->njnt*1), m_->jnt_type, buffer_handle);
        jnt_qposadr = py::array_t<int>(size_t(m_->njnt*1), m_->jnt_qposadr, buffer_handle);
        jnt_dofadr = py::array_t<int>(size_t(m_->njnt*1), m_->jnt_dofadr, buffer_handle);
        jnt_bodyid = py::array_t<int>(size_t(m_->njnt*1), m_->jnt_bodyid, buffer_handle);
        jnt_group = py::array_t<int>(size_t(m_->njnt*1), m_->jnt_group, buffer_handle);
        jnt_limited = py::array_t<mjtByte>(size_t(m_->njnt*1), m_->jnt_limited, buffer_handle);
        jnt_solref = py::array_t<mjtNum>(size_t(m_->njnt*mjNREF), m_->jnt_solref, buffer_handle);
        jnt_solimp = py::array_t<mjtNum>(size_t(m_->njnt*mjNIMP), m_->jnt_solimp, buffer_handle);
        jnt_pos = py::array_t<mjtNum>(size_t(m_->njnt*3), m_->jnt_pos, buffer_handle);
        jnt_axis = py::array_t<mjtNum>(size_t(m_->njnt*3), m_->jnt_axis, buffer_handle);
        jnt_stiffness = py::array_t<mjtNum>(size_t(m_->njnt*1), m_->jnt_stiffness, buffer_handle);
        jnt_range = py::array_t<mjtNum>(size_t(m_->njnt*2), m_->jnt_range, buffer_handle);
        jnt_margin = py::array_t<mjtNum>(size_t(m_->njnt*1), m_->jnt_margin, buffer_handle);
        jnt_user = py::array_t<mjtNum>(size_t(m_->njnt*m_->nuser_jnt), m_->jnt_user, buffer_handle);

// dofs
        dof_bodyid = py::array_t<int>(size_t(m_->nv*1), m_->dof_bodyid, buffer_handle);
        dof_jntid = py::array_t<int>(size_t(m_->nv*1), m_->dof_jntid, buffer_handle);
        dof_parentid = py::array_t<int>(size_t(m_->nv*1), m_->dof_parentid, buffer_handle);
        dof_Madr = py::array_t<int>(size_t(m_->nv*1), m_->dof_Madr, buffer_handle);
        dof_simplenum = py::array_t<int>(size_t(m_->nv*1), m_->dof_simplenum, buffer_handle);

        dof_solref = py::array_t<mjtNum>(size_t(m_->nv*mjNREF), m_->dof_solref, buffer_handle);
        dof_solimp = py::array_t<mjtNum>(size_t(m_->nv*mjNIMP), m_->dof_solimp, buffer_handle);

        dof_frictionloss = py::array_t<mjtNum>(size_t(m_->nv*1), m_->dof_frictionloss, buffer_handle);
        dof_armature = py::array_t<mjtNum>(size_t(m_->nv*1), m_->dof_armature, buffer_handle);
        dof_damping = py::array_t<mjtNum>(size_t(m_->nv*1), m_->dof_damping, buffer_handle);
        dof_invweight0 = py::array_t<mjtNum>(size_t(m_->nv*1), m_->dof_invweight0, buffer_handle);
        dof_M0 = py::array_t<mjtNum>(size_t(m_->nv*1), m_->dof_M0, buffer_handle);
// geoms
         geom_type = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_type, buffer_handle);
         geom_contype = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_contype, buffer_handle);
         geom_conaffinity = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_conaffinity, buffer_handle);
         geom_condim = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_condim, buffer_handle);
         geom_bodyid = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_bodyid, buffer_handle);
         geom_dataid = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_dataid, buffer_handle);
         geom_matid = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_matid, buffer_handle);
         geom_group = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_group, buffer_handle);
         geom_priority = py::array_t<int>(size_t(m_->ngeom*1), m_->geom_priority, buffer_handle);

         geom_sameframe = py::array_t<mjtByte>(size_t(m_->ngeom*1), m_->geom_sameframe, buffer_handle);

         geom_solmix = py::array_t<mjtNum>(size_t(m_->ngeom*1), m_->geom_solmix, buffer_handle);
         geom_solref = py::array_t<mjtNum>(size_t(m_->ngeom*mjNREF), m_->geom_solref, buffer_handle);
         geom_solimp = py::array_t<mjtNum>(size_t(m_->ngeom*mjNIMP), m_->geom_solimp, buffer_handle);
         geom_size = py::array_t<mjtNum>(size_t(m_->ngeom*3), m_->geom_size, buffer_handle);
         geom_rbound = py::array_t<mjtNum>(size_t(m_->ngeom*1), m_->geom_rbound, buffer_handle);
         geom_pos = py::array_t<mjtNum>(size_t(m_->ngeom*3), m_->geom_pos, buffer_handle);
         geom_quat = py::array_t<mjtNum>(size_t(m_->ngeom*4), m_->geom_quat, buffer_handle);
         geom_friction = py::array_t<mjtNum>(size_t(m_->ngeom*3), m_->geom_friction, buffer_handle);
         geom_margin = py::array_t<mjtNum>(size_t(m_->ngeom*1), m_->geom_margin, buffer_handle);
         geom_gap = py::array_t<mjtNum>(size_t(m_->ngeom*1), m_->geom_gap, buffer_handle);
         geom_user = py::array_t<mjtNum>(size_t(m_->ngeom*m_->nuser_geom), m_->geom_user, buffer_handle);
         geom_rgba = py::array_t<float>(size_t(m_->ngeom*4), m_->geom_rgba, buffer_handle);
// sites
         site_type = py::array_t<int>(size_t(m_->nsite*1), m_->site_type, buffer_handle);
         site_bodyid = py::array_t<int>(size_t(m_->nsite*1), m_->site_bodyid, buffer_handle);
         site_matid = py::array_t<int>(size_t(m_->nsite*1), m_->site_matid, buffer_handle);
         site_group = py::array_t<int>(size_t(m_->nsite*1), m_->site_group, buffer_handle);
         site_sameframe = py::array_t<mjtByte>(size_t(m_->nsite*1), m_->site_sameframe, buffer_handle);
         site_size = py::array_t<mjtNum>(size_t(m_->nsite*3), m_->site_size, buffer_handle);
         site_pos = py::array_t<mjtNum>(size_t(m_->nsite*3), m_->site_pos, buffer_handle);
         site_quat = py::array_t<mjtNum>(size_t(m_->nsite*4), m_->site_quat, buffer_handle);
         site_user = py::array_t<mjtNum>(size_t(m_->nsite*nuser_site), m_->site_user, buffer_handle);
         site_rgba = py::array_t<float>(size_t(m_->nsite*4), m_->site_rgba, buffer_handle);

#if 0
    // cameras
    py::array_t<int>      cam_mode;             // camera tracking mode (mjtCamLight)       (ncam x 1)
    py::array_t<int>      cam_bodyid;           // id of camera's body                      (ncam x 1)
    py::array_t<int>      cam_targetbodyid;     // id of targeted body; -1: none            (ncam x 1)
    py::array_t<mjtNum>   cam_pos;              // position rel. to body frame              (ncam x 3)
    py::array_t<mjtNum>   cam_quat;             // orientation rel. to body frame           (ncam x 4)
    py::array_t<mjtNum>   cam_poscom0;          // global position rel. to sub-com in qpos0 (ncam x 3)
    py::array_t<mjtNum>   cam_pos0;             // global position rel. to body in qpos0    (ncam x 3)
    py::array_t<mjtNum>   cam_mat0;             // global orientation in qpos0              (ncam x 9)
    py::array_t<mjtNum>   cam_fovy;             // y-field of view (deg)                    (ncam x 1)
    py::array_t<mjtNum>   cam_ipd;              // inter-pupilary distance                  (ncam x 1)
    py::array_t<mjtNum>   cam_user;             // user data                                (ncam x nuser_cam)

    // lights
    py::array_t<int>      light_mode;           // light tracking mode (mjtCamLight)        (nlight x 1)
    py::array_t<int>      light_bodyid;         // id of light's body                       (nlight x 1)
    py::array_t<int>      light_targetbodyid;   // id of targeted body; -1: none            (nlight x 1)
    py::array_t<mjtByte>  light_directional;    // directional light                        (nlight x 1)
    py::array_t<mjtByte>  light_castshadow;     // does light cast shadows                  (nlight x 1)
    py::array_t<mjtByte>  light_active;         // is light on                              (nlight x 1)
    py::array_t<mjtNum>   light_pos;            // position rel. to body frame              (nlight x 3)
    py::array_t<mjtNum>   light_dir;            // direction rel. to body frame             (nlight x 3)
    py::array_t<mjtNum>   light_poscom0;        // global position rel. to sub-com in qpos0 (nlight x 3)
    py::array_t<mjtNum>   light_pos0;           // global position rel. to body in qpos0    (nlight x 3)
    py::array_t<mjtNum>   light_dir0;           // global direction in qpos0                (nlight x 3)
    py::array_t<float>    light_attenuation;    // OpenGL attenuation (quadratic model)     (nlight x 3)
    py::array_t<float>    light_cutoff;         // OpenGL cutoff                            (nlight x 1)
    py::array_t<float>    light_exponent;       // OpenGL exponent                          (nlight x 1)
    py::array_t<float>    light_ambient;        // ambient rgb (alpha=1)                    (nlight x 3)
    py::array_t<float>    light_diffuse;        // diffuse rgb (alpha=1)                    (nlight x 3)
    py::array_t<float>    light_specular;       // specular rgb (alpha=1)                   (nlight x 3)

    // meshes
    py::array_t<int>      mesh_vertadr;         // first vertex address                     (nmesh x 1)
    py::array_t<int>      mesh_vertnum;         // number of vertices                       (nmesh x 1)
    py::array_t<int>      mesh_texcoordadr;     // texcoord data address; -1: no texcoord   (nmesh x 1)
    py::array_t<int>      mesh_faceadr;         // first face address                       (nmesh x 1)
    py::array_t<int>      mesh_facenum;         // number of faces                          (nmesh x 1)
    py::array_t<int>      mesh_graphadr;        // graph data address; -1: no graph         (nmesh x 1)
    py::array_t<float>    mesh_vert;            // vertex positions for all meshe           (nmeshvert x 3)
    py::array_t<float>    mesh_normal;          // vertex normals for all meshes            (nmeshvert x 3)
    py::array_t<float>    mesh_texcoord;        // vertex texcoords for all meshes          (nmeshtexvert x 2)
    py::array_t<int>      mesh_face;            // triangle face data                       (nmeshface x 3)
    py::array_t<int>      mesh_graph;           // convex graph data                        (nmeshgraph x 1)

    // skins
    py::array_t<int>      skin_matid;           // skin material id; -1: none               (nskin x 1)
    py::array_t<float>    skin_rgba;            // skin rgba                                (nskin x 4)
    py::array_t<float>    skin_inflate;         // inflate skin in normal direction         (nskin x 1)
    py::array_t<int>      skin_vertadr;         // first vertex address                     (nskin x 1)
    py::array_t<int>      skin_vertnum;         // number of vertices                       (nskin x 1)
    py::array_t<int>      skin_texcoordadr;     // texcoord data address; -1: no texcoord   (nskin x 1)
    py::array_t<int>      skin_faceadr;         // first face address                       (nskin x 1)
    py::array_t<int>      skin_facenum;         // number of faces                          (nskin x 1)
    py::array_t<int>      skin_boneadr;         // first bone in skin                       (nskin x 1)
    py::array_t<int>      skin_bonenum;         // number of bones in skin                  (nskin x 1)
    py::array_t<float>    skin_vert;            // vertex positions for all skin meshes     (nskinvert x 3)
    py::array_t<float>    skin_texcoord;        // vertex texcoords for all skin meshes     (nskintexvert x 2)
    py::array_t<int>      skin_face;            // triangle faces for all skin meshes       (nskinface x 3)
    py::array_t<int>      skin_bonevertadr;     // first vertex in each bone                (nskinbone x 1)
    py::array_t<int>      skin_bonevertnum;     // number of vertices in each bone          (nskinbone x 1)
    py::array_t<float>    skin_bonebindpos;     // bind pos of each bone                    (nskinbone x 3)
    py::array_t<float>    skin_bonebindquat;    // bind quat of each bone                   (nskinbone x 4)
    py::array_t<int>      skin_bonebodyid;      // body id of each bone                     (nskinbone x 1)
    py::array_t<int>      skin_bonevertid;      // mesh ids of vertices in each bone        (nskinbonevert x 1)
    py::array_t<float>    skin_bonevertweight;  // weights of vertices in each bone         (nskinbonevert x 1)

    // height fields
    py::array_t<mjtNum>   hfield_size;          // (x, y, z_top, z_bottom)                  (nhfield x 4)
    py::array_t<int>      hfield_nrow;          // number of rows in grid                   (nhfield x 1)
    py::array_t<int>      hfield_ncol;          // number of columns in grid                (nhfield x 1)
    py::array_t<int>      hfield_adr;           // address in hfield_data                   (nhfield x 1)
    py::array_t<float>    hfield_data;          // elevation data                           (nhfielddata x 1)

    // textures
    py::array_t<int>      tex_type;             // texture type (mjtTexture)                (ntex x 1)
    py::array_t<int>      tex_height;           // number of rows in texture image          (ntex x 1)
    py::array_t<int>      tex_width;            // number of columns in texture image       (ntex x 1)
    py::array_t<int>      tex_adr;              // address in rgb                           (ntex x 1)
    py::array_t<mjtByte>  tex_rgb;              // rgb (alpha = 1)                          (ntexdata x 1)

    // materials
    py::array_t<int>      mat_texid;            // texture id; -1: none                     (nmat x 1)
    py::array_t<mjtByte>  mat_texuniform;       // make texture cube uniform                (nmat x 1)
    py::array_t<float>    mat_texrepeat;        // texture repetition for 2d mapping        (nmat x 2)
    py::array_t<float>    mat_emission;         // emission (x rgb)                         (nmat x 1)
    py::array_t<float>    mat_specular;         // specular (x white)                       (nmat x 1)
    py::array_t<float>    mat_shininess;        // shininess coef                           (nmat x 1)
    py::array_t<float>    mat_reflectance;      // reflectance (0: disable)                 (nmat x 1)
    py::array_t<float>    mat_rgba;             // rgba                                     (nmat x 4)

    // predefined geom pairs for collision detection; has precedence over exclude
    py::array_t<int>      pair_dim;             // contact dimensionality                   (npair x 1)
    py::array_t<int>      pair_geom1;           // id of geom1                              (npair x 1)
    py::array_t<int>      pair_geom2;           // id of geom2                              (npair x 1)
    py::array_t<int>      pair_signature;       // (body1+1)<<16 + body2+1                  (npair x 1)
    py::array_t<mjtNum>   pair_solref;          // constraint solver reference: contact     (npair x mjNREF)
    py::array_t<mjtNum>   pair_solimp;          // constraint solver impedance: contact     (npair x mjNIMP)
    py::array_t<mjtNum>   pair_margin;          // detect contact if dist<margin            (npair x 1)
    py::array_t<mjtNum>   pair_gap;             // include in solver if dist<margin-gap     (npair x 1)
    py::array_t<mjtNum>   pair_friction;        // tangent1, 2, spin, roll1, 2              (npair x 5)

    // excluded body pairs for collision detection
    py::array_t<int>      exclude_signature;    // (body1+1)<<16 + body2+1                  (nexclude x 1)

    // equality constraints
    py::array_t<int>      eq_type;              // constraint type (mjtEq)                  (neq x 1)
    py::array_t<int>      eq_obj1id;            // id of object 1                           (neq x 1)
    py::array_t<int>      eq_obj2id;            // id of object 2                           (neq x 1)
    py::array_t<mjtByte>  eq_active;            // enable/disable constraint                (neq x 1)
    py::array_t<mjtNum>   eq_solref;            // constraint solver reference              (neq x mjNREF)
    py::array_t<mjtNum>   eq_solimp;            // constraint solver impedance              (neq x mjNIMP)
    py::array_t<mjtNum>   eq_data;              // numeric data for constraint              (neq x mjNEQDATA)

    // tendons
    py::array_t<int>      tendon_adr;           // address of first object in tendon's path (ntendon x 1)
    py::array_t<int>      tendon_num;           // number of objects in tendon's path       (ntendon x 1)
    py::array_t<int>      tendon_matid;         // material id for rendering                (ntendon x 1)
    py::array_t<int>      tendon_group;         // group for visibility                     (ntendon x 1)
    py::array_t<mjtByte>  tendon_limited;       // does tendon have length limits           (ntendon x 1)
    py::array_t<mjtNum>   tendon_width;         // width for rendering                      (ntendon x 1)
    py::array_t<mjtNum>   tendon_solref_lim;    // constraint solver reference: limit       (ntendon x mjNREF)
    py::array_t<mjtNum>   tendon_solimp_lim;    // constraint solver impedance: limit       (ntendon x mjNIMP)
    py::array_t<mjtNum>   tendon_solref_fri;    // constraint solver reference: friction    (ntendon x mjNREF)
    py::array_t<mjtNum>   tendon_solimp_fri;    // constraint solver impedance: friction    (ntendon x mjNIMP)
    py::array_t<mjtNum>   tendon_range;         // tendon length limits                     (ntendon x 2)
    py::array_t<mjtNum>   tendon_margin;        // min distance for limit detection         (ntendon x 1)
    py::array_t<mjtNum>   tendon_stiffness;     // stiffness coefficient                    (ntendon x 1)
    py::array_t<mjtNum>   tendon_damping;       // damping coefficient                      (ntendon x 1)
    py::array_t<mjtNum>   tendon_frictionloss;  // loss due to friction                     (ntendon x 1)
    py::array_t<mjtNum>   tendon_lengthspring;  // tendon length in qpos_spring             (ntendon x 1)
    py::array_t<mjtNum>   tendon_length0;       // tendon length in qpos0                   (ntendon x 1)
    py::array_t<mjtNum>   tendon_invweight0;    // inv. weight in qpos0                     (ntendon x 1)
    py::array_t<mjtNum>   tendon_user;          // user data                                (ntendon x nuser_tendon)
    py::array_t<float>    tendon_rgba;          // rgba when material is omitted            (ntendon x 4)

    // list of all wrap objects in tendon paths
    py::array_t<int>      wrap_type;            // wrap object type (mjtWrap)               (nwrap x 1)
    py::array_t<int>      wrap_objid;           // object id: geom, site, joint             (nwrap x 1)
    py::array_t<mjtNum>   wrap_prm;             // divisor, joint coef, or site id          (nwrap x 1)

    // actuators
    py::array_t<int>      actuator_trntype;     // transmission type (mjtTrn)               (nu x 1)
    py::array_t<int>      actuator_dyntype;     // dynamics type (mjtDyn)                   (nu x 1)
    py::array_t<int>      actuator_gaintype;    // gain type (mjtGain)                      (nu x 1)
    py::array_t<int>      actuator_biastype;    // bias type (mjtBias)                      (nu x 1)
    py::array_t<int>      actuator_trnid;       // transmission id: joint, tendon, site     (nu x 2)
    py::array_t<int>      actuator_group;       // group for visibility                     (nu x 1)
    py::array_t<mjtByte>  actuator_ctrllimited; // is control limited                       (nu x 1)
    py::array_t<mjtByte>  actuator_forcelimited;// is force limited                         (nu x 1)
    py::array_t<mjtNum>   actuator_dynprm;      // dynamics parameters                      (nu x mjNDYN)
    py::array_t<mjtNum>   actuator_gainprm;     // gain parameters                          (nu x mjNGAIN)
    py::array_t<mjtNum>   actuator_biasprm;     // bias parameters                          (nu x mjNBIAS)
    py::array_t<mjtNum>   actuator_ctrlrange;   // range of controls                        (nu x 2)
    py::array_t<mjtNum>   actuator_forcerange;  // range of forces                          (nu x 2)
    py::array_t<mjtNum>   actuator_gear;        // scale length and transmitted force       (nu x 6)
    py::array_t<mjtNum>   actuator_cranklength; // crank length for slider-crank            (nu x 1)
    py::array_t<mjtNum>   actuator_acc0;        // acceleration from unit force in qpos0    (nu x 1)
    py::array_t<mjtNum>   actuator_length0;     // actuator length in qpos0                 (nu x 1)
    py::array_t<mjtNum>   actuator_lengthrange; // feasible actuator length range           (nu x 2)
    py::array_t<mjtNum>   actuator_user;        // user data                                (nu x nuser_actuator)

    // sensors
    py::array_t<int>      sensor_type;          // sensor type (mjtSensor)                  (nsensor x 1)
    py::array_t<int>      sensor_datatype;      // numeric data type (mjtDataType)          (nsensor x 1)
    py::array_t<int>      sensor_needstage;     // required compute stage (mjtStage)        (nsensor x 1)
    py::array_t<int>      sensor_objtype;       // type of sensorized object (mjtObj)       (nsensor x 1)
    py::array_t<int>      sensor_objid;         // id of sensorized object                  (nsensor x 1)
    py::array_t<int>      sensor_dim;           // number of scalar outputs                 (nsensor x 1)
    py::array_t<int>      sensor_adr;           // address in sensor array                  (nsensor x 1)
    py::array_t<mjtNum>   sensor_cutoff;        // cutoff for real and positive; 0: ignore  (nsensor x 1)
    py::array_t<mjtNum>   sensor_noise;         // noise standard deviation                 (nsensor x 1)
    py::array_t<mjtNum>   sensor_user;          // user data                                (nsensor x nuser_sensor)

    // custom numeric fields
    py::array_t<int>      numeric_adr;          // address of field in numeric_data         (nnumeric x 1)
    py::array_t<int>      numeric_size;         // size of numeric field                    (nnumeric x 1)
    py::array_t<mjtNum>   numeric_data;         // array of all numeric fields              (nnumericdata x 1)

    // custom text fields
    py::array_t<int>      text_adr;             // address of text in text_data             (ntext x 1)
    py::array_t<int>      text_size;            // size of text field (strlen+1)            (ntext x 1)
    py::array_t<char>     text_data;            // array of all text fields (0-terminated)  (ntextdata x 1)

    // custom tuple fields
    py::array_t<int>      tuple_adr;            // address of text in text_data             (ntuple x 1)
    py::array_t<int>      tuple_size;           // number of objects in tuple               (ntuple x 1)
    py::array_t<int>      tuple_objtype;        // array of object types in all tuples      (ntupledata x 1)
    py::array_t<int>      tuple_objid;          // array of object ids in all tuples        (ntupledata x 1)
    py::array_t<mjtNum>   tuple_objprm;         // array of object params in all tuples     (ntupledata x 1)

    // keyframes
    py::array_t<mjtNum>   key_time;             // key time                                 (nkey x 1)
    py::array_t<mjtNum>   key_qpos;             // key position                             (nkey x nq)
    py::array_t<mjtNum>   key_qvel;             // key velocity                             (nkey x nv)
    py::array_t<mjtNum>   key_act;              // key activation                           (nkey x na)
    py::array_t<mjtNum>   key_mpos;             // key mocap position                       (nkey x 3*nmocap)
    py::array_t<mjtNum>   key_mquat;            // key mocap quaternion                     (nkey x 4*nmocap)
#endif

    // names
    
    name_bodyadr = py::array_t<int>(size_t(m_->nbody*1), m_->name_bodyadr, buffer_handle);
    name_jntadr = py::array_t<int>(size_t(m_->njnt*1), m_->name_jntadr, buffer_handle);
    name_geomadr = py::array_t<int>(size_t(m_->ngeom*1), m_->name_geomadr, buffer_handle);
    name_siteadr = py::array_t<int>(size_t(m_->nsite*1), m_->name_siteadr, buffer_handle);
    name_camadr = py::array_t<int>(size_t(m_->ncam*1), m_->name_camadr, buffer_handle);
    name_lightadr = py::array_t<int>(size_t(m_->nlight*1), m_->name_lightadr, buffer_handle);
    name_meshadr = py::array_t<int>(size_t(m_->nmesh*1), m_->name_meshadr, buffer_handle);
    name_skinadr = py::array_t<int>(size_t(m_->nskin*1), m_->name_skinadr, buffer_handle);
    name_hfieldadr = py::array_t<int>(size_t(m_->nhfield*1), m_->name_hfieldadr, buffer_handle);
    name_texadr = py::array_t<int>(size_t(m_->ntex*1), m_->name_texadr, buffer_handle);
    name_matadr = py::array_t<int>(size_t(m_->nmat*1), m_->name_matadr, buffer_handle);
    name_pairadr = py::array_t<int>(size_t(m_->npair*1), m_->name_pairadr, buffer_handle);
    name_excludeadr = py::array_t<int>(size_t(m_->nexclude*1), m_->name_excludeadr, buffer_handle);
    name_eqadr = py::array_t<int>(size_t(m_->neq*1), m_->name_eqadr, buffer_handle);
    name_tendonadr = py::array_t<int>(size_t(m_->ntendon*1), m_->name_tendonadr, buffer_handle);
    name_actuatoradr = py::array_t<int>(size_t(m_->nbody*1), m_->name_actuatoradr, buffer_handle);
    name_sensoradr = py::array_t<int>(size_t(m_->nsensor*1), m_->name_sensoradr, buffer_handle);
    name_numericadr = py::array_t<int>(size_t(m_->nnumeric*1), m_->name_numericadr, buffer_handle);
    name_textadr = py::array_t<int>(size_t(m_->ntext*1), m_->name_textadr, buffer_handle);
    name_tupleadr = py::array_t<int>(size_t(m_->ntuple*1), m_->name_tupleadr, buffer_handle);
    name_keyadr = py::array_t<int>(size_t(m_->nkey*1), m_->name_keyadr, buffer_handle);
    
    names = py::array_t<char>(size_t(m_->nnames*1), m_->names, buffer_handle);

    }

    explicit operator const mjModel*() const { 
        return m_; 
    }

    virtual ~PyMjModel()
    {
        
    }
};


void py_mj_deleteModel(PyMjModel* m)
{
    mj_deleteModel(m->m_);
    delete m;
}


struct PyMjContact                   // result of collision detection functions
{
    // contact parameters set by geom-specific collision detector
    py::array_t<mjtNum> dist;                    // distance between nearest points; neg: penetration
    py::array_t<mjtNum> pos;                  // position of contact point: midpoint between geoms
    py::array_t<mjtNum> frame;                // normal is in [0-2]

    // contact parameters set by mj_collideGeoms
    py::array_t<mjtNum> includemargin;           // include if dist<includemargin=margin-gap
    py::array_t<mjtNum> friction;             // tangent1, 2, spin, roll1, 2
    py::array_t<mjtNum> solref;          // constraint solver reference
    py::array_t<mjtNum> solimp;          // constraint solver impedance

    // internal storage used by solver
    py::array_t<mjtNum> mu;                      // friction of regularized cone, set by mj_makeConstraint
    py::array_t<mjtNum> H;                   // cone Hessian, set by mj_updateConstraint

    // contact descriptors set by mj_collideGeoms
    py::array_t<int> dim;                        // contact space dimensionality: 1, 3, 4 or 6
    py::array_t<int> geom1;                      // id of geom 1
    py::array_t<int> geom2;                      // id of geom 2

    // flag set by mj_fuseContact or mj_instantianteEquality
    py::array_t<int> exclude;                    // 0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs

    // address computed by mj_instantiateContact
    py::array_t<int> efc_address;                // address in efc; -1: not included, -2-i: distance constraint i
    py::capsule buffer_handle;

    PyMjContact(mjContact& ct)
        :buffer_handle(py::capsule ([](){}))
    {
        
        dist = py::array_t<mjtNum>(size_t(1), &ct.dist, buffer_handle);
        pos = py::array_t<mjtNum>(size_t(3), ct.pos, buffer_handle);
        frame = py::array_t<mjtNum>(size_t(9), ct.frame, buffer_handle);
        includemargin = py::array_t<mjtNum>(size_t(1), &ct.includemargin, buffer_handle);
        friction = py::array_t<mjtNum>(size_t(9), ct.friction, buffer_handle);
        solref = py::array_t<mjtNum>(size_t(mjNREF), ct.solref, buffer_handle);
        solimp = py::array_t<mjtNum>(size_t(mjNIMP), ct.solimp, buffer_handle);
        mu = py::array_t<mjtNum>(size_t(1), &ct.mu, buffer_handle);
        H = py::array_t<mjtNum>(size_t(36), ct.H, buffer_handle);
        dim = py::array_t<int>(size_t(1), &ct.dim, buffer_handle);
        geom1 = py::array_t<int>(size_t(1), &ct.geom1, buffer_handle);
        geom2 = py::array_t<int>(size_t(1), &ct.geom2, buffer_handle);
        exclude = py::array_t<int>(size_t(1), &ct.exclude, buffer_handle);
        efc_address = py::array_t<int>(size_t(1), &ct.efc_address, buffer_handle);
    }
};


struct PyMjData 
{
    mjData* d_;
    const mjModel* m_;

    py::array_t<int> solver_iter;                // number of solver iterations
    py::array_t<int> solver_nnz;                 // number of non-zeros in Hessian or efc_AR
    //mjtNum solver_fwdinv[2];        // forward-inverse comparison: qfrc, efc

    // variable sizes
    py::array_t<int> ne;                         // number of equality constraints
    py::array_t<int> nf;                         // number of friction constraints
    py::array_t<int> nefc;                       // number of constraints
    py::array_t<int> ncon;                       // number of detected contacts

    // global properties
    py::array_t<mjtNum> time_;                    // simulation time
     
// state
    py::array_t<mjtNum> qpos;                 // position                                 (nq x 1)
    py::array_t<mjtNum> qvel;                 // velocity                                 (nv x 1)
    py::array_t<mjtNum> act;                  // actuator activation                      (na x 1)
    py::array_t<mjtNum> qacc_warmstart;       // acceleration used for warmstart          (nv x 1)

    // control
    py::array_t<mjtNum> ctrl;                 // control                                  (nu x 1)
    py::array_t<mjtNum> qfrc_applied;         // applied generalized force                (nv x 1)
    py::array_t<mjtNum> xfrc_applied;         // applied Cartesian force/torque           (nbody x 6)

// dynamics
    py::array_t<mjtNum> qacc;                 // acceleration                             (nv x 1)
    py::array_t<mjtNum> act_dot;              // time-derivative of actuator activation   (na x 1)


    // computed by mj_fwdPosition/mj_kinematics
    py::array_t<mjtNum>   xpos;                 // Cartesian position of body frame         (nbody x 3)
    py::array_t<mjtNum>   xquat;                // Cartesian orientation of body frame      (nbody x 4)
    py::array_t<mjtNum>   xmat;                 // Cartesian orientation of body frame      (nbody x 9)
    py::array_t<mjtNum>   xipos;                // Cartesian position of body com           (nbody x 3)
    py::array_t<mjtNum>   ximat;                // Cartesian orientation of body inertia    (nbody x 9)
    py::array_t<mjtNum>   xanchor;              // Cartesian position of joint anchor       (njnt x 3)
    py::array_t<mjtNum>   xaxis;                // Cartesian joint axis                     (njnt x 3)
    py::array_t<mjtNum>   geom_xpos;            // Cartesian geom position                  (ngeom x 3)
    py::array_t<mjtNum>   geom_xmat;            // Cartesian geom orientation               (ngeom x 9)
    py::array_t<mjtNum>   site_xpos;            // Cartesian site position                  (nsite x 3)
    py::array_t<mjtNum>   site_xmat;            // Cartesian site orientation               (nsite x 9)
    py::array_t<mjtNum>   cam_xpos;             // Cartesian camera position                (ncam x 3)
    py::array_t<mjtNum>   cam_xmat;             // Cartesian camera orientation             (ncam x 9)
    py::array_t<mjtNum>   light_xpos;           // Cartesian light position                 (nlight x 3)
    py::array_t<mjtNum>   light_xdir;           // Cartesian light direction                (nlight x 3)

    // computed by mj_fwdPosition/mj_comPos
    py::array_t<mjtNum>   subtree_com;          // center of mass of each subtree           (nbody x 3)
    py::array_t<mjtNum>   cdof;                 // com-based motion axis of each dof        (nv x 6)
    py::array_t<mjtNum>   cinert;               // com-based body inertia and mass          (nbody x 10)

       // computed by mj_fwdPosition/mj_transmission
    py::array_t<mjtNum>   actuator_length;      // actuator lengths                         (nu x 1)
    py::array_t<mjtNum>   actuator_moment;      // actuator moments                         (nu x nv)

     // computed by mj_fwdPosition/mj_crb
    py::array_t<mjtNum>   crb;                  // com-based composite inertia and mass     (nbody x 10)
    py::array_t<mjtNum>   qM;                   // total inertia                            (nM x 1)

    // computed by mj_fwdPosition/mj_factorM
    py::array_t<mjtNum>   qLD;                  // L'*D*L factorization of M                (nM x 1)
    py::array_t<mjtNum>   qLDiagInv;            // 1/diag(D)                                (nv x 1)
    py::array_t<mjtNum>   qLDiagSqrtInv;        // 1/sqrt(diag(D))                          (nv x 1)

    // computed by mj_fwdPosition/mj_collision
    //py::array_t<mjContact> contact;             // list of all detected contacts            (nconmax x 1)


    py::capsule buffer_handle;

    PyMjData (mjData* d, const mjModel* m)
        :d_(d),
        m_(m),
        buffer_handle(py::capsule([](){}))
    {
        
        //map constants to numpy arrays too, so they are always up-to-date
        solver_iter = py::array_t<int>(1, &d_->solver_iter, buffer_handle);
        solver_nnz = py::array_t<int>(1, &d_->solver_nnz, buffer_handle);
        ne = py::array_t<int>(1, &d_->ne, buffer_handle);
        nf = py::array_t<int>(1, &d_->nf, buffer_handle);
        nefc = py::array_t<int>(1, &d_->nefc, buffer_handle);
        ncon = py::array_t<int>(1, &d_->ncon, buffer_handle);
        time_ = py::array_t<mjtNum>(1, &(d_->time), buffer_handle);
        
        //c-arrays are mapped to numpy arrays without copy
        qpos =      py::array_t<mjtNum>(m_->nq, d_->qpos,buffer_handle);
        qvel = py::array_t<mjtNum>(size_t(m_->nv), d_->qvel, buffer_handle);
        act =  py::array_t<mjtNum>(m_->na, d_->act,buffer_handle);
        qacc_warmstart =  py::array_t<mjtNum>(m_->nv, d_->qacc_warmstart,buffer_handle);
    
        // control
        ctrl =  py::array_t<mjtNum>(m_->nu, d_->ctrl,buffer_handle);
        qfrc_applied =  py::array_t<mjtNum>(m_->nv, d_->qfrc_applied,buffer_handle);
        xfrc_applied =  py::array_t<mjtNum>(m_->nbody*6, d_->xfrc_applied,buffer_handle);
        
        // dynamics
        qacc = py::array_t<mjtNum>(size_t(m_->nv), d_->qacc, buffer_handle);
        act_dot = py::array_t<mjtNum>(size_t(m_->na), d_->act_dot, buffer_handle);
        
        // computed by mj_fwdPosition/mj_kinematics

        xpos = py::array_t<mjtNum>(size_t(m_->nbody*3), d_->xpos, buffer_handle);
        xquat = py::array_t<mjtNum>(size_t(m_->nbody*4), d_->xquat, buffer_handle);
        xmat = py::array_t<mjtNum>(size_t(m_->nbody*9), d_->xmat, buffer_handle);
        xipos = py::array_t<mjtNum>(size_t(m_->nbody*3), d_->xipos, buffer_handle);
        ximat = py::array_t<mjtNum>(size_t(m_->nbody*9), d_->ximat, buffer_handle);
        xanchor = py::array_t<mjtNum>(size_t(m_->njnt*3), d_->xanchor, buffer_handle);
        xaxis = py::array_t<mjtNum>(size_t(m_->njnt*3), d_->xaxis, buffer_handle);
        geom_xpos = py::array_t<mjtNum>(size_t(m_->ngeom*3), d_->geom_xpos, buffer_handle);
        geom_xmat = py::array_t<mjtNum>(size_t(m_->ngeom*9), d_->geom_xmat, buffer_handle);
        site_xpos = py::array_t<mjtNum>(size_t(m_->nsite*3), d_->site_xpos, buffer_handle);
        site_xmat = py::array_t<mjtNum>(size_t(m_->nsite*9), d_->site_xmat, buffer_handle);

#if 0
    py::array_t<mjtNum>   cam_xpos;             // Cartesian camera position                (ncam x 3)
    py::array_t<mjtNum>   cam_xmat;             // Cartesian camera orientation             (ncam x 9)
    py::array_t<mjtNum>   light_xpos;           // Cartesian light position                 (nlight x 3)
    py::array_t<mjtNum>   light_xdir;           // Cartesian light direction                (nlight x 3)
#endif

        // computed by mj_fwdPosition/mj_comPos
        subtree_com = py::array_t<mjtNum>(size_t(m_->nbody*3), d_->subtree_com, buffer_handle);
        cdof = py::array_t<mjtNum>(size_t(m_->nv*6), d_->cdof, buffer_handle);
        cinert = py::array_t<mjtNum>(size_t(m_->nbody*10), d_->cinert, buffer_handle);

        // computed by mj_fwdPosition/mj_transmission
        actuator_length = py::array_t<mjtNum>(size_t(m_->nu*1), d_->actuator_length, buffer_handle);
        actuator_moment = py::array_t<mjtNum>(size_t(m_->nu*m_->nv), d_->actuator_moment, buffer_handle);
        
     // computed by mj_fwdPosition/mj_crb
        crb = py::array_t<mjtNum>(size_t(m_->nbody*10), d_->crb, buffer_handle);
        qM = py::array_t<mjtNum>(size_t(m_->nM*1), d_->qM, buffer_handle);
    
    // computed by mj_fwdPosition/mj_factorM
        qLD = py::array_t<mjtNum>(size_t(m_->nM*1), d_->qLD, buffer_handle);
        qLDiagInv = py::array_t<mjtNum>(size_t(m_->nv*1), d_->qLDiagInv, buffer_handle);
        qLDiagSqrtInv = py::array_t<mjtNum>(size_t(m_->nv*1), d_->qLDiagSqrtInv, buffer_handle);
    
        // computed by mj_fwdPosition/mj_collision
        //contact = py::array_t<mjContact>(size_t(m_->nconmax*1), d_->contact, buffer_handle);

    }


    PyMjContact get_contact(int index)
    {
        if (index<0 || index>=d_->ncon)
        {
            printf("ERROR: index for get_contact needs to be in range [0..data.ncon]\n");
            index=0;
        }
        return PyMjContact(d_->contact[index]);
    }
    
    
    void add_elem(float value)
	{
        d_->qpos[0] += value;
    }
    

    virtual ~PyMjData()
    {
        
    }
};

PyMjData* py_mj_makeData(const PyMjModel* m)
{
    mjData* d = mj_makeData(m->m_);
    
    return new PyMjData(d, m->m_);
}

void py_mj_deleteData(PyMjData* d)
{
    mj_deleteData(d->d_);
    delete d;
}


inline PyMjModel* py_mj_loadXML(const char* filename)
{
    char error[500] = "";
    mjModel* mnew = 0;
    mjModel* m = mj_loadXML(filename, 0, error, 500);
    if (m)
    {
        return new PyMjModel(m);
    }
    return 0;
    
}


PyMjModel* py_mj_loadModel(const char* filename)
{
    mjModel* m = mj_loadModel(filename, 0);
    return new PyMjModel(m);
}

void py_mj_step(const PyMjModel* m, PyMjData* d)
{
    mj_step(m->m_, d->d_);
}


// advance simulation in two phases: before input is set by user
void py_mj_step1(const PyMjModel* m, PyMjData* d)
{
    mj_step1(m->m_, d->d_);
}

// advance simulation in two phases: before input is set by user
void py_mj_step2(const PyMjModel* m, PyMjData* d)
{
    mj_step2(m->m_, d->d_);
}

// advance simulation in two phases: before input is set by user
void py_mj_forward(const PyMjModel* m, PyMjData* d)
{
    mj_forward(m->m_, d->d_);
}

void py_mj_inverse(const PyMjModel* m, PyMjData* d)
{
    mj_inverse(m->m_, d->d_);
}

void py_mj_forwardSkip(const PyMjModel* m, PyMjData* d, int skipstage, int skipsensor)
{
    mj_forwardSkip(m->m_, d->d_, skipstage, skipsensor);
}

void py_mj_inverseSkip(const PyMjModel* m, PyMjData* d, int skipstage, int skipsensor)
{
    mj_inverseSkip(m->m_, d->d_, skipstage, skipsensor);
}

void py_mj_printData(const PyMjModel* m, PyMjData* d, const char* filename)
{
    mj_printData(m->m_, d->d_, filename);
}
  

void py_mj_fwdPosition(const PyMjModel* m, PyMjData* d)
{
    mj_fwdPosition(m->m_, d->d_);
}

void py_mj_fwdVelocity(const PyMjModel* m, PyMjData* d)
{
    mj_fwdVelocity(m->m_, d->d_);
}

void py_mj_fwdActuation(const PyMjModel* m, PyMjData* d)
{
    mj_fwdActuation(m->m_, d->d_);
}

void py_mj_fwdAcceleration(const PyMjModel* m, PyMjData* d)
{
    mj_fwdAcceleration(m->m_, d->d_);
}


void py_mj_fwdConstraint(const PyMjModel* m, PyMjData* d)
{
    mj_fwdConstraint(m->m_, d->d_);
}


void py_mj_Euler(const PyMjModel* m, PyMjData* d)
{
    mj_Euler(m->m_, d->d_);
}


void py_mj_checkPos(const PyMjModel* m, PyMjData* d)
{
    mj_checkPos(m->m_, d->d_);
}

void py_mj_checkVel(const PyMjModel* m, PyMjData* d)
{
    mj_checkVel(m->m_, d->d_);
}

void py_mj_checkAcc(const PyMjModel* m, PyMjData* d)
{
    mj_checkAcc(m->m_, d->d_);
}

void py_mj_kinematics(const PyMjModel* m, PyMjData* d)
{
    mj_kinematics(m->m_, d->d_);
}

void py_mj_comPos(const PyMjModel* m, PyMjData* d)
{
    mj_comPos(m->m_, d->d_);
}

void py_mj_collision(const PyMjModel* m, PyMjData* d)
{
    mj_collision(m->m_, d->d_);
}


void py_mj_printModel(const PyMjModel* m, const char* filename)
{
    mj_printModel(m->m_, filename);
}


struct PyMjRenderer
{
    PyMjRenderer()
    {
        py_mjv_init();
    }

    bool render(PyMjModel* m, PyMjData* d)
    {
        return py_mjv_render(m->m_, d->d_);
    }

    virtual ~PyMjRenderer()
    {
        py_mjv_exit();
    }
};

PyMjRenderer* py_mjv_create_renderer()
{
    PyMjRenderer* r = new PyMjRenderer();
    return r;
}

void py_mjv_delete_renderer(PyMjRenderer* r)
{
    delete r;
}


PYBIND11_MODULE(pymujoco, m) {
#include "pymujoco.inl"
}
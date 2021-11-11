


  m.doc() = R"pbdoc(
        pymujoco python plugin
        -----------------------

        .. currentmodule:: pymujoco

        .. autosummary::
           :toctree: _generate

    )pbdoc";

  py::class_<mjModel>(m, "mjModel")
   
   .def_readonly("nq", &mjModel::nq)// number of generalized coordinates = dim(qpos)
   .def_readonly("nv", &mjModel::nv)// number of degrees of freedom = dim(qvel)
  
    // ------------------------------- sizes

    // sizes needed at mjModel construction
    .def_readonly("nu", &mjModel::nu)                         // number of actuators/controls = dim(ctrl)
    .def_readonly("na", &mjModel::na)                         // number of activation states = dim(act)
    .def_readonly("nbody", &mjModel::nbody)                      // number of bodies
    .def_readonly("njnt", &mjModel::njnt)                       // number of joints
    .def_readonly("ngeom", &mjModel::ngeom)                      // number of geoms
    .def_readonly("nsite", &mjModel::nsite)                      // number of sites
    .def_readonly("ncam", &mjModel::ncam)                       // number of cameras
    .def_readonly("nlight", &mjModel::nlight)                     // number of lights
    .def_readonly("nmesh", &mjModel::nmesh)                      // number of meshes
    .def_readonly("nmeshvert", &mjModel::nmeshvert)                  // number of vertices in all meshes
    .def_readonly("nmeshtexvert", &mjModel::nmeshtexvert)               // number of vertices with texcoords in all meshes
    .def_readonly("nmeshface", &mjModel::nmeshface)                  // number of triangular faces in all meshes
    .def_readonly("nmeshgraph", &mjModel::nmeshgraph)                 // number of ints in mesh auxiliary data
    .def_readonly("nskin", &mjModel::nskin)                      // number of skins
    .def_readonly("nskinvert", &mjModel::nskinvert)                  // number of vertices in all skins
    .def_readonly("nskintexvert", &mjModel::nskintexvert)               // number of vertiex with texcoords in all skins
    .def_readonly("nskinface", &mjModel::nskinface)                  // number of triangular faces in all skins
    .def_readonly("nskinbone", &mjModel::nskinbone)                  // number of bones in all skins
    .def_readonly("nskinbonevert", &mjModel::nskinbonevert)              // number of vertices in all skin bones
    .def_readonly("nhfield", &mjModel::nhfield)                    // number of heightfields
    .def_readonly("nhfielddata", &mjModel::nhfielddata)                // number of data points in all heightfields
    .def_readonly("ntex", &mjModel::ntex)                       // number of textures
    .def_readonly("ntexdata", &mjModel::ntexdata)                   // number of bytes in texture rgb data
    .def_readonly("nmat", &mjModel::nmat)                       // number of materials
    .def_readonly("npair", &mjModel::npair)                      // number of predefined geom pairs
    .def_readonly("nexclude", &mjModel::nexclude)                   // number of excluded geom pairs
    .def_readonly("neq", &mjModel::neq)                        // number of equality constraints
    .def_readonly("ntendon", &mjModel::ntendon)                    // number of tendons
    .def_readonly("nwrap", &mjModel::nwrap)                      // number of wrap objects in all tendon paths
    .def_readonly("nsensor", &mjModel::nsensor)                    // number of sensors
    .def_readonly("nnumeric", &mjModel::nnumeric)                   // number of numeric custom fields
    .def_readonly("nnumericdata", &mjModel::nnumericdata)               // number of mjtNums in all numeric fields
    .def_readonly("ntext", &mjModel::ntext)                      // number of text custom fields
    .def_readonly("ntextdata", &mjModel::ntextdata)                  // number of mjtBytes in all text fields
    .def_readonly("ntuple", &mjModel::ntuple)                     // number of tuple custom fields
    .def_readonly("ntupledata", &mjModel::ntupledata)                 // number of objects in all tuple fields
    .def_readonly("nkey", &mjModel::nkey)                       // number of keyframes
    .def_readonly("nmocap", &mjModel::nmocap)                     // number of mocap bodies
    .def_readonly("nuser_body", &mjModel::nuser_body)                 // number of mjtNums in body_user
    .def_readonly("nuser_jnt", &mjModel::nuser_jnt)                  // number of mjtNums in jnt_user
    .def_readonly("nuser_geom", &mjModel::nuser_geom)                 // number of mjtNums in geom_user
    .def_readonly("nuser_site", &mjModel::nuser_site)                 // number of mjtNums in site_user
    .def_readonly("nuser_cam", &mjModel::nuser_cam)                  // number of mjtNums in cam_user
    .def_readonly("nuser_tendon", &mjModel::nuser_tendon)               // number of mjtNums in tendon_user
    .def_readonly("nuser_actuator", &mjModel::nuser_actuator)             // number of mjtNums in actuator_user
    .def_readonly("nuser_sensor", &mjModel::nuser_sensor)               // number of mjtNums in sensor_user
    .def_readonly("nnames", &mjModel::nnames)                     // number of chars in all names

    // sizes set after mjModel construction (only affect mjData)
    .def_readonly("nM", &mjModel::nM)                         // number of non-zeros in sparse inertia matrix
    .def_readonly("nemax", &mjModel::nemax)                      // number of potential equality-constraint rows
    .def_readonly("njmax", &mjModel::njmax)                      // number of available rows in constraint Jacobian
    .def_readonly("nconmax", &mjModel::nconmax)                    // number of potential contacts in contact list
    .def_readonly("nstack", &mjModel::nstack)                     // number of fields in mjData stack
    .def_readonly("nuserdata", &mjModel::nuserdata)                  // number of extra fields in mjData
    .def_readonly("nsensordata", &mjModel::nsensordata)                // number of fields in sensor data vector
    
     .def_readonly("nbuffer", &mjModel::nbuffer)                    // number of bytes in buffer

    // ------------------------------- options and statistics

    //mjOption opt;                   // physics options
    //mjVisual vis;                   // visualization options
    //mjStatistic stat;               // model statistics

    // ------------------------------- buffers

    // main buffer
    //void*     buffer;               // main buffer; all pointers point in it    (nbuffer)

    // default generalized coordinates
    .def_readonly("qpos0", &mjModel::qpos0)                // qpos values at default pose              (nq x 1)
    .def_readonly("qpos_spring", &mjModel::qpos_spring)          // reference pose for springs               (nq x 1)

    // bodies
    .def_readonly("body_parentid", &mjModel::body_parentid)        // id of body's parent                      (nbody x 1)
    .def_readonly("body_rootid", &mjModel::body_rootid)          // id of root above body                    (nbody x 1)
    .def_readonly("body_weldid", &mjModel::body_weldid)          // id of body that this body is welded to   (nbody x 1)
    .def_readonly("body_mocapid", &mjModel::body_mocapid)         // id of mocap data; -1: none               (nbody x 1)
    .def_readonly("body_jntnum", &mjModel::body_jntnum)          // number of joints for this body           (nbody x 1)
    .def_readonly("body_jntadr", &mjModel::body_jntadr)          // start addr of joints; -1: no joints      (nbody x 1)
    .def_readonly("body_dofnum", &mjModel::body_dofnum)          // number of motion degrees of freedom      (nbody x 1)
    .def_readonly("body_dofadr", &mjModel::body_dofadr)          // start addr of dofs; -1: no dofs          (nbody x 1)
    .def_readonly("body_geomnum", &mjModel::body_geomnum)         // number of geoms                          (nbody x 1)
    .def_readonly("body_geomadr", &mjModel::body_geomadr)         // start addr of geoms; -1: no geoms        (nbody x 1)

    .def_readonly("body_simple", &mjModel::body_simple)          // body is simple (has diagonal M)          (nbody x 1)
    .def_readonly("body_sameframe", &mjModel::body_sameframe)       // inertial frame is same as body frame     (nbody x 1)
    .def_readonly("body_pos", &mjModel::body_pos)             // position offset rel. to parent body      (nbody x 3)
    .def_readonly("body_quat", &mjModel::body_quat)            // orientation offset rel. to parent body   (nbody x 4)
    .def_readonly("body_ipos", &mjModel::body_ipos)            // local position of center of mass         (nbody x 3)
    .def_readonly("body_iquat", &mjModel::body_iquat)           // local orientation of inertia ellipsoid   (nbody x 4)
    .def_readonly("body_mass", &mjModel::body_mass)            // mass                                     (nbody x 1)
    .def_readonly("body_subtreemass", &mjModel::body_subtreemass)     // mass of subtree starting at this body    (nbody x 1)
    .def_readonly("body_inertia", &mjModel::body_inertia)         // diagonal inertia in ipos/iquat frame     (nbody x 3)
    .def_readonly("body_invweight0", &mjModel::body_invweight0)      // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
    //.def_readonly("body_user", &mjModel::body_user)            // user data                                (nbody x nuser_body)
#if 0
    // joints
    int*      jnt_type;             // type of joint (mjtJoint)                 (njnt x 1)
    int*      jnt_qposadr;          // start addr in 'qpos' for joint's data    (njnt x 1)
    int*      jnt_dofadr;           // start addr in 'qvel' for joint's data    (njnt x 1)
    int*      jnt_bodyid;           // id of joint's body                       (njnt x 1)
    int*      jnt_group;            // group for visibility                     (njnt x 1)
    mjtByte*  jnt_limited;          // does joint have limits                   (njnt x 1)
    mjtNum*   jnt_solref;           // constraint solver reference: limit       (njnt x mjNREF)
    mjtNum*   jnt_solimp;           // constraint solver impedance: limit       (njnt x mjNIMP)
    mjtNum*   jnt_pos;              // local anchor position                    (njnt x 3)
    mjtNum*   jnt_axis;             // local joint axis                         (njnt x 3)
    mjtNum*   jnt_stiffness;        // stiffness coefficient                    (njnt x 1)
    mjtNum*   jnt_range;            // joint limits                             (njnt x 2)
    mjtNum*   jnt_margin;           // min distance for limit detection         (njnt x 1)
    mjtNum*   jnt_user;             // user data                                (njnt x nuser_jnt)

    // dofs
    int*      dof_bodyid;           // id of dof's body                         (nv x 1)
    int*      dof_jntid;            // id of dof's joint                        (nv x 1)
    int*      dof_parentid;         // id of dof's parent; -1: none             (nv x 1)
    int*      dof_Madr;             // dof address in M-diagonal                (nv x 1)
    int*      dof_simplenum;        // number of consecutive simple dofs        (nv x 1)
    mjtNum*   dof_solref;           // constraint solver reference:frictionloss (nv x mjNREF)
    mjtNum*   dof_solimp;           // constraint solver impedance:frictionloss (nv x mjNIMP)
    mjtNum*   dof_frictionloss;     // dof friction loss                        (nv x 1)
    mjtNum*   dof_armature;         // dof armature inertia/mass                (nv x 1)
    mjtNum*   dof_damping;          // damping coefficient                      (nv x 1)
    mjtNum*   dof_invweight0;       // diag. inverse inertia in qpos0           (nv x 1)
    mjtNum*   dof_M0;               // diag. inertia in qpos0                   (nv x 1)

    // geoms
    int*      geom_type;            // geometric type (mjtGeom)                 (ngeom x 1)
    int*      geom_contype;         // geom contact type                        (ngeom x 1)
    int*      geom_conaffinity;     // geom contact affinity                    (ngeom x 1)
    int*      geom_condim;          // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
    int*      geom_bodyid;          // id of geom's body                        (ngeom x 1)
    int*      geom_dataid;          // id of geom's mesh/hfield (-1: none)      (ngeom x 1)
    int*      geom_matid;           // material id for rendering                (ngeom x 1)
    int*      geom_group;           // group for visibility                     (ngeom x 1)
    int*      geom_priority;        // geom contact priority                    (ngeom x 1)
    mjtByte*  geom_sameframe;       // same as body frame (1) or iframe (2)     (ngeom x 1)
    mjtNum*   geom_solmix;          // mixing coef for solref/imp in geom pair  (ngeom x 1)
    mjtNum*   geom_solref;          // constraint solver reference: contact     (ngeom x mjNREF)
    mjtNum*   geom_solimp;          // constraint solver impedance: contact     (ngeom x mjNIMP)
    mjtNum*   geom_size;            // geom-specific size parameters            (ngeom x 3)
    mjtNum*   geom_rbound;          // radius of bounding sphere                (ngeom x 1)
    mjtNum*   geom_pos;             // local position offset rel. to body       (ngeom x 3)
    mjtNum*   geom_quat;            // local orientation offset rel. to body    (ngeom x 4)
    mjtNum*   geom_friction;        // friction for (slide, spin, roll)         (ngeom x 3)
    mjtNum*   geom_margin;          // detect contact if dist<margin            (ngeom x 1)
    mjtNum*   geom_gap;             // include in solver if dist<margin-gap     (ngeom x 1)
    mjtNum*   geom_user;            // user data                                (ngeom x nuser_geom)
    float*    geom_rgba;            // rgba when material is omitted            (ngeom x 4)

    // sites
    int*      site_type;            // geom type for rendering (mjtGeom)        (nsite x 1)
    int*      site_bodyid;          // id of site's body                        (nsite x 1)
    int*      site_matid;           // material id for rendering                (nsite x 1)
    int*      site_group;           // group for visibility                     (nsite x 1)
    mjtByte*  site_sameframe;       // same as body frame (1) or iframe (2)     (nsite x 1)
    mjtNum*   site_size;            // geom size for rendering                  (nsite x 3)
    mjtNum*   site_pos;             // local position offset rel. to body       (nsite x 3)
    mjtNum*   site_quat;            // local orientation offset rel. to body    (nsite x 4)
    mjtNum*   site_user;            // user data                                (nsite x nuser_site)
    float*    site_rgba;            // rgba when material is omitted            (nsite x 4)

    // cameras
    int*      cam_mode;             // camera tracking mode (mjtCamLight)       (ncam x 1)
    int*      cam_bodyid;           // id of camera's body                      (ncam x 1)
    int*      cam_targetbodyid;     // id of targeted body; -1: none            (ncam x 1)
    mjtNum*   cam_pos;              // position rel. to body frame              (ncam x 3)
    mjtNum*   cam_quat;             // orientation rel. to body frame           (ncam x 4)
    mjtNum*   cam_poscom0;          // global position rel. to sub-com in qpos0 (ncam x 3)
    mjtNum*   cam_pos0;             // global position rel. to body in qpos0    (ncam x 3)
    mjtNum*   cam_mat0;             // global orientation in qpos0              (ncam x 9)
    mjtNum*   cam_fovy;             // y-field of view (deg)                    (ncam x 1)
    mjtNum*   cam_ipd;              // inter-pupilary distance                  (ncam x 1)
    mjtNum*   cam_user;             // user data                                (ncam x nuser_cam)

    // lights
    int*      light_mode;           // light tracking mode (mjtCamLight)        (nlight x 1)
    int*      light_bodyid;         // id of light's body                       (nlight x 1)
    int*      light_targetbodyid;   // id of targeted body; -1: none            (nlight x 1)
    mjtByte*  light_directional;    // directional light                        (nlight x 1)
    mjtByte*  light_castshadow;     // does light cast shadows                  (nlight x 1)
    mjtByte*  light_active;         // is light on                              (nlight x 1)
    mjtNum*   light_pos;            // position rel. to body frame              (nlight x 3)
    mjtNum*   light_dir;            // direction rel. to body frame             (nlight x 3)
    mjtNum*   light_poscom0;        // global position rel. to sub-com in qpos0 (nlight x 3)
    mjtNum*   light_pos0;           // global position rel. to body in qpos0    (nlight x 3)
    mjtNum*   light_dir0;           // global direction in qpos0                (nlight x 3)
    float*    light_attenuation;    // OpenGL attenuation (quadratic model)     (nlight x 3)
    float*    light_cutoff;         // OpenGL cutoff                            (nlight x 1)
    float*    light_exponent;       // OpenGL exponent                          (nlight x 1)
    float*    light_ambient;        // ambient rgb (alpha=1)                    (nlight x 3)
    float*    light_diffuse;        // diffuse rgb (alpha=1)                    (nlight x 3)
    float*    light_specular;       // specular rgb (alpha=1)                   (nlight x 3)

    // meshes
    int*      mesh_vertadr;         // first vertex address                     (nmesh x 1)
    int*      mesh_vertnum;         // number of vertices                       (nmesh x 1)
    int*      mesh_texcoordadr;     // texcoord data address; -1: no texcoord   (nmesh x 1)
    int*      mesh_faceadr;         // first face address                       (nmesh x 1)
    int*      mesh_facenum;         // number of faces                          (nmesh x 1)
    int*      mesh_graphadr;        // graph data address; -1: no graph         (nmesh x 1)
    float*    mesh_vert;            // vertex positions for all meshe           (nmeshvert x 3)
    float*    mesh_normal;          // vertex normals for all meshes            (nmeshvert x 3)
    float*    mesh_texcoord;        // vertex texcoords for all meshes          (nmeshtexvert x 2)
    int*      mesh_face;            // triangle face data                       (nmeshface x 3)
    int*      mesh_graph;           // convex graph data                        (nmeshgraph x 1)

    // skins
    int*      skin_matid;           // skin material id; -1: none               (nskin x 1)
    float*    skin_rgba;            // skin rgba                                (nskin x 4)
    float*    skin_inflate;         // inflate skin in normal direction         (nskin x 1)
    int*      skin_vertadr;         // first vertex address                     (nskin x 1)
    int*      skin_vertnum;         // number of vertices                       (nskin x 1)
    int*      skin_texcoordadr;     // texcoord data address; -1: no texcoord   (nskin x 1)
    int*      skin_faceadr;         // first face address                       (nskin x 1)
    int*      skin_facenum;         // number of faces                          (nskin x 1)
    int*      skin_boneadr;         // first bone in skin                       (nskin x 1)
    int*      skin_bonenum;         // number of bones in skin                  (nskin x 1)
    float*    skin_vert;            // vertex positions for all skin meshes     (nskinvert x 3)
    float*    skin_texcoord;        // vertex texcoords for all skin meshes     (nskintexvert x 2)
    int*      skin_face;            // triangle faces for all skin meshes       (nskinface x 3)
    int*      skin_bonevertadr;     // first vertex in each bone                (nskinbone x 1)
    int*      skin_bonevertnum;     // number of vertices in each bone          (nskinbone x 1)
    float*    skin_bonebindpos;     // bind pos of each bone                    (nskinbone x 3)
    float*    skin_bonebindquat;    // bind quat of each bone                   (nskinbone x 4)
    int*      skin_bonebodyid;      // body id of each bone                     (nskinbone x 1)
    int*      skin_bonevertid;      // mesh ids of vertices in each bone        (nskinbonevert x 1)
    float*    skin_bonevertweight;  // weights of vertices in each bone         (nskinbonevert x 1)

    // height fields
    mjtNum*   hfield_size;          // (x, y, z_top, z_bottom)                  (nhfield x 4)
    int*      hfield_nrow;          // number of rows in grid                   (nhfield x 1)
    int*      hfield_ncol;          // number of columns in grid                (nhfield x 1)
    int*      hfield_adr;           // address in hfield_data                   (nhfield x 1)
    float*    hfield_data;          // elevation data                           (nhfielddata x 1)

    // textures
    int*      tex_type;             // texture type (mjtTexture)                (ntex x 1)
    int*      tex_height;           // number of rows in texture image          (ntex x 1)
    int*      tex_width;            // number of columns in texture image       (ntex x 1)
    int*      tex_adr;              // address in rgb                           (ntex x 1)
    mjtByte*  tex_rgb;              // rgb (alpha = 1)                          (ntexdata x 1)

    // materials
    int*      mat_texid;            // texture id; -1: none                     (nmat x 1)
    mjtByte*  mat_texuniform;       // make texture cube uniform                (nmat x 1)
    float*    mat_texrepeat;        // texture repetition for 2d mapping        (nmat x 2)
    float*    mat_emission;         // emission (x rgb)                         (nmat x 1)
    float*    mat_specular;         // specular (x white)                       (nmat x 1)
    float*    mat_shininess;        // shininess coef                           (nmat x 1)
    float*    mat_reflectance;      // reflectance (0: disable)                 (nmat x 1)
    float*    mat_rgba;             // rgba                                     (nmat x 4)

    // predefined geom pairs for collision detection; has precedence over exclude
    int*      pair_dim;             // contact dimensionality                   (npair x 1)
    int*      pair_geom1;           // id of geom1                              (npair x 1)
    int*      pair_geom2;           // id of geom2                              (npair x 1)
    int*      pair_signature;       // (body1+1)<<16 + body2+1                  (npair x 1)
    mjtNum*   pair_solref;          // constraint solver reference: contact     (npair x mjNREF)
    mjtNum*   pair_solimp;          // constraint solver impedance: contact     (npair x mjNIMP)
    mjtNum*   pair_margin;          // detect contact if dist<margin            (npair x 1)
    mjtNum*   pair_gap;             // include in solver if dist<margin-gap     (npair x 1)
    mjtNum*   pair_friction;        // tangent1, 2, spin, roll1, 2              (npair x 5)

    // excluded body pairs for collision detection
    int*      exclude_signature;    // (body1+1)<<16 + body2+1                  (nexclude x 1)

    // equality constraints
    int*      eq_type;              // constraint type (mjtEq)                  (neq x 1)
    int*      eq_obj1id;            // id of object 1                           (neq x 1)
    int*      eq_obj2id;            // id of object 2                           (neq x 1)
    mjtByte*  eq_active;            // enable/disable constraint                (neq x 1)
    mjtNum*   eq_solref;            // constraint solver reference              (neq x mjNREF)
    mjtNum*   eq_solimp;            // constraint solver impedance              (neq x mjNIMP)
    mjtNum*   eq_data;              // numeric data for constraint              (neq x mjNEQDATA)

    // tendons
    int*      tendon_adr;           // address of first object in tendon's path (ntendon x 1)
    int*      tendon_num;           // number of objects in tendon's path       (ntendon x 1)
    int*      tendon_matid;         // material id for rendering                (ntendon x 1)
    int*      tendon_group;         // group for visibility                     (ntendon x 1)
    mjtByte*  tendon_limited;       // does tendon have length limits           (ntendon x 1)
    mjtNum*   tendon_width;         // width for rendering                      (ntendon x 1)
    mjtNum*   tendon_solref_lim;    // constraint solver reference: limit       (ntendon x mjNREF)
    mjtNum*   tendon_solimp_lim;    // constraint solver impedance: limit       (ntendon x mjNIMP)
    mjtNum*   tendon_solref_fri;    // constraint solver reference: friction    (ntendon x mjNREF)
    mjtNum*   tendon_solimp_fri;    // constraint solver impedance: friction    (ntendon x mjNIMP)
    mjtNum*   tendon_range;         // tendon length limits                     (ntendon x 2)
    mjtNum*   tendon_margin;        // min distance for limit detection         (ntendon x 1)
    mjtNum*   tendon_stiffness;     // stiffness coefficient                    (ntendon x 1)
    mjtNum*   tendon_damping;       // damping coefficient                      (ntendon x 1)
    mjtNum*   tendon_frictionloss;  // loss due to friction                     (ntendon x 1)
    mjtNum*   tendon_lengthspring;  // tendon length in qpos_spring             (ntendon x 1)
    mjtNum*   tendon_length0;       // tendon length in qpos0                   (ntendon x 1)
    mjtNum*   tendon_invweight0;    // inv. weight in qpos0                     (ntendon x 1)
    mjtNum*   tendon_user;          // user data                                (ntendon x nuser_tendon)
    float*    tendon_rgba;          // rgba when material is omitted            (ntendon x 4)

    // list of all wrap objects in tendon paths
    int*      wrap_type;            // wrap object type (mjtWrap)               (nwrap x 1)
    int*      wrap_objid;           // object id: geom, site, joint             (nwrap x 1)
    mjtNum*   wrap_prm;             // divisor, joint coef, or site id          (nwrap x 1)

    // actuators
    int*      actuator_trntype;     // transmission type (mjtTrn)               (nu x 1)
    int*      actuator_dyntype;     // dynamics type (mjtDyn)                   (nu x 1)
    int*      actuator_gaintype;    // gain type (mjtGain)                      (nu x 1)
    int*      actuator_biastype;    // bias type (mjtBias)                      (nu x 1)
    int*      actuator_trnid;       // transmission id: joint, tendon, site     (nu x 2)
    int*      actuator_group;       // group for visibility                     (nu x 1)
    mjtByte*  actuator_ctrllimited; // is control limited                       (nu x 1)
    mjtByte*  actuator_forcelimited;// is force limited                         (nu x 1)
    mjtNum*   actuator_dynprm;      // dynamics parameters                      (nu x mjNDYN)
    mjtNum*   actuator_gainprm;     // gain parameters                          (nu x mjNGAIN)
    mjtNum*   actuator_biasprm;     // bias parameters                          (nu x mjNBIAS)
    mjtNum*   actuator_ctrlrange;   // range of controls                        (nu x 2)
    mjtNum*   actuator_forcerange;  // range of forces                          (nu x 2)
    mjtNum*   actuator_gear;        // scale length and transmitted force       (nu x 6)
    mjtNum*   actuator_cranklength; // crank length for slider-crank            (nu x 1)
    mjtNum*   actuator_acc0;        // acceleration from unit force in qpos0    (nu x 1)
    mjtNum*   actuator_length0;     // actuator length in qpos0                 (nu x 1)
    mjtNum*   actuator_lengthrange; // feasible actuator length range           (nu x 2)
    mjtNum*   actuator_user;        // user data                                (nu x nuser_actuator)

    // sensors
    int*      sensor_type;          // sensor type (mjtSensor)                  (nsensor x 1)
    int*      sensor_datatype;      // numeric data type (mjtDataType)          (nsensor x 1)
    int*      sensor_needstage;     // required compute stage (mjtStage)        (nsensor x 1)
    int*      sensor_objtype;       // type of sensorized object (mjtObj)       (nsensor x 1)
    int*      sensor_objid;         // id of sensorized object                  (nsensor x 1)
    int*      sensor_dim;           // number of scalar outputs                 (nsensor x 1)
    int*      sensor_adr;           // address in sensor array                  (nsensor x 1)
    mjtNum*   sensor_cutoff;        // cutoff for real and positive; 0: ignore  (nsensor x 1)
    mjtNum*   sensor_noise;         // noise standard deviation                 (nsensor x 1)
    mjtNum*   sensor_user;          // user data                                (nsensor x nuser_sensor)

    // custom numeric fields
    int*      numeric_adr;          // address of field in numeric_data         (nnumeric x 1)
    int*      numeric_size;         // size of numeric field                    (nnumeric x 1)
    mjtNum*   numeric_data;         // array of all numeric fields              (nnumericdata x 1)

    // custom text fields
    int*      text_adr;             // address of text in text_data             (ntext x 1)
    int*      text_size;            // size of text field (strlen+1)            (ntext x 1)
    char*     text_data;            // array of all text fields (0-terminated)  (ntextdata x 1)

    // custom tuple fields
    int*      tuple_adr;            // address of text in text_data             (ntuple x 1)
    int*      tuple_size;           // number of objects in tuple               (ntuple x 1)
    int*      tuple_objtype;        // array of object types in all tuples      (ntupledata x 1)
    int*      tuple_objid;          // array of object ids in all tuples        (ntupledata x 1)
    mjtNum*   tuple_objprm;         // array of object params in all tuples     (ntupledata x 1)

    // keyframes
    mjtNum*   key_time;             // key time                                 (nkey x 1)
    mjtNum*   key_qpos;             // key position                             (nkey x nq)
    mjtNum*   key_qvel;             // key velocity                             (nkey x nv)
    mjtNum*   key_act;              // key activation                           (nkey x na)
    mjtNum*   key_mpos;             // key mocap position                       (nkey x 3*nmocap)
    mjtNum*   key_mquat;            // key mocap quaternion                     (nkey x 4*nmocap)
#endif
    // names
    .def_readonly("name_bodyadr", &mjModel::name_bodyadr)         // body name pointers                       (nbody x 1)
    .def_readonly("name_jntadr", &mjModel::name_jntadr)          // joint name pointers                      (njnt x 1)
    .def_readonly("name_geomadr", &mjModel::name_geomadr)         // geom name pointers                       (ngeom x 1)
    .def_readonly("name_siteadr", &mjModel::name_siteadr)         // site name pointers                       (nsite x 1)
    .def_readonly("name_camadr", &mjModel::name_camadr)          // camera name pointers                     (ncam x 1)
    .def_readonly("name_lightadr", &mjModel::name_lightadr)        // light name pointers                      (nlight x 1)
    .def_readonly("name_meshadr", &mjModel::name_meshadr)         // mesh name pointers                       (nmesh x 1)
    .def_readonly("name_skinadr", &mjModel::name_skinadr)         // skin name pointers                       (nskin x 1)
    .def_readonly("name_hfieldadr", &mjModel::name_hfieldadr)       // hfield name pointers                     (nhfield x 1)
    .def_readonly("name_texadr", &mjModel::name_texadr)          // texture name pointers                    (ntex x 1)
    .def_readonly("name_matadr", &mjModel::name_matadr)          // material name pointers                   (nmat x 1)
    .def_readonly("name_pairadr", &mjModel::name_pairadr)         // geom pair name pointers                  (npair x 1)
    .def_readonly("name_excludeadr", &mjModel::name_excludeadr)      // exclude name pointers                    (nexclude x 1)
    .def_readonly("name_eqadr", &mjModel::name_eqadr)           // equality constraint name pointers        (neq x 1)
    .def_readonly("name_tendonadr", &mjModel::name_tendonadr)       // tendon name pointers                     (ntendon x 1)
    .def_readonly("name_actuatoradr", &mjModel::name_actuatoradr)     // actuator name pointers                   (nu x 1)
    .def_readonly("name_sensoradr", &mjModel::name_sensoradr)       // sensor name pointers                     (nsensor x 1)
    .def_readonly("name_numericadr", &mjModel::name_numericadr)      // numeric name pointers                    (nnumeric x 1)
    .def_readonly("name_textadr", &mjModel::name_textadr)         // text name pointers                       (ntext x 1)
    .def_readonly("name_tupleadr", &mjModel::name_tupleadr)        // tuple name pointers                      (ntuple x 1)
    .def_readonly("name_keyadr", &mjModel::name_keyadr)          // keyframe name pointers                   (nkey x 1)
    .def_readonly("names", &mjModel::names)                // names of all objects, 0-terminated       (nnames x 1)

      ;


//////////////////////////////////////////////////////////////////////////////////

py::class_<mjData>(m, "mjData")
   
// constant sizes
   .def_readwrite("nstack", &mjData::nstack) // number of mjtNums that can fit in stack
   .def_readwrite("nbuffer", &mjData::nbuffer)  // size of main buffer in bytes
   .def_readwrite("pstack", &mjData::pstack) // first available mjtNum address in stack
// memory utilization stats
    .def_readwrite("maxuse_stack", &mjData::maxuse_stack)// maximum stack allocation
    .def_readwrite("maxuse_con", &mjData::maxuse_con)// maximum number of contacts
    .def_readwrite("maxuse_efc", &mjData::maxuse_efc) // maximum number of scalar constraints
    // diagnostics
    //.def_readwrite("warning", &mjData::warning)\
    // .def_readwrite("mjTimerStat", &mjData::mjTimerStat)
    // .def_readwrite("mjSolverStat", &mjData::mjSolverStat) // solver statistics per iteration
    .def_readwrite("solver_iter", &mjData::solver_iter) // number of solver iterations
    .def_readwrite("solver_nnz", &mjData::solver_nnz) // number of non-zeros in Hessian or efc_AR
    //.def_readwrite("solver_fwdinv", &mjData::solver_fwdinv)// forward-inverse comparison: qfrc, efc
// variable sizes
    .def_readwrite("ne", &mjData::ne)  // number of equality constraints
    .def_readwrite("nf", &mjData::nf) // number of friction constraints
    .def_readwrite("nefc", &mjData::nefc) // number of constraints
    .def_readonly("ncon", &mjData::ncon) // number of detected contacts
// global properties

    .def_readwrite("time", &mjData::time) // simulation time
    .def_readonly("energy", &mjData::energy) // potential, kinetic energy
//-------------------------------- end of info header
    // buffers
    //void*     buffer;               // main buffer; all pointers point in it    (nbuffer bytes)
    //mjtNum*   stack;                // stack buffer                             (nstack mjtNums)

    //-------------------------------- main inputs and outputs of the computation

    // state
    .def_readwrite("qpos", &mjData::qpos) // position                                 (nq x 1)
    .def_readwrite("qvel", &mjData::qvel)   // velocity                                 (nv x 1)
    .def_readwrite("act", &mjData::act)   // actuator activation                      (na x 1)
    .def_readwrite("qacc_warmstart", &mjData::qacc_warmstart) // acceleration used for warmstart          (nv x 1)
    // control
    .def_readwrite("ctrl", &mjData::ctrl)  // control                                  (nu x 1)
    .def_readwrite("qfrc_applied", &mjData::qfrc_applied)  // applied generalized force                (nv x 1)
    .def_readwrite("xfrc_applied", &mjData::xfrc_applied)  // applied Cartesian force/torque           (nbody x 6)
    // dynamics  
    .def_readwrite("qacc", &mjData::qacc)                   // acceleration                             (nv x 1)
    .def_readwrite("act_dot", &mjData::act_dot)                 // time-derivative of actuator activation   (na x 1)

// mocap data
    .def_readwrite("mocap_pos", &mjData::mocap_pos)// positions of mocap bodies                (nmocap x 3)
    .def_readwrite("mocap_quat", &mjData::mocap_quat)// orientations of mocap bodies             (nmocap x 4)
     // user data
    //mjtNum*  userdata;              // user data, not touched by engine         (nuserdata x 1)

      // sensors
    .def_readwrite("sensordata", &mjData::sensordata) // sensor data array                        (nsensordata x 1)

        //-------------------------------- POSITION dependent

    // computed by mj_fwdPosition/mj_kinematics
    .def_readwrite("xpos", &mjData::xpos)                 // Cartesian position of body frame         (nbody x 3)
    .def_readwrite("xquat", &mjData::xquat)                // Cartesian orientation of body frame      (nbody x 4)
    .def_readwrite("xmat", &mjData::xmat)                 // Cartesian orientation of body frame      (nbody x 9)
    .def_readwrite("xipos", &mjData::xipos)              // Cartesian position of body com           (nbody x 3)
    .def_readwrite("ximat", &mjData::ximat)               // Cartesian orientation of body inertia    (nbody x 9)
    .def_readwrite("xanchor", &mjData::xanchor)              // Cartesian position of joint anchor       (njnt x 3)
    .def_readwrite("xaxis", &mjData::xaxis)                // Cartesian joint axis                     (njnt x 3)
    .def_readwrite("geom_xpos", &mjData::geom_xpos)            // Cartesian geom position                  (ngeom x 3)
    .def_readwrite("geom_xmat", &mjData::geom_xmat)            // Cartesian geom orientation               (ngeom x 9)
    .def_readwrite("site_xpos", &mjData::site_xpos)            // Cartesian site position                  (nsite x 3)
    .def_readwrite("site_xmat", &mjData::site_xmat)            // Cartesian site orientation               (nsite x 9)
    .def_readwrite("cam_xpos", &mjData::cam_xpos)             // Cartesian camera position                (ncam x 3)
    .def_readwrite("cam_xmat", &mjData::cam_xmat)             // Cartesian camera orientation             (ncam x 9)
    .def_readwrite("light_xpos", &mjData::light_xpos)           // Cartesian light position                 (nlight x 3)
    .def_readwrite("light_xdir", &mjData::light_xdir)           // Cartesian light direction                (nlight x 3)

// computed by mj_fwdPosition/mj_comPos
    .def_readwrite("subtree_com", &mjData::subtree_com)         // center of mass of each subtree           (nbody x 3)
    .def_readwrite("cdof", &mjData::cdof)                 // com-based motion axis of each dof        (nv x 6)
    .def_readwrite("cinert", &mjData::cinert)               // com-based body inertia and mass          (nbody x 10)

    

    // computed by mj_fwdPosition/mj_tendon
    .def_readwrite("ten_wrapadr", &mjData::ten_wrapadr)          // start address of tendon's path           (ntendon x 1)
    .def_readwrite("ten_wrapnum", &mjData::ten_wrapnum)          // number of wrap points in path            (ntendon x 1)
    .def_readwrite("ten_J_rownnz", &mjData::ten_J_rownnz)        // number of non-zeros in Jacobian row      (ntendon x 1)
    .def_readwrite("ten_J_rowadr", &mjData::ten_J_rowadr)         // row start address in colind array        (ntendon x 1)
    .def_readwrite("ten_J_colind", &mjData::ten_J_colind)         // column indices in sparse Jacobian        (ntendon x nv)
    .def_readwrite("ten_length", &mjData::ten_length)           // tendon lengths                           (ntendon x 1)
    .def_readwrite("ten_J", &mjData::ten_J)                // tendon Jacobian                          (ntendon x nv)
    .def_readwrite("wrap_obj", &mjData::wrap_obj)             // geom id; -1: site; -2: pulley            (nwrap*2 x 1)
    .def_readwrite("wrap_xpos", &mjData::wrap_xpos)            // Cartesian 3D points in all path          (nwrap*2 x 3)

    
    // computed by mj_fwdPosition/mj_transmission
    .def_readwrite("actuator_length", &mjData::actuator_length)      // actuator lengths                         (nu x 1)
    .def_readwrite("actuator_moment", &mjData::actuator_moment)      // actuator moments                         (nu x nv)

    

    // computed by mj_fwdPosition/mj_crb
    .def_readwrite("crb", &mjData::crb)                  // com-based composite inertia and mass     (nbody x 10)
    .def_readwrite("qM", &mjData::qM)                  // total inertia                            (nM x 1)

    // computed by mj_fwdPosition/mj_factorM
    .def_readwrite("qLD", &mjData::qLD)                  // L'*D*L factorization of M                (nM x 1)
    .def_readwrite("qLDiagInv", &mjData::qLDiagInv)            // 1/diag(D)                                (nv x 1)
    .def_readwrite("qLDiagSqrtInv", &mjData::qLDiagSqrtInv)        // 1/sqrt(diag(D))                          (nv x 1)

    // computed by mj_fwdPosition/mj_collision
    .def_readwrite("contact", &mjData::contact)             // list of all detected contacts            (nconmax x 1)

          // computed by mj_fwdPosition/mj_makeConstraint
    .def_readwrite("efc_type", &mjData::efc_type)             // constraint type (mjtConstraint)          (njmax x 1)
    .def_readwrite("efc_id", &mjData::efc_id)               // id of object of specified type           (njmax x 1)
    .def_readwrite("efc_J_rownnz", &mjData::efc_J_rownnz)         // number of non-zeros in Jacobian row      (njmax x 1)
    .def_readwrite("efc_J_rowadr", &mjData::efc_J_rowadr)         // row start address in colind array        (njmax x 1)
    .def_readwrite("efc_J_rowsuper", &mjData::efc_J_rowsuper)       // number of subsequent rows in supernode   (njmax x 1)
    .def_readwrite("efc_J_colind", &mjData::efc_J_colind)         // column indices in Jacobian               (njmax x nv)
    .def_readwrite("efc_JT_rownnz", &mjData::efc_JT_rownnz)        // number of non-zeros in Jacobian row    T (nv x 1)
    .def_readwrite("efc_JT_rowadr", &mjData::efc_JT_rowadr)        // row start address in colind array      T (nv x 1)
    .def_readwrite("efc_JT_rowsuper", &mjData::efc_JT_rowsuper)      // number of subsequent rows in supernode T (nv x 1)
    .def_readwrite("efc_JT_colind", &mjData::efc_JT_colind)        // column indices in Jacobian             T (nv x njmax)
    .def_readwrite("efc_J", &mjData::efc_J)                // constraint Jacobian                      (njmax x nv)
    .def_readwrite("efc_JT", &mjData::efc_JT)               // constraint Jacobian transposed           (nv x njmax)
    .def_readwrite("efc_pos", &mjData::efc_pos)              // constraint position (equality, contact)  (njmax x 1)
    .def_readwrite("efc_margin", &mjData::efc_margin)           // inclusion margin (contact)               (njmax x 1)
    .def_readwrite("efc_frictionloss", &mjData::efc_frictionloss)     // frictionloss (friction)                  (njmax x 1)
    .def_readwrite("efc_diagApprox", &mjData::efc_diagApprox)       // approximation to diagonal of A           (njmax x 1)
    .def_readwrite("efc_KBIP", &mjData::efc_KBIP)             // stiffness, damping, impedance, imp'      (njmax x 4)
    .def_readwrite("efc_D", &mjData::efc_D)                // constraint mass                          (njmax x 1)
    .def_readwrite("efc_R", &mjData::efc_R)                // inverse constraint mass                  (njmax x 1)

    // computed by mj_fwdPosition/mj_projectConstraint
    .def_readwrite("efc_AR_rownnz", &mjData::efc_AR_rownnz)        // number of non-zeros in AR                (njmax x 1)
    .def_readwrite("efc_AR_rowadr", &mjData::efc_AR_rowadr)        // row start address in colind array        (njmax x 1)
    .def_readwrite("efc_AR_colind", &mjData::efc_AR_colind)       // column indices in sparse AR              (njmax x njmax)
    .def_readwrite("efc_AR", &mjData::efc_AR)               // J*inv(M)*J' + R                          (njmax x njmax)

    //-------------------------------- POSITION, VELOCITY dependent

    // computed by mj_fwdVelocity
    .def_readwrite("ten_velocity", &mjData::ten_velocity)         // tendon velocities                        (ntendon x 1)
    .def_readwrite("actuator_velocity", &mjData::actuator_velocity)    // actuator velocities                      (nu x 1)

    // computed by mj_fwdVelocity/mj_comVel
    .def_readwrite("cvel", &mjData::cvel)                 // com-based velocity [3D rot; 3D tran]     (nbody x 6)
    .def_readwrite("cdof_dot", &mjData::cdof_dot)             // time-derivative of cdof                  (nv x 6)

    // computed by mj_fwdVelocity/mj_rne (without acceleration)
    .def_readwrite("qfrc_bias", &mjData::qfrc_bias)            // C(qpos,qvel)                             (nv x 1)

    // computed by mj_fwdVelocity/mj_passive
    .def_readwrite("qfrc_passive", &mjData::qfrc_passive)         // passive force                            (nv x 1)

    // computed by mj_fwdVelocity/mj_referenceConstraint
    .def_readwrite("efc_vel", &mjData::efc_vel)              // velocity in constraint space: J*qvel     (njmax x 1)
    .def_readwrite("efc_aref", &mjData::efc_aref)             // reference pseudo-acceleration            (njmax x 1)

    // computed by mj_sensorVel/mj_subtreeVel if needed
    .def_readwrite("subtree_linvel", &mjData::subtree_linvel)       // linear velocity of subtree com           (nbody x 3)
    .def_readwrite("subtree_angmom", &mjData::subtree_angmom)       // angular momentum about subtree com       (nbody x 3)

    //-------------------------------- POSITION, VELOCITY, CONTROL/ACCELERATION dependent

    // computed by mj_fwdActuation
    .def_readwrite("actuator_force", &mjData::actuator_force)       // actuator force in actuation space        (nu x 1)
    .def_readwrite("qfrc_actuator", &mjData::qfrc_actuator)        // actuator force                           (nv x 1)

    // computed by mj_fwdAcceleration
    .def_readwrite("qfrc_unc", &mjData::qfrc_unc)             // net unconstrained force                  (nv x 1)
    .def_readwrite("qacc_unc", &mjData::qacc_unc)             // unconstrained acceleration               (nv x 1)

    // computed by mj_fwdConstraint/mj_inverse
    .def_readwrite("efc_b", &mjData::efc_b)                // linear cost term: J*qacc_unc - aref      (njmax x 1)
    .def_readwrite("efc_force", &mjData::efc_force)            // constraint force in constraint space     (njmax x 1)
    .def_readwrite("efc_state", &mjData::efc_state)            // constraint state (mjtConstraintState)    (njmax x 1)
    .def_readwrite("qfrc_constraint", &mjData::qfrc_constraint)      // constraint force                         (nv x 1)

    // computed by mj_inverse
    .def_readwrite("qfrc_inverse", &mjData::qfrc_inverse)         // net external force; should equal:        (nv x 1)
                                    //  qfrc_applied + J'*xfrc_applied + qfrc_actuator

    // computed by mj_sensorAcc/mj_rnePostConstraint if needed; rotation:translation format
    .def_readwrite("cacc", &mjData::cacc)                 // com-based acceleration                   (nbody x 6)
    .def_readwrite("cfrc_int", &mjData::cfrc_int)             // com-based interaction force with parent  (nbody x 6)
    .def_readwrite("cfrc_ext", &mjData::cfrc_ext)             // com-based external force on body         (nbody x 6)

    ;




//////////////////////////////////////////////////////////////////////////////////


  m.def("mj_version", &mj_version);
  m.def("mju_error", &mju_error);

  //---------------------- Parse and compile ----------------------------------------------


  m.def("mj_loadXML", &mj_loadXML2, pybind11::return_value_policy::reference);
  m.def("mj_loadModel", &mj_loadModel2, pybind11::return_value_policy::reference);

  m.def("mj_deleteModel", &mj_deleteModel);

  m.def("mj_saveLastXML", &mj_saveLastXML);
  m.def("mj_saveModel", &mj_saveModel);

  
  m.def("mj_makeData", &mj_makeData, pybind11::return_value_policy::reference);
  m.def("mj_deleteData", &mj_deleteData);
  

  m.def("mj_name2id", &mj_name2id);
  m.def("mju_copy", &mju_copy);
  
  

  //---------------------- Main simulation ------------------------------------------------

// Advance simulation, use control callback to obtain external force and control.
 m.def("mj_step", &mj_step);
// Advance simulation in two steps: before external force and control is set by user.

 m.def("mj_step1", &mj_step1);

// Advance simulation in two steps: after external force and control is set by user.
 m.def("mj_step2", &mj_step2);

// Forward dynamics: same as mj_step but do not integrate in time.
  m.def("mj_forward", &mj_forward);


// Inverse dynamics: qacc must be set before calling.
   m.def("mj_inverse", &mj_inverse);


// Forward dynamics with skip; skipstage is mjtStage.
   m.def("mj_forwardSkip", &mj_forwardSkip);

// Inverse dynamics with skip; skipstage is mjtStage.
m.def("mj_inverseSkip", &mj_inverseSkip);


//---------------------- Printing -------------------------------------------------------
  m.def("mj_printModel", &mj_printModel);
  
  // Print data to text file.
   m.def("mj_printData", &mj_printData);


// Print matrix to screen.
   m.def("mju_printMat", &mju_printMat);


// Print sparse matrix to screen.
      m.def("mju_printMatSparse", &mju_printMatSparse);
  
  

      
//---------------------- Components -----------------------------------------------------

// Run position-dependent computations.
  m.def("mj_fwdPosition", &mj_fwdPosition);


// Run velocity-dependent computations.
    m.def("mj_fwdVelocity", &mj_fwdVelocity);


// Compute actuator force qfrc_actuation.
 m.def("mj_fwdActuation", &mj_fwdActuation);

// Add up all non-constraint forces, compute qacc_unc.
 m.def("mj_fwdAcceleration", &mj_fwdAcceleration);


// Run selected constraint solver.
  m.def("mj_fwdConstraint", &mj_fwdConstraint);

// Euler integrator, semi-implicit in velocity.
  m.def("mj_Euler", &mj_Euler);


// Runge-Kutta explicit order-N integrator.
  m.def("mj_RungeKutta", &mj_RungeKutta);


// Run position-dependent computations in inverse dynamics.
  m.def("mj_invPosition", &mj_invPosition);


// Run velocity-dependent computations in inverse dynamics.
  m.def("mj_invVelocity", &mj_invVelocity);

   
// Apply the analytical formula for inverse constraint dynamics.
  m.def("mj_invConstraint", &mj_invConstraint);


// Compare forward and inverse dynamics, save results in fwdinv.
    m.def("mj_compareFwdInv", &mj_compareFwdInv);

  
    

//---------------------- Sub components -------------------------------------------------

// Evaluate position-dependent sensors.
     m.def("mj_sensorPos", &mj_sensorPos);


// Evaluate velocity-dependent sensors.
  m.def("mj_sensorVel", &mj_sensorVel);


// Evaluate acceleration and force-dependent sensors.
    m.def("mj_sensorAcc", &mj_sensorAcc);


// Evaluate position-dependent energy (potential).
    m.def("mj_energyPos", &mj_energyPos);

    
    
// Evaluate velocity-dependent energy (kinetic).
  m.def("mj_energyVel", &mj_energyVel);


// Check qpos, reset if any element is too big or nan.
    m.def("mj_checkPos", &mj_checkPos);


// Check qvel, reset if any element is too big or nan.
  m.def("mj_checkVel", &mj_checkVel);

// Check qacc, reset if any element is too big or nan.
  m.def("mj_checkAcc", &mj_checkAcc);


// Run forward kinematics.
    m.def("mj_kinematics", &mj_kinematics);

// Map inertias and motion dofs to global frame centered at CoM.
   m.def("mj_comPos", &mj_comPos);

// Compute camera and light positions and orientations.
      m.def("mj_camlight", &mj_camlight);

// Compute tendon lengths, velocities and moment arms.
   m.def("mj_tendon", &mj_tendon);

// Compute actuator transmission lengths and moments.
  m.def("mj_transmission", &mj_transmission);

// Run composite rigid body inertia algorithm (CRB).
    m.def("mj_crb", &mj_crb);

// Compute sparse L'*D*L factorizaton of inertia matrix.
   m.def("mj_factorM", &mj_factorM);

// Solve linear system M * x = y using factorization:  x = inv(L'*D*L)*y
      m.def("mj_solveM", &mj_solveM);

// Half of linear solve:  x = sqrt(inv(D))*inv(L')*y
      m.def("mj_solveM2", &mj_solveM2);

// Compute cvel, cdof_dot.
    m.def("mj_comVel", &mj_comVel);

// Compute qfrc_passive from spring-dampers, viscosity and density.
  m.def("mj_passive", &mj_passive);

// subtree linear velocity and angular momentum
  m.def("mj_subtreeVel", &mj_subtreeVel);


// RNE: compute M(qpos)*qacc + C(qpos,qvel); flg_acc=0 removes inertial term.
    m.def("mj_rne", &mj_rne);


// RNE with complete data: compute cacc, cfrc_ext, cfrc_int.
    m.def("mj_rnePostConstraint", &mj_rnePostConstraint);


// Run collision detection.
    m.def("mj_collision", &mj_collision);

// Construct constraints.
    m.def("mj_makeConstraint", &mj_makeConstraint);

// Compute inverse constaint inertia efc_AR.
    m.def("mj_projectConstraint", &mj_projectConstraint);


// Compute efc_vel, efc_aref.
    m.def("mj_referenceConstraint", &mj_referenceConstraint);

// Compute efc_state, efc_force, qfrc_constraint, and (optionally) cone Hessians.
// If cost is not NULL, set *cost = s(jar) where jar = Jac*qacc-aref.
    m.def("mj_constraintUpdate", &mj_constraintUpdate);

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////


#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif


  
#ifdef VERSION_INFO
  m.attr("__version__") = VERSION_INFO;
#else
  m.attr("__version__") = "dev";
#endif


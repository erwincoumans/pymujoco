


  m.doc() = R"pbdoc(
        pymujoco python plugin
        -----------------------

        .. currentmodule:: pymujoco

        .. autosummary::
           :toctree: _generate

    )pbdoc";

    py::class_<PyMjModel>(m, "PyMjModel")
    .def_readonly("nq", &PyMjModel::nq,                        R"pbdoc( number of generalized coordinates = dim(qpos))pbdoc")
    .def_readonly("nv", &PyMjModel::nv,                        R"pbdoc( number of degrees of freedom = dim(qvel))pbdoc")
    .def_readonly("nu", &PyMjModel::nu,                        R"pbdoc( number of actuators/controls = dim(ctrl))pbdoc")
    .def_readonly("na", &PyMjModel::na,                        R"pbdoc( number of activation states = dim(act))pbdoc")
    .def_readonly("nbody", &PyMjModel::nbody,                  R"pbdoc( number of bodies)pbdoc")
    .def_readonly("njnt", &PyMjModel::njnt,                    R"pbdoc( number of joints)pbdoc")
    .def_readonly("ngeom", &PyMjModel::ngeom,                  R"pbdoc( number of geoms)pbdoc")
    .def_readonly("nsite", &PyMjModel::nsite,                  R"pbdoc( number of sites)pbdoc")
    .def_readonly("ncam", &PyMjModel::ncam,                    R"pbdoc( number of cameras)pbdoc")
    .def_readonly("nlight", &PyMjModel::nlight,                R"pbdoc( number of lights)pbdoc")
    .def_readonly("nmesh", &PyMjModel::nmesh,                  R"pbdoc( number of meshes)pbdoc")
    .def_readonly("nmeshvert", &PyMjModel::nmeshvert,          R"pbdoc( number of vertices in all meshes)pbdoc")
    .def_readonly("nmeshtexvert", &PyMjModel::nmeshtexvert,    R"pbdoc( number of vertices with texcoords in all meshes)pbdoc")
    .def_readonly("nmeshface", &PyMjModel::nmeshface,          R"pbdoc( number of triangular faces in all meshes)pbdoc")
    .def_readonly("nmeshgraph", &PyMjModel::nmeshgraph,        R"pbdoc( number of ints in mesh auxiliary data)pbdoc")
    .def_readonly("nskin", &PyMjModel::nskin,                  R"pbdoc( number of skins)pbdoc")
    .def_readonly("nskinvert", &PyMjModel::nskinvert,          R"pbdoc( number of vertices in all skins)pbdoc")
    .def_readonly("nskintexvert", &PyMjModel::nskintexvert,    R"pbdoc( number of vertiex with texcoords in all skins)pbdoc")
    .def_readonly("nskinface", &PyMjModel::nskinface,          R"pbdoc( number of triangular faces in all skins)pbdoc")
    .def_readonly("nskinbone", &PyMjModel::nskinbone,          R"pbdoc( number of bones in all skins)pbdoc")
    .def_readonly("nskinbonevert", &PyMjModel::nskinbonevert,  R"pbdoc( number of vertices in all skin bones)pbdoc")
    .def_readonly("nhfield", &PyMjModel::nhfield,              R"pbdoc( number of heightfields)pbdoc")
    .def_readonly("nhfielddata", &PyMjModel::nhfielddata,      R"pbdoc( number of data points in all heightfields)pbdoc")
    .def_readonly("ntex", &PyMjModel::ntex,                    R"pbdoc( number of textures)pbdoc")
    .def_readonly("ntexdata", &PyMjModel::ntexdata,            R"pbdoc( number of bytes in texture rgb data)pbdoc")
    .def_readonly("nmat", &PyMjModel::nmat,                    R"pbdoc( number of materials)pbdoc")
    .def_readonly("npair", &PyMjModel::npair,                  R"pbdoc( number of predefined geom pairs)pbdoc")
    .def_readonly("nexclude", &PyMjModel::nexclude,            R"pbdoc( number of excluded geom pairs)pbdoc")
    .def_readonly("neq", &PyMjModel::neq,                      R"pbdoc( number of equality constraints)pbdoc")
    .def_readonly("ntendon", &PyMjModel::ntendon,              R"pbdoc( number of tendons)pbdoc")
    .def_readonly("nwrap", &PyMjModel::nwrap,                  R"pbdoc( number of wrap objects in all tendon paths)pbdoc")
    .def_readonly("nsensor", &PyMjModel::nsensor,              R"pbdoc( number of sensors)pbdoc")
    .def_readonly("nnumeric", &PyMjModel::nnumeric,            R"pbdoc( number of numeric custom fields)pbdoc")
    .def_readonly("nnumericdata", &PyMjModel::nnumericdata,    R"pbdoc( number of mjtNums in all numeric fields)pbdoc")
    .def_readonly("ntext", &PyMjModel::ntext,                  R"pbdoc( number of text custom fields)pbdoc")
    .def_readonly("ntextdata", &PyMjModel::ntextdata,          R"pbdoc( number of mjtBytes in all text fields)pbdoc")
    .def_readonly("ntuple", &PyMjModel::ntuple,                R"pbdoc( number of tuple custom fields)pbdoc")
    .def_readonly("ntupledata", &PyMjModel::ntupledata,        R"pbdoc( number of objects in all tuple fields)pbdoc")
    .def_readonly("nkey", &PyMjModel::nkey,                    R"pbdoc( number of keyframes)pbdoc")
    .def_readonly("nmocap", &PyMjModel::nmocap,                R"pbdoc( number of mocap bodies)pbdoc")
    .def_readonly("nuser_body", &PyMjModel::nuser_body,        R"pbdoc( number of mjtNums in body_user)pbdoc")
    .def_readonly("nuser_jnt", &PyMjModel::nuser_jnt,          R"pbdoc( number of mjtNums in jnt_user)pbdoc")
    .def_readonly("nuser_geom", &PyMjModel::nuser_geom,        R"pbdoc( number of mjtNums in geom_user)pbdoc")
    .def_readonly("nuser_site", &PyMjModel::nuser_site,        R"pbdoc( number of mjtNums in site_user)pbdoc")
    .def_readonly("nuser_cam", &PyMjModel::nuser_cam,          R"pbdoc( number of mjtNums in cam_user)pbdoc")
    .def_readonly("nuser_tendon", &PyMjModel::nuser_tendon,    R"pbdoc( number of mjtNums in tendon_user)pbdoc")
    .def_readonly("nuser_actuator", &PyMjModel::nuser_actuator,R"pbdoc( number of mjtNums in actuator_user)pbdoc")
    .def_readonly("nuser_sensor", &PyMjModel::nuser_sensor,    R"pbdoc( number of mjtNums in sensor_user)pbdoc")
    .def_readonly("nnames", &PyMjModel::nnames,                R"pbdoc( number of chars in all names)pbdoc")
// sizes set after mjModel construction (only affect mjData)
    .def_readonly("nM", &PyMjModel::nM,                        R"pbdoc( number of non-zeros in sparse inertia matrix)pbdoc")
    .def_readonly("nemax", &PyMjModel::nemax,                  R"pbdoc( number of potential equality-constraint rows)pbdoc")
    .def_readonly("njmax", &PyMjModel::njmax,                  R"pbdoc( number of available rows in constraint Jacobian)pbdoc")
    .def_readonly("nconmax", &PyMjModel::nconmax,              R"pbdoc( number of potential contacts in contact list)pbdoc")
    .def_readonly("nstack", &PyMjModel::nstack,                R"pbdoc( number of fields in mjData stack)pbdoc")
    .def_readonly("nuserdata", &PyMjModel::nuserdata,          R"pbdoc( number of extra fields in mjData)pbdoc")
    .def_readonly("nsensordata", &PyMjModel::nsensordata,      R"pbdoc( number of fields in sensor data vector)pbdoc")
    .def_readonly("nbuffer", &PyMjModel::nbuffer,              R"pbdoc( number of bytes in buffer)pbdoc")

// default generalized coordinates
    .def_readonly("qpos0", &PyMjModel::qpos0,                  R"pbdoc(number of degrees of freedom = dim(qvel))pbdoc")
    .def_readonly("qpos_spring", &PyMjModel::qpos_spring,      R"pbdoc(reference pose for springs (nq x 1))pbdoc")
        
// bodies
    .def_readonly("body_parentid", &PyMjModel::body_parentid,  R"pbdoc( id of body's parent                      (nbody x 1))pbdoc")
    .def_readonly("body_rootid", &PyMjModel::body_rootid,        R"pbdoc( id of root above body                    (nbody x 1))pbdoc")
    .def_readonly("body_weldid", &PyMjModel::body_weldid,        R"pbdoc( id of body that this body is welded to   (nbody x 1))pbdoc")
  ;

#if 0
    //todo

    // ------------------------------- options and statistics

    //mjOption opt;                   // physics options
    //mjVisual vis;                   // visualization options
    //mjStatistic stat;               // model statistics

   
    
    
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

    // joints
    .def_readonly("jnt_type", &mjModel::             // type of joint (mjtJoint)                 (njnt x 1)
    .def_readonly("jnt_qposadr", &mjModel::          // start addr in 'qpos' for joint's data    (njnt x 1)
    .def_readonly("jnt_dofadr", &mjModel::           // start addr in 'qvel' for joint's data    (njnt x 1)
    .def_readonly("jnt_bodyid", &mjModel::           // id of joint's body                       (njnt x 1)
    .def_readonly("jnt_group", &mjModel::            // group for visibility                     (njnt x 1)
    .def_readonly("jnt_limited", &mjModel::          // does joint have limits                   (njnt x 1)
    .def_readonly("jnt_solref", &mjModel::           // constraint solver reference: limit       (njnt x mjNREF)
    .def_readonly("jnt_solimp", &mjModel::           // constraint solver impedance: limit       (njnt x mjNIMP)
    .def_readonly("jnt_pos", &mjModel::              // local anchor position                    (njnt x 3)
    .def_readonly("jnt_axis", &mjModel::             // local joint axis                         (njnt x 3)
    .def_readonly("jnt_stiffness", &mjModel::        // stiffness coefficient                    (njnt x 1)
    .def_readonly("jnt_range", &mjModel::            // joint limits                             (njnt x 2)
    .def_readonly("jnt_margin", &mjModel::           // min distance for limit detection         (njnt x 1)
    .def_readonly("jnt_user", &mjModel::             // user data                                (njnt x nuser_jnt)

    // dofs
    .def_readonly("dof_bodyid", &mjModel::           // id of dof's body                         (nv x 1)
    .def_readonly("dof_jntid", &mjModel::            // id of dof's joint                        (nv x 1)
    .def_readonly("dof_parentid", &mjModel::         // id of dof's parent; -1: none             (nv x 1)
    .def_readonly("dof_Madr", &mjModel::             // dof address in M-diagonal                (nv x 1)
    .def_readonly("dof_simplenum", &mjModel::        // number of consecutive simple dofs        (nv x 1)
    .def_readonly("dof_solref", &mjModel::           // constraint solver reference:frictionloss (nv x mjNREF)
    .def_readonly("dof_solimp", &mjModel::           // constraint solver impedance:frictionloss (nv x mjNIMP)
    .def_readonly("dof_frictionloss", &mjModel::     // dof friction loss                        (nv x 1)
    .def_readonly("dof_armature", &mjModel::         // dof armature inertia/mass                (nv x 1)
    .def_readonly("dof_damping", &mjModel::          // damping coefficient                      (nv x 1)
    .def_readonly("dof_invweight0", &mjModel::       // diag. inverse inertia in qpos0           (nv x 1)
    .def_readonly("dof_M0", &mjModel::               // diag. inertia in qpos0                   (nv x 1)

    // geoms
    .def_readonly("geom_type", &mjModel::            // geometric type (mjtGeom)                 (ngeom x 1)
    .def_readonly("geom_contype", &mjModel::         // geom contact type                        (ngeom x 1)
    .def_readonly("geom_conaffinity", &mjModel::     // geom contact affinity                    (ngeom x 1)
    .def_readonly("geom_condim", &mjModel::          // contact dimensionality (1, 3, 4, 6)      (ngeom x 1)
    .def_readonly("geom_bodyid", &mjModel::          // id of geom's body                        (ngeom x 1)
    .def_readonly("geom_dataid", &mjModel::          // id of geom's mesh/hfield (-1: none)      (ngeom x 1)
    .def_readonly("geom_matid", &mjModel::           // material id for rendering                (ngeom x 1)
    .def_readonly("geom_group", &mjModel::           // group for visibility                     (ngeom x 1)
    .def_readonly("geom_priority", &mjModel::        // geom contact priority                    (ngeom x 1)
    .def_readonly("geom_sameframe", &mjModel::       // same as body frame (1) or iframe (2)     (ngeom x 1)
    .def_readonly("geom_solmix", &mjModel::          // mixing coef for solref/imp in geom pair  (ngeom x 1)
    .def_readonly("geom_solref", &mjModel::          // constraint solver reference: contact     (ngeom x mjNREF)
    .def_readonly("geom_solimp", &mjModel::          // constraint solver impedance: contact     (ngeom x mjNIMP)
    .def_readonly("geom_size", &mjModel::            // geom-specific size parameters            (ngeom x 3)
    .def_readonly("geom_rbound", &mjModel::          // radius of bounding sphere                (ngeom x 1)
    .def_readonly("geom_pos", &mjModel::             // local position offset rel. to body       (ngeom x 3)
    .def_readonly("geom_quat", &mjModel::            // local orientation offset rel. to body    (ngeom x 4)
    .def_readonly("geom_friction", &mjModel::        // friction for (slide, spin, roll)         (ngeom x 3)
    .def_readonly("geom_margin", &mjModel::          // detect contact if dist<margin            (ngeom x 1)
    .def_readonly("geom_gap", &mjModel::             // include in solver if dist<margin-gap     (ngeom x 1)
    .def_readonly("geom_user", &mjModel::            // user data                                (ngeom x nuser_geom)
    .def_readonly("geom_rgba", &mjModel::            // rgba when material is omitted            (ngeom x 4)

    // sites
    .def_readonly("site_type", &mjModel::            // geom type for rendering (mjtGeom)        (nsite x 1)
    .def_readonly("site_bodyid", &mjModel::          // id of site's body                        (nsite x 1)
    .def_readonly("site_matid", &mjModel::           // material id for rendering                (nsite x 1)
    .def_readonly("site_group", &mjModel::           // group for visibility                     (nsite x 1)
    .def_readonly("site_sameframe", &mjModel::       // same as body frame (1) or iframe (2)     (nsite x 1)
    .def_readonly("site_size", &mjModel::            // geom size for rendering                  (nsite x 3)
    .def_readonly("site_pos", &mjModel::             // local position offset rel. to body       (nsite x 3)
    .def_readonly("site_quat", &mjModel::            // local orientation offset rel. to body    (nsite x 4)
    .def_readonly("site_user", &mjModel::            // user data                                (nsite x nuser_site)
    .def_readonly("site_rgba", &mjModel::            // rgba when material is omitted            (nsite x 4)

    // cameras
    .def_readonly("cam_mode", &mjModel::             // camera tracking mode (mjtCamLight)       (ncam x 1)
    .def_readonly("cam_bodyid", &mjModel::           // id of camera's body                      (ncam x 1)
    .def_readonly("cam_targetbodyid", &mjModel::     // id of targeted body; -1: none            (ncam x 1)
    .def_readonly("cam_pos", &mjModel::              // position rel. to body frame              (ncam x 3)
    .def_readonly("cam_quat", &mjModel::             // orientation rel. to body frame           (ncam x 4)
    .def_readonly("cam_poscom0", &mjModel::          // global position rel. to sub-com in qpos0 (ncam x 3)
    .def_readonly("cam_pos0", &mjModel::             // global position rel. to body in qpos0    (ncam x 3)
    .def_readonly("cam_mat0", &mjModel::             // global orientation in qpos0              (ncam x 9)
    .def_readonly("cam_fovy", &mjModel::             // y-field of view (deg)                    (ncam x 1)
    .def_readonly("cam_ipd", &mjModel::              // inter-pupilary distance                  (ncam x 1)
    .def_readonly("cam_user", &mjModel::             // user data                                (ncam x nuser_cam)

    // lights
    .def_readonly("light_mode", &mjModel::           // light tracking mode (mjtCamLight)        (nlight x 1)
    .def_readonly("light_bodyid", &mjModel::         // id of light's body                       (nlight x 1)
    .def_readonly("light_targetbodyid", &mjModel::   // id of targeted body; -1: none            (nlight x 1)
    .def_readonly("light_directional", &mjModel::    // directional light                        (nlight x 1)
    .def_readonly("light_castshadow", &mjModel::     // does light cast shadows                  (nlight x 1)
    .def_readonly("light_active", &mjModel::         // is light on                              (nlight x 1)
    .def_readonly("light_pos", &mjModel::            // position rel. to body frame              (nlight x 3)
    .def_readonly("light_dir", &mjModel::            // direction rel. to body frame             (nlight x 3)
    .def_readonly("light_poscom0", &mjModel::        // global position rel. to sub-com in qpos0 (nlight x 3)
    .def_readonly("light_pos0", &mjModel::           // global position rel. to body in qpos0    (nlight x 3)
    .def_readonly("light_dir0", &mjModel::           // global direction in qpos0                (nlight x 3)
    .def_readonly("light_attenuation", &mjModel::    // OpenGL attenuation (quadratic model)     (nlight x 3)
    .def_readonly("light_cutoff", &mjModel::         // OpenGL cutoff                            (nlight x 1)
    .def_readonly("light_exponent", &mjModel::       // OpenGL exponent                          (nlight x 1)
    .def_readonly("light_ambient", &mjModel::        // ambient rgb (alpha=1)                    (nlight x 3)
    .def_readonly("light_diffuse", &mjModel::        // diffuse rgb (alpha=1)                    (nlight x 3)
    .def_readonly("light_specular", &mjModel::       // specular rgb (alpha=1)                   (nlight x 3)

    // meshes
    .def_readonly("mesh_vertadr", &mjModel::         // first vertex address                     (nmesh x 1)
    .def_readonly("mesh_vertnum", &mjModel::         // number of vertices                       (nmesh x 1)
    .def_readonly("mesh_texcoordadr", &mjModel::     // texcoord data address; -1: no texcoord   (nmesh x 1)
    .def_readonly("mesh_faceadr", &mjModel::         // first face address                       (nmesh x 1)
    .def_readonly("mesh_facenum", &mjModel::         // number of faces                          (nmesh x 1)
    .def_readonly("mesh_graphadr", &mjModel::        // graph data address; -1: no graph         (nmesh x 1)
    .def_readonly("mesh_vert", &mjModel::            // vertex positions for all meshe           (nmeshvert x 3)
    .def_readonly("mesh_normal", &mjModel::          // vertex normals for all meshes            (nmeshvert x 3)
    .def_readonly("mesh_texcoord", &mjModel::        // vertex texcoords for all meshes          (nmeshtexvert x 2)
    .def_readonly("mesh_face", &mjModel::            // triangle face data                       (nmeshface x 3)
    .def_readonly("mesh_graph", &mjModel::           // convex graph data                        (nmeshgraph x 1)

    // skins
    .def_readonly("skin_matid", &mjModel::           // skin material id; -1: none               (nskin x 1)
    .def_readonly("skin_rgba", &mjModel::            // skin rgba                                (nskin x 4)
    .def_readonly("skin_inflate", &mjModel::         // inflate skin in normal direction         (nskin x 1)
    .def_readonly("skin_vertadr", &mjModel::         // first vertex address                     (nskin x 1)
    .def_readonly("skin_vertnum", &mjModel::         // number of vertices                       (nskin x 1)
    .def_readonly("skin_texcoordadr", &mjModel::     // texcoord data address; -1: no texcoord   (nskin x 1)
    .def_readonly("skin_faceadr", &mjModel::         // first face address                       (nskin x 1)
    .def_readonly("skin_facenum", &mjModel::         // number of faces                          (nskin x 1)
    .def_readonly("skin_boneadr", &mjModel::         // first bone in skin                       (nskin x 1)
    .def_readonly("skin_bonenum", &mjModel::         // number of bones in skin                  (nskin x 1)
    .def_readonly("skin_vert", &mjModel::            // vertex positions for all skin meshes     (nskinvert x 3)
    .def_readonly("skin_texcoord", &mjModel::        // vertex texcoords for all skin meshes     (nskintexvert x 2)
    .def_readonly("skin_face", &mjModel::            // triangle faces for all skin meshes       (nskinface x 3)
    .def_readonly("skin_bonevertadr", &mjModel::     // first vertex in each bone                (nskinbone x 1)
    .def_readonly("skin_bonevertnum", &mjModel::     // number of vertices in each bone          (nskinbone x 1)
    .def_readonly("skin_bonebindpos", &mjModel::     // bind pos of each bone                    (nskinbone x 3)
    .def_readonly("skin_bonebindquat", &mjModel::    // bind quat of each bone                   (nskinbone x 4)
    .def_readonly("skin_bonebodyid", &mjModel::      // body id of each bone                     (nskinbone x 1)
    .def_readonly("skin_bonevertid", &mjModel::      // mesh ids of vertices in each bone        (nskinbonevert x 1)
    .def_readonly("skin_bonevertweight", &mjModel::  // weights of vertices in each bone         (nskinbonevert x 1)

    // height fields
    .def_readonly("hfield_size", &mjModel::          // (x, y, z_top, z_bottom)                  (nhfield x 4)
    .def_readonly("hfield_nrow", &mjModel::          // number of rows in grid                   (nhfield x 1)
    .def_readonly("hfield_ncol", &mjModel::          // number of columns in grid                (nhfield x 1)
    .def_readonly("hfield_adr", &mjModel::           // address in hfield_data                   (nhfield x 1)
    .def_readonly("hfield_data", &mjModel::          // elevation data                           (nhfielddata x 1)

    // textures
    .def_readonly("tex_type", &mjModel::             // texture type (mjtTexture)                (ntex x 1)
    .def_readonly("tex_height", &mjModel::           // number of rows in texture image          (ntex x 1)
    .def_readonly("tex_width", &mjModel::            // number of columns in texture image       (ntex x 1)
    .def_readonly("tex_adr", &mjModel::              // address in rgb                           (ntex x 1)
    .def_readonly("tex_rgb", &mjModel::              // rgb (alpha = 1)                          (ntexdata x 1)

    // materials
    .def_readonly("mat_texid", &mjModel::            // texture id; -1: none                     (nmat x 1)
    .def_readonly("at_texuniform", &mjModel::       // make texture cube uniform                (nmat x 1)
    .def_readonly("mat_texrepeat", &mjModel::        // texture repetition for 2d mapping        (nmat x 2)
    .def_readonly("mat_emission", &mjModel::         // emission (x rgb)                         (nmat x 1)
    .def_readonly("mat_specular", &mjModel::         // specular (x white)                       (nmat x 1)
    .def_readonly("mat_shininess", &mjModel::        // shininess coef                           (nmat x 1)
    .def_readonly("mat_reflectance", &mjModel::      // reflectance (0: disable)                 (nmat x 1)
    .def_readonly("mat_rgba", &mjModel::             // rgba                                     (nmat x 4)

    // predefined geom pairs for collision detection; has precedence over exclude
    .def_readonly("pair_dim", &mjModel::             // contact dimensionality                   (npair x 1)
    .def_readonly("pair_geom1", &mjModel::           // id of geom1                              (npair x 1)
    .def_readonly("pair_geom2", &mjModel::           // id of geom2                              (npair x 1)
    .def_readonly("pair_signature", &mjModel::       // (body1+1)<<16 + body2+1                  (npair x 1)
    .def_readonly("pair_solref", &mjModel::          // constraint solver reference: contact     (npair x mjNREF)
    .def_readonly("pair_solimp", &mjModel::          // constraint solver impedance: contact     (npair x mjNIMP)
    .def_readonly("pair_margin", &mjModel::          // detect contact if dist<margin            (npair x 1)
    .def_readonly("pair_gap", &mjModel::             // include in solver if dist<margin-gap     (npair x 1)
    .def_readonly("pair_friction", &mjModel::        // tangent1, 2, spin, roll1, 2              (npair x 5)

    // excluded body pairs for collision detection
    .def_readonly("exclude_signature", &mjModel::    // (body1+1)<<16 + body2+1                  (nexclude x 1)

    // equality constraints
    .def_readonly("eq_type", &mjModel::              // constraint type (mjtEq)                  (neq x 1)
    .def_readonly("eq_obj1id", &mjModel::            // id of object 1                           (neq x 1)
    .def_readonly("eq_obj2id", &mjModel::            // id of object 2                           (neq x 1)
    .def_readonly("eq_active", &mjModel::            // enable/disable constraint                (neq x 1)
    .def_readonly("eq_solref", &mjModel::            // constraint solver reference              (neq x mjNREF)
    .def_readonly("eq_solimp", &mjModel::            // constraint solver impedance              (neq x mjNIMP)
    .def_readonly("eq_data", &mjModel::              // numeric data for constraint              (neq x mjNEQDATA)

    // tendons
    .def_readonly("tendon_adr", &mjModel::           // address of first object in tendon's path (ntendon x 1)
    .def_readonly("tendon_num", &mjModel::           // number of objects in tendon's path       (ntendon x 1)
    .def_readonly("tendon_matid", &mjModel::         // material id for rendering                (ntendon x 1)
    .def_readonly("tendon_group", &mjModel::         // group for visibility                     (ntendon x 1)
    .def_readonly("tendon_limited", &mjModel::       // does tendon have length limits           (ntendon x 1)
    .def_readonly("tendon_width", &mjModel::         // width for rendering                      (ntendon x 1)
    .def_readonly("tendon_solref_lim", &mjModel::    // constraint solver reference: limit       (ntendon x mjNREF)
    .def_readonly("tendon_solimp_lim", &mjModel::    // constraint solver impedance: limit       (ntendon x mjNIMP)
    .def_readonly("tendon_solref_fri", &mjModel::    // constraint solver reference: friction    (ntendon x mjNREF)
    .def_readonly("tendon_solimp_fri", &mjModel::    // constraint solver impedance: friction    (ntendon x mjNIMP)
    .def_readonly("tendon_range", &mjModel::         // tendon length limits                     (ntendon x 2)
    .def_readonly("tendon_margin", &mjModel::        // min distance for limit detection         (ntendon x 1)
    .def_readonly("tendon_stiffness", &mjModel::     // stiffness coefficient                    (ntendon x 1)
    .def_readonly("tendon_damping", &mjModel::       // damping coefficient                      (ntendon x 1)
    .def_readonly("tendon_frictionloss", &mjModel::  // loss due to friction                     (ntendon x 1)
    .def_readonly("tendon_lengthspring", &mjModel::  // tendon length in qpos_spring             (ntendon x 1)
    .def_readonly("tendon_length0", &mjModel::       // tendon length in qpos0                   (ntendon x 1)
    .def_readonly("tendon_invweight0", &mjModel::    // inv. weight in qpos0                     (ntendon x 1)
    .def_readonly("tendon_user", &mjModel::          // user data                                (ntendon x nuser_tendon)
    .def_readonly("tendon_rgba", &mjModel::          // rgba when material is omitted            (ntendon x 4)

    // list of all wrap objects in tendon paths
    .def_readonly("wrap_type", &mjModel::            // wrap object type (mjtWrap)               (nwrap x 1)
    .def_readonly("wrap_objid", &mjModel::           // object id: geom, site, joint             (nwrap x 1)
    .def_readonly("wrap_prm", &mjModel::             // divisor, joint coef, or site id          (nwrap x 1)

    // actuators
    .def_readonly("actuator_trntype", &mjModel::     // transmission type (mjtTrn)               (nu x 1)
    .def_readonly("actuator_dyntype", &mjModel::     // dynamics type (mjtDyn)                   (nu x 1)
    .def_readonly("actuator_gaintype", &mjModel::    // gain type (mjtGain)                      (nu x 1)
    .def_readonly("actuator_biastype", &mjModel::    // bias type (mjtBias)                      (nu x 1)
    .def_readonly("actuator_trnid", &mjModel::       // transmission id: joint, tendon, site     (nu x 2)
    .def_readonly("actuator_group", &mjModel::       // group for visibility                     (nu x 1)
    .def_readonly("actuator_ctrllimited", &mjModel:: // is control limited                       (nu x 1)
    .def_readonly("actuator_forcelimited", &mjModel::// is force limited                         (nu x 1)
    .def_readonly("actuator_dynprm", &mjModel::      // dynamics parameters                      (nu x mjNDYN)
    .def_readonly("actuator_gainprm", &mjModel::     // gain parameters                          (nu x mjNGAIN)
    .def_readonly("actuator_biasprm", &mjModel::     // bias parameters                          (nu x mjNBIAS)
    .def_readonly("actuator_ctrlrange", &mjModel::   // range of controls                        (nu x 2)
    .def_readonly("actuator_forcerange", &mjModel::  // range of forces                          (nu x 2)
    .def_readonly("actuator_gear", &mjModel::        // scale length and transmitted force       (nu x 6)
    .def_readonly("actuator_cranklength", &mjModel:: // crank length for slider-crank            (nu x 1)
    .def_readonly("actuator_acc0", &mjModel::        // acceleration from unit force in qpos0    (nu x 1)
    .def_readonly("actuator_length0", &mjModel::     // actuator length in qpos0                 (nu x 1)
    .def_readonly("actuator_lengthrange", &mjModel:: // feasible actuator length range           (nu x 2)
    .def_readonly("actuator_user", &mjModel::        // user data                                (nu x nuser_actuator)

    // sensors
    .def_readonly("sensor_type", &mjModel::          // sensor type (mjtSensor)                  (nsensor x 1)
    .def_readonly("sensor_datatype", &mjModel::      // numeric data type (mjtDataType)          (nsensor x 1)
    .def_readonly("sensor_needstage", &mjModel::     // required compute stage (mjtStage)        (nsensor x 1)
    .def_readonly("sensor_objtype", &mjModel::       // type of sensorized object (mjtObj)       (nsensor x 1)
    .def_readonly("sensor_objid", &mjModel::         // id of sensorized object                  (nsensor x 1)
    .def_readonly("sensor_dim", &mjModel::           // number of scalar outputs                 (nsensor x 1)
    .def_readonly("sensor_adr", &mjModel::           // address in sensor array                  (nsensor x 1)
    .def_readonly("sensor_cutoff", &mjModel::        // cutoff for real and positive; 0: ignore  (nsensor x 1)
    .def_readonly("sensor_noise", &mjModel::         // noise standard deviation                 (nsensor x 1)
    .def_readonly("sensor_user", &mjModel::          // user data                                (nsensor x nuser_sensor)

    // custom numeric fields
    .def_readonly("numeric_adr", &mjModel::          // address of field in numeric_data         (nnumeric x 1)
    .def_readonly("numeric_size", &mjModel::         // size of numeric field                    (nnumeric x 1)
    .def_readonly("numeric_data", &mjModel::         // array of all numeric fields              (nnumericdata x 1)

    // custom text fields
    .def_readonly("text_adr", &mjModel::             // address of text in text_data             (ntext x 1)
    .def_readonly("text_size", &mjModel::            // size of text field (strlen+1)            (ntext x 1)
    .def_readonly("text_data", &mjModel::            // array of all text fields (0-terminated)  (ntextdata x 1)

    // custom tuple fields
    .def_readonly("tuple_adr", &mjModel::            // address of text in text_data             (ntuple x 1)
    .def_readonly("tuple_size", &mjModel::           // number of objects in tuple               (ntuple x 1)
    .def_readonly("tuple_objtype", &mjModel::        // array of object types in all tuples      (ntupledata x 1)
    .def_readonly("tuple_objid", &mjModel::          // array of object ids in all tuples        (ntupledata x 1)
    .def_readonly("tuple_objprm", &mjModel::         // array of object params in all tuples     (ntupledata x 1)

    // keyframes
    .def_readonly("key_time", &mjModel::             // key time                                 (nkey x 1)
    .def_readonly("key_qpos", &mjModel::             // key position                             (nkey x nq)
    .def_readonly("key_qvel", &mjModel::             // key velocity                             (nkey x nv)
    .def_readonly("key_act", &mjModel::              // key activation                           (nkey x na)
    .def_readonly("key_mpos", &mjModel::             // key mocap position                       (nkey x 3*nmocap)
    .def_readonly("key_mquat", &mjModel::            // key mocap quaternion                     (nkey x 4*nmocap)

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
#endif

//////////////////////////////////////////////////////////////////////////////////

//py::class_<mjData>(m, "mjData")
   
    py::class_<PyMjData>(m, "PyMjData")
#if 0
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
    //.def_readonly("stack;                // stack buffer                             (nstack mjtNums)

    //-------------------------------- main inputs and outputs of the computation
#endif
    // state
    .def_readwrite("qpos", &PyMjData::qpos_) // position                                 (nq x 1)

#if 0
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
#endif
    ;




//////////////////////////////////////////////////////////////////////////////////


  m.def("mj_version", &mj_version);
  m.def("mju_error", &mju_error);

  //---------------------- Parse and compile ----------------------------------------------


  m.def("mj_loadXML", &py_mj_loadXML, pybind11::return_value_policy::reference);
  m.def("mj_loadModel", &py_mj_loadModel, pybind11::return_value_policy::reference);

  m.def("mj_deleteModel", &py_mj_deleteModel);

  //m.def("mj_saveLastXML", &mj_saveLastXML);
  //m.def("mj_saveModel", &mj_saveModel);

  
  m.def("mj_makeData", &py_mj_makeData, pybind11::return_value_policy::reference);
  m.def("mj_deleteData", &py_mj_deleteData);

  m.def("mjv_create_renderer", &py_mjv_create_renderer , pybind11::return_value_policy::reference);
  m.def("mjv_delete_renderer", &py_mjv_delete_renderer);
  
  
  py::class_<PyMjRenderer>(m, "PyMjRenderer")
      .def("render", &PyMjRenderer::render)
      ;
  

  //m.def("mj_name2id", &mj_name2id);
  //m.def("mju_copy", &mju_copy);
  
  

  //---------------------- Main simulation ------------------------------------------------

// Advance simulation, use control callback to obtain external force and control.
 m.def("mj_step", &py_mj_step);

 
 // Advance simulation in two steps: before external force and control is set by user.

 m.def("mj_step1", &mj_step1);

 #if 0

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
#endif
  m.def("mj_printModel", &py_mj_printModel);
  
#if 0
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
#endif
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


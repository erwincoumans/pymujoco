
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
    .def_readonly("body_rootid", &PyMjModel::body_rootid,      R"pbdoc( id of root above body                    (nbody x 1))pbdoc")
    .def_readonly("body_weldid", &PyMjModel::body_weldid,      R"pbdoc( id of body that this body is welded to   (nbody x 1))pbdoc")
    .def_readonly("body_mocapid", &PyMjModel::body_mocapid,    R"pbdoc( id of mocap data; -1: none               (nbody x 1))pbdoc")
    .def_readonly("body_jntnum", &PyMjModel::body_jntnum,      R"pbdoc( number of joints for this body           (nbody x 1))pbdoc")
    .def_readonly("body_jntadr", &PyMjModel::body_jntadr,      R"pbdoc( start addr of joints; -1: no joints      (nbody x 1))pbdoc")
    .def_readonly("body_dofnum", &PyMjModel::body_dofnum,      R"pbdoc( number of motion degrees of freedom      (nbody x 1))pbdoc")
    .def_readonly("body_dofadr", &PyMjModel::body_dofadr,      R"pbdoc( start addr of dofs; -1: no dofs          (nbody x 1))pbdoc")
    .def_readonly("body_geomnum", &PyMjModel::body_geomnum,    R"pbdoc( number of geoms                          (nbody x 1))pbdoc")
    .def_readonly("body_geomadr", &PyMjModel::body_geomadr,    R"pbdoc( start addr of geoms; -1: no geoms        (nbody x 1))pbdoc")

    //mjOption opt;                   // physics options
    //mjVisual vis;                   // visualization options
    //mjStatistic stat;               // model statistics
        
    // ------------------------------- options and statistics
    
    .def_readonly("body_simple", &PyMjModel::body_simple)          // body is simple (has diagonal M)          (nbody x 1)
    .def_readonly("body_sameframe", &PyMjModel::body_sameframe)       // inertial frame is same as body frame     (nbody x 1)
    .def_readonly("body_pos", &PyMjModel::body_pos)             // position offset rel. to parent body      (nbody x 3)
    .def_readonly("body_quat", &PyMjModel::body_quat)            // orientation offset rel. to parent body   (nbody x 4)
    .def_readonly("body_ipos", &PyMjModel::body_ipos)            // local position of center of mass         (nbody x 3)
    .def_readonly("body_iquat", &PyMjModel::body_iquat)           // local orientation of inertia ellipsoid   (nbody x 4)
    .def_readonly("body_mass", &PyMjModel::body_mass)            // mass                                     (nbody x 1)
    .def_readonly("body_subtreemass", &PyMjModel::body_subtreemass)     // mass of subtree starting at this body    (nbody x 1)
    .def_readonly("body_inertia", &PyMjModel::body_inertia)         // diagonal inertia in ipos/iquat frame     (nbody x 3)
    .def_readonly("body_invweight0", &PyMjModel::body_invweight0)      // mean inv inert in qpos0 (trn, rot)       (nbody x 2)
    .def_readonly("body_user", &PyMjModel::body_user)            // user data                                (nbody x nuser_body)

    // joints
    .def_readonly("jnt_type", &PyMjModel::jnt_type)             // type of joint (mjtJoint)                 (njnt x 1)
    .def_readonly("jnt_qposadr", &PyMjModel::jnt_qposadr)          // start addr in 'qpos' for joint's data    (njnt x 1)
    .def_readonly("jnt_dofadr", &PyMjModel::jnt_dofadr)           // start addr in 'qvel' for joint's data    (njnt x 1)
    .def_readonly("jnt_bodyid", &PyMjModel::jnt_bodyid)           // id of joint's body                       (njnt x 1)
    .def_readonly("jnt_group", &PyMjModel::jnt_group)           // group for visibility                     (njnt x 1)
    .def_readonly("jnt_limited", &PyMjModel::jnt_limited)          // does joint have limits                   (njnt x 1)
    .def_readonly("jnt_solref", &PyMjModel::jnt_solref)           // constraint solver reference: limit       (njnt x mjNREF)
    .def_readonly("jnt_solimp", &PyMjModel::jnt_solimp)           // constraint solver impedance: limit       (njnt x mjNIMP)
    .def_readonly("jnt_pos", &PyMjModel::jnt_pos)              // local anchor position                    (njnt x 3)
    .def_readonly("jnt_axis", &PyMjModel::jnt_axis)             // local joint axis                         (njnt x 3)
    .def_readonly("jnt_stiffness", &PyMjModel::jnt_stiffness)        // stiffness coefficient                    (njnt x 1)
    .def_readonly("jnt_range", &PyMjModel::jnt_range)            // joint limits                             (njnt x 2)
    .def_readonly("jnt_margin", &PyMjModel::jnt_margin)           // min distance for limit detection         (njnt x 1)
    .def_readonly("jnt_user", &PyMjModel::jnt_user)             // user data                                (njnt x nuser_jnt)

    // dofs
    .def_readonly("dof_bodyid", &PyMjModel::dof_bodyid)           // id of dof's body                         (nv x 1)
    .def_readonly("dof_jntid", &PyMjModel::dof_jntid)            // id of dof's joint                        (nv x 1)
    .def_readonly("dof_parentid", &PyMjModel::dof_parentid)         // id of dof's parent; -1: none             (nv x 1)
    .def_readonly("dof_Madr", &PyMjModel::dof_Madr)             // dof address in M-diagonal                (nv x 1)
    .def_readonly("dof_simplenum", &PyMjModel::dof_simplenum)        // number of consecutive simple dofs        (nv x 1)
    .def_readonly("dof_solref", &PyMjModel::dof_solref)           // constraint solver reference:frictionloss (nv x mjNREF)
    .def_readonly("dof_solimp", &PyMjModel::dof_solimp)           // constraint solver impedance:frictionloss (nv x mjNIMP)
    .def_readonly("dof_frictionloss", &PyMjModel::dof_frictionloss)     // dof friction loss                        (nv x 1)
    .def_readonly("dof_armature", &PyMjModel::dof_armature)         // dof armature inertia/mass                (nv x 1)
    .def_readonly("dof_damping", &PyMjModel::dof_damping)          // damping coefficient                      (nv x 1)
    .def_readonly("dof_invweight0", &PyMjModel::dof_invweight0)       // diag. inverse inertia in qpos0           (nv x 1)
    .def_readonly("dof_M0", &PyMjModel::dof_M0)               // diag. inertia in qpos0                   (nv x 1)

    .def_readonly("geom_type", &PyMjModel::geom_type,          R"pbdoc( geometric type (mjtGeom)                 (ngeom x 1))pbdoc")
    .def_readonly("geom_contype", &PyMjModel::geom_contype,          R"pbdoc( geom contact type                        (ngeom x 1))pbdoc")
    .def_readonly("geom_conaffinity", &PyMjModel::geom_conaffinity,          R"pbdoc( geom contact affinity                    (ngeom x 1))pbdoc")
    .def_readonly("geom_condim", &PyMjModel::geom_condim,          R"pbdoc( contact dimensionality (1, 3, 4, 6)      (ngeom x 1))pbdoc")
    .def_readonly("geom_bodyid", &PyMjModel::geom_bodyid,          R"pbdoc( id of geom's body                        (ngeom x 1))pbdoc")
    .def_readonly("geom_dataid", &PyMjModel::geom_dataid,          R"pbdoc( id of geom's mesh/hfield (-1: none)      (ngeom x 1))pbdoc")
    .def_readonly("geom_matid", &PyMjModel::geom_matid,          R"pbdoc( material id for rendering                (ngeom x 1))pbdoc")
    .def_readonly("geom_group", &PyMjModel::geom_group,          R"pbdoc( group for visibility                     (ngeom x 1))pbdoc")
    .def_readonly("geom_priority", &PyMjModel::geom_priority,          R"pbdoc( geom contact priority                    (ngeom x 1))pbdoc")
    .def_readonly("geom_sameframe", &PyMjModel::geom_sameframe,          R"pbdoc( same as body frame (1) or iframe (2)     (ngeom x 1))pbdoc")
    .def_readonly("geom_solmix", &PyMjModel::geom_solmix,          R"pbdoc( mixing coef for solref/imp in geom pair  (ngeom x 1))pbdoc")
    .def_readonly("geom_solref", &PyMjModel::geom_solref,          R"pbdoc( constraint solver reference: contact     (ngeom x mjNREF))pbdoc")
    .def_readonly("geom_solimp", &PyMjModel::geom_solimp,          R"pbdoc( constraint solver impedance: contact     (ngeom x mjNIMP))pbdoc")
    .def_readonly("geom_size", &PyMjModel::geom_size,          R"pbdoc( geom-specific size parameters            (ngeom x 3))pbdoc")
    .def_readonly("geom_rbound", &PyMjModel::geom_rbound,          R"pbdoc( radius of bounding sphere                (ngeom x 1))pbdoc")
    .def_readonly("geom_pos", &PyMjModel::geom_pos,          R"pbdoc( local position offset rel. to body       (ngeom x 3))pbdoc")
    .def_readonly("geom_quat", &PyMjModel::geom_quat,          R"pbdoc( local orientation offset rel. to body    (ngeom x 4))pbdoc")
    .def_readonly("geom_friction", &PyMjModel::geom_friction,          R"pbdoc( friction for (slide, spin, roll)         (ngeom x 3))pbdoc")
    .def_readonly("geom_margin", &PyMjModel::geom_margin,          R"pbdoc( detect contact if dist<margin            (ngeom x 1))pbdoc")
    .def_readonly("geom_gap", &PyMjModel::geom_gap,          R"pbdoc( include in solver if dist<margin-gap     (ngeom x 1))pbdoc")
    .def_readonly("geom_user", &PyMjModel::geom_user,          R"pbdoc( user data                                (ngeom x nuser_geom))pbdoc")
    .def_readonly("geom_rgba", &PyMjModel::geom_rgba,          R"pbdoc( rgba when material is omitted            (ngeom x 4))pbdoc")

    .def_readonly("site_type", &PyMjModel::site_type,          R"pbdoc( geom type for rendering (mjtGeom)        (nsite x 1))pbdoc")
    .def_readonly("site_bodyid", &PyMjModel::site_bodyid,          R"pbdoc( id of site's body                        (nsite x 1))pbdoc")
    .def_readonly("site_matid", &PyMjModel::site_matid,          R"pbdoc( material id for rendering                (nsite x 1))pbdoc")
    .def_readonly("site_group", &PyMjModel::site_group,          R"pbdoc( group for visibility                     (nsite x 1))pbdoc")
    .def_readonly("site_sameframe", &PyMjModel::site_sameframe,          R"pbdoc( same as body frame (1) or iframe (2)     (nsite x 1))pbdoc")
    .def_readonly("site_size", &PyMjModel::site_size,          R"pbdoc( geom size for rendering                  (nsite x 3))pbdoc")
    .def_readonly("site_pos", &PyMjModel::site_pos,          R"pbdoc( local position offset rel. to body       (nsite x 3))pbdoc")
    .def_readonly("site_quat", &PyMjModel::site_quat,          R"pbdoc( local orientation offset rel. to body    (nsite x 4))pbdoc")
    .def_readonly("site_user", &PyMjModel::site_user,          R"pbdoc( user data                                (nsite x nuser_site))pbdoc")
    .def_readonly("site_rgba", &PyMjModel::site_rgba,          R"pbdoc( rgba when material is omitted            (nsite x 4))pbdoc")
#if 0
    .def_readonly("cam_mode", &PyMjModel::cam_mode,          R"pbdoc( camera tracking mode (mjtCamLight)       (ncam x 1))pbdoc")
    .def_readonly("cam_bodyid", &PyMjModel::cam_bodyid,          R"pbdoc( id of camera's body                      (ncam x 1))pbdoc")
    .def_readonly("cam_targetbodyid", &PyMjModel::cam_targetbodyid,          R"pbdoc( id of targeted body; -1: none            (ncam x 1))pbdoc")
    .def_readonly("cam_pos", &PyMjModel::cam_pos,          R"pbdoc( position rel. to body frame              (ncam x 3))pbdoc")
    .def_readonly("cam_quat", &PyMjModel::cam_quat,          R"pbdoc( orientation rel. to body frame           (ncam x 4))pbdoc")
    .def_readonly("cam_poscom0", &PyMjModel::cam_poscom0,          R"pbdoc( global position rel. to sub-com in qpos0 (ncam x 3))pbdoc")
    .def_readonly("cam_pos0", &PyMjModel::cam_pos0,          R"pbdoc( global position rel. to body in qpos0    (ncam x 3))pbdoc")
    .def_readonly("cam_mat0", &PyMjModel::cam_mat0,          R"pbdoc( global orientation in qpos0              (ncam x 9))pbdoc")
    .def_readonly("cam_fovy", &PyMjModel::cam_fovy,          R"pbdoc( y-field of view (deg)                    (ncam x 1))pbdoc")
    .def_readonly("cam_ipd", &PyMjModel::cam_ipd,          R"pbdoc( inter-pupilary distance                  (ncam x 1))pbdoc")
    .def_readonly("cam_user", &PyMjModel::cam_user,          R"pbdoc( user data                                (ncam x nuser_cam))pbdoc")
    .def_readonly("light_mode", &PyMjModel::light_mode,          R"pbdoc( light tracking mode (mjtCamLight)        (nlight x 1))pbdoc")
    .def_readonly("light_bodyid", &PyMjModel::light_bodyid,          R"pbdoc( id of light's body                       (nlight x 1))pbdoc")
    .def_readonly("light_targetbodyid", &PyMjModel::light_targetbodyid,          R"pbdoc( id of targeted body; -1: none            (nlight x 1))pbdoc")
    .def_readonly("light_directional", &PyMjModel::light_directional,          R"pbdoc( directional light                        (nlight x 1))pbdoc")
    .def_readonly("light_castshadow", &PyMjModel::light_castshadow,          R"pbdoc( does light cast shadows                  (nlight x 1))pbdoc")
    .def_readonly("light_active", &PyMjModel::light_active,          R"pbdoc( is light on                              (nlight x 1))pbdoc")
    .def_readonly("light_pos", &PyMjModel::light_pos,          R"pbdoc( position rel. to body frame              (nlight x 3))pbdoc")
    .def_readonly("light_dir", &PyMjModel::light_dir,          R"pbdoc( direction rel. to body frame             (nlight x 3))pbdoc")
    .def_readonly("light_poscom0", &PyMjModel::light_poscom0,          R"pbdoc( global position rel. to sub-com in qpos0 (nlight x 3))pbdoc")
    .def_readonly("light_pos0", &PyMjModel::light_pos0,          R"pbdoc( global position rel. to body in qpos0    (nlight x 3))pbdoc")
    .def_readonly("light_dir0", &PyMjModel::light_dir0,          R"pbdoc( global direction in qpos0                (nlight x 3))pbdoc")
    .def_readonly("light_attenuation", &PyMjModel::light_attenuation,          R"pbdoc( OpenGL attenuation (quadratic model)     (nlight x 3))pbdoc")
    .def_readonly("light_cutoff", &PyMjModel::light_cutoff,          R"pbdoc( OpenGL cutoff                            (nlight x 1))pbdoc")
    .def_readonly("light_exponent", &PyMjModel::light_exponent,          R"pbdoc( OpenGL exponent                          (nlight x 1))pbdoc")
    .def_readonly("light_ambient", &PyMjModel::light_ambient,          R"pbdoc( ambient rgb (alpha=1)                    (nlight x 3))pbdoc")
    .def_readonly("light_diffuse", &PyMjModel::light_diffuse,          R"pbdoc( diffuse rgb (alpha=1)                    (nlight x 3))pbdoc")
    .def_readonly("light_specular", &PyMjModel::light_specular,          R"pbdoc( specular rgb (alpha=1)                   (nlight x 3))pbdoc")
    .def_readonly("mesh_vertadr", &PyMjModel::mesh_vertadr,          R"pbdoc( first vertex address                     (nmesh x 1))pbdoc")
    .def_readonly("mesh_vertnum", &PyMjModel::mesh_vertnum,          R"pbdoc( number of vertices                       (nmesh x 1))pbdoc")
    .def_readonly("mesh_texcoordadr", &PyMjModel::mesh_texcoordadr,          R"pbdoc( texcoord data address; -1: no texcoord   (nmesh x 1))pbdoc")
    .def_readonly("mesh_faceadr", &PyMjModel::mesh_faceadr,          R"pbdoc( first face address                       (nmesh x 1))pbdoc")
    .def_readonly("mesh_facenum", &PyMjModel::mesh_facenum,          R"pbdoc( number of faces                          (nmesh x 1))pbdoc")
    .def_readonly("mesh_graphadr", &PyMjModel::mesh_graphadr,          R"pbdoc( graph data address; -1: no graph         (nmesh x 1))pbdoc")
    .def_readonly("mesh_vert", &PyMjModel::mesh_vert,          R"pbdoc( vertex positions for all meshe           (nmeshvert x 3))pbdoc")
    .def_readonly("mesh_normal", &PyMjModel::mesh_normal,          R"pbdoc( vertex normals for all meshes            (nmeshvert x 3))pbdoc")
    .def_readonly("mesh_texcoord", &PyMjModel::mesh_texcoord,          R"pbdoc( vertex texcoords for all meshes          (nmeshtexvert x 2))pbdoc")
    .def_readonly("mesh_face", &PyMjModel::mesh_face,          R"pbdoc( triangle face data                       (nmeshface x 3))pbdoc")
    .def_readonly("mesh_graph", &PyMjModel::mesh_graph,          R"pbdoc( convex graph data                        (nmeshgraph x 1))pbdoc")
    .def_readonly("skin_matid", &PyMjModel::skin_matid,          R"pbdoc( skin material id; -1: none               (nskin x 1))pbdoc")
    .def_readonly("skin_rgba", &PyMjModel::skin_rgba,          R"pbdoc( skin rgba                                (nskin x 4))pbdoc")
    .def_readonly("skin_inflate", &PyMjModel::skin_inflate,          R"pbdoc( inflate skin in normal direction         (nskin x 1))pbdoc")
    .def_readonly("skin_vertadr", &PyMjModel::skin_vertadr,          R"pbdoc( first vertex address                     (nskin x 1))pbdoc")
    .def_readonly("skin_vertnum", &PyMjModel::skin_vertnum,          R"pbdoc( number of vertices                       (nskin x 1))pbdoc")
    .def_readonly("skin_texcoordadr", &PyMjModel::skin_texcoordadr,          R"pbdoc( texcoord data address; -1: no texcoord   (nskin x 1))pbdoc")
    .def_readonly("skin_faceadr", &PyMjModel::skin_faceadr,          R"pbdoc( first face address                       (nskin x 1))pbdoc")
    .def_readonly("skin_facenum", &PyMjModel::skin_facenum,          R"pbdoc( number of faces                          (nskin x 1))pbdoc")
    .def_readonly("skin_boneadr", &PyMjModel::skin_boneadr,          R"pbdoc( first bone in skin                       (nskin x 1))pbdoc")
    .def_readonly("skin_bonenum", &PyMjModel::skin_bonenum,          R"pbdoc( number of bones in skin                  (nskin x 1))pbdoc")
    .def_readonly("skin_vert", &PyMjModel::skin_vert,          R"pbdoc( vertex positions for all skin meshes     (nskinvert x 3))pbdoc")
    .def_readonly("skin_texcoord", &PyMjModel::skin_texcoord,          R"pbdoc( vertex texcoords for all skin meshes     (nskintexvert x 2))pbdoc")
    .def_readonly("skin_face", &PyMjModel::skin_face,          R"pbdoc( triangle faces for all skin meshes       (nskinface x 3))pbdoc")
    .def_readonly("skin_bonevertadr", &PyMjModel::skin_bonevertadr,          R"pbdoc( first vertex in each bone                (nskinbone x 1))pbdoc")
    .def_readonly("skin_bonevertnum", &PyMjModel::skin_bonevertnum,          R"pbdoc( number of vertices in each bone          (nskinbone x 1))pbdoc")
    .def_readonly("skin_bonebindpos", &PyMjModel::skin_bonebindpos,          R"pbdoc( bind pos of each bone                    (nskinbone x 3))pbdoc")
    .def_readonly("skin_bonebindquat", &PyMjModel::skin_bonebindquat,          R"pbdoc( bind quat of each bone                   (nskinbone x 4))pbdoc")
    .def_readonly("skin_bonebodyid", &PyMjModel::skin_bonebodyid,          R"pbdoc( body id of each bone                     (nskinbone x 1))pbdoc")
    .def_readonly("skin_bonevertid", &PyMjModel::skin_bonevertid,          R"pbdoc( mesh ids of vertices in each bone        (nskinbonevert x 1))pbdoc")
    .def_readonly("skin_bonevertweight", &PyMjModel::skin_bonevertweight,          R"pbdoc( weights of vertices in each bone         (nskinbonevert x 1))pbdoc")
    .def_readonly("hfield_size", &PyMjModel::hfield_size,          R"pbdoc( (x, y, z_top, z_bottom)                  (nhfield x 4))pbdoc")
    .def_readonly("hfield_nrow", &PyMjModel::hfield_nrow,          R"pbdoc( number of rows in grid                   (nhfield x 1))pbdoc")
    .def_readonly("hfield_ncol", &PyMjModel::hfield_ncol,          R"pbdoc( number of columns in grid                (nhfield x 1))pbdoc")
    .def_readonly("hfield_adr", &PyMjModel::hfield_adr,          R"pbdoc( address in hfield_data                   (nhfield x 1))pbdoc")
    .def_readonly("hfield_data", &PyMjModel::hfield_data,          R"pbdoc( elevation data                           (nhfielddata x 1))pbdoc")
    .def_readonly("tex_type", &PyMjModel::tex_type,          R"pbdoc( texture type (mjtTexture)                (ntex x 1))pbdoc")
    .def_readonly("tex_height", &PyMjModel::tex_height,          R"pbdoc( number of rows in texture image          (ntex x 1))pbdoc")
    .def_readonly("tex_width", &PyMjModel::tex_width,          R"pbdoc( number of columns in texture image       (ntex x 1))pbdoc")
    .def_readonly("tex_adr", &PyMjModel::tex_adr,          R"pbdoc( address in rgb                           (ntex x 1))pbdoc")
    .def_readonly("tex_rgb", &PyMjModel::tex_rgb,          R"pbdoc( rgb (alpha = 1)                          (ntexdata x 1))pbdoc")
    .def_readonly("mat_texid", &PyMjModel::mat_texid,          R"pbdoc( texture id; -1: none                     (nmat x 1))pbdoc")
    .def_readonly("mat_texuniform", &PyMjModel::mat_texuniform,          R"pbdoc( make texture cube uniform                (nmat x 1))pbdoc")
    .def_readonly("mat_texrepeat", &PyMjModel::mat_texrepeat,          R"pbdoc( texture repetition for 2d mapping        (nmat x 2))pbdoc")
    .def_readonly("mat_emission", &PyMjModel::mat_emission,          R"pbdoc( emission (x rgb)                         (nmat x 1))pbdoc")
    .def_readonly("mat_specular", &PyMjModel::mat_specular,          R"pbdoc( specular (x white)                       (nmat x 1))pbdoc")
    .def_readonly("mat_shininess", &PyMjModel::mat_shininess,          R"pbdoc( shininess coef                           (nmat x 1))pbdoc")
    .def_readonly("mat_reflectance", &PyMjModel::mat_reflectance,          R"pbdoc( reflectance (0: disable)                 (nmat x 1))pbdoc")
    .def_readonly("mat_rgba", &PyMjModel::mat_rgba,          R"pbdoc( rgba                                     (nmat x 4))pbdoc")
    .def_readonly("pair_dim", &PyMjModel::pair_dim,          R"pbdoc( contact dimensionality                   (npair x 1))pbdoc")
    .def_readonly("pair_geom1", &PyMjModel::pair_geom1,          R"pbdoc( id of geom1                              (npair x 1))pbdoc")
    .def_readonly("pair_geom2", &PyMjModel::pair_geom2,          R"pbdoc( id of geom2                              (npair x 1))pbdoc")
    .def_readonly("pair_signature", &PyMjModel::pair_signature,          R"pbdoc( (body1+1)<<16 + body2+1                  (npair x 1))pbdoc")
    .def_readonly("pair_solref", &PyMjModel::pair_solref,          R"pbdoc( constraint solver reference: contact     (npair x mjNREF))pbdoc")
    .def_readonly("pair_solimp", &PyMjModel::pair_solimp,          R"pbdoc( constraint solver impedance: contact     (npair x mjNIMP))pbdoc")
    .def_readonly("pair_margin", &PyMjModel::pair_margin,          R"pbdoc( detect contact if dist<margin            (npair x 1))pbdoc")
    .def_readonly("pair_gap", &PyMjModel::pair_gap,          R"pbdoc( include in solver if dist<margin-gap     (npair x 1))pbdoc")
    .def_readonly("pair_friction", &PyMjModel::pair_friction,          R"pbdoc( tangent1, 2, spin, roll1, 2              (npair x 5))pbdoc")
    .def_readonly("exclude_signature", &PyMjModel::exclude_signature,          R"pbdoc( (body1+1)<<16 + body2+1                  (nexclude x 1))pbdoc")
    .def_readonly("eq_type", &PyMjModel::eq_type,          R"pbdoc( constraint type (mjtEq)                  (neq x 1))pbdoc")
    .def_readonly("eq_obj1id", &PyMjModel::eq_obj1id,          R"pbdoc( id of object 1                           (neq x 1))pbdoc")
    .def_readonly("eq_obj2id", &PyMjModel::eq_obj2id,          R"pbdoc( id of object 2                           (neq x 1))pbdoc")
    .def_readonly("eq_active", &PyMjModel::eq_active,          R"pbdoc( enable/disable constraint                (neq x 1))pbdoc")
    .def_readonly("eq_solref", &PyMjModel::eq_solref,          R"pbdoc( constraint solver reference              (neq x mjNREF))pbdoc")
    .def_readonly("eq_solimp", &PyMjModel::eq_solimp,          R"pbdoc( constraint solver impedance              (neq x mjNIMP))pbdoc")
    .def_readonly("eq_data", &PyMjModel::eq_data,          R"pbdoc( numeric data for constraint              (neq x mjNEQDATA))pbdoc")
    .def_readonly("tendon_adr", &PyMjModel::tendon_adr,          R"pbdoc( address of first object in tendon's path (ntendon x 1))pbdoc")
    .def_readonly("tendon_num", &PyMjModel::tendon_num,          R"pbdoc( number of objects in tendon's path       (ntendon x 1))pbdoc")
    .def_readonly("tendon_matid", &PyMjModel::tendon_matid,          R"pbdoc( material id for rendering                (ntendon x 1))pbdoc")
    .def_readonly("tendon_group", &PyMjModel::tendon_group,          R"pbdoc( group for visibility                     (ntendon x 1))pbdoc")
    .def_readonly("tendon_limited", &PyMjModel::tendon_limited,          R"pbdoc( does tendon have length limits           (ntendon x 1))pbdoc")
    .def_readonly("tendon_width", &PyMjModel::tendon_width,          R"pbdoc( width for rendering                      (ntendon x 1))pbdoc")
    .def_readonly("tendon_solref_lim", &PyMjModel::tendon_solref_lim,          R"pbdoc( constraint solver reference: limit       (ntendon x mjNREF))pbdoc")
    .def_readonly("tendon_solimp_lim", &PyMjModel::tendon_solimp_lim,          R"pbdoc( constraint solver impedance: limit       (ntendon x mjNIMP))pbdoc")
    .def_readonly("tendon_solref_fri", &PyMjModel::tendon_solref_fri,          R"pbdoc( constraint solver reference: friction    (ntendon x mjNREF))pbdoc")
    .def_readonly("tendon_solimp_fri", &PyMjModel::tendon_solimp_fri,          R"pbdoc( constraint solver impedance: friction    (ntendon x mjNIMP))pbdoc")
    .def_readonly("tendon_range", &PyMjModel::tendon_range,          R"pbdoc( tendon length limits                     (ntendon x 2))pbdoc")
    .def_readonly("tendon_margin", &PyMjModel::tendon_margin,          R"pbdoc( min distance for limit detection         (ntendon x 1))pbdoc")
    .def_readonly("tendon_stiffness", &PyMjModel::tendon_stiffness,          R"pbdoc( stiffness coefficient                    (ntendon x 1))pbdoc")
    .def_readonly("tendon_damping", &PyMjModel::tendon_damping,          R"pbdoc( damping coefficient                      (ntendon x 1))pbdoc")
    .def_readonly("tendon_frictionloss", &PyMjModel::tendon_frictionloss,          R"pbdoc( loss due to friction                     (ntendon x 1))pbdoc")
    .def_readonly("tendon_lengthspring", &PyMjModel::tendon_lengthspring,          R"pbdoc( tendon length in qpos_spring             (ntendon x 1))pbdoc")
    .def_readonly("tendon_length0", &PyMjModel::tendon_length0,          R"pbdoc( tendon length in qpos0                   (ntendon x 1))pbdoc")
    .def_readonly("tendon_invweight0", &PyMjModel::tendon_invweight0,          R"pbdoc( inv. weight in qpos0                     (ntendon x 1))pbdoc")
    .def_readonly("tendon_user", &PyMjModel::tendon_user,          R"pbdoc( user data                                (ntendon x nuser_tendon))pbdoc")
    .def_readonly("tendon_rgba", &PyMjModel::tendon_rgba,          R"pbdoc( rgba when material is omitted            (ntendon x 4))pbdoc")
    .def_readonly("wrap_type", &PyMjModel::wrap_type,          R"pbdoc( wrap object type (mjtWrap)               (nwrap x 1))pbdoc")
    .def_readonly("wrap_objid", &PyMjModel::wrap_objid,          R"pbdoc( object id: geom, site, joint             (nwrap x 1))pbdoc")
    .def_readonly("wrap_prm", &PyMjModel::wrap_prm,          R"pbdoc( divisor, joint coef, or site id          (nwrap x 1))pbdoc")
    .def_readonly("actuator_trntype", &PyMjModel::actuator_trntype,          R"pbdoc( transmission type (mjtTrn)               (nu x 1))pbdoc")
    .def_readonly("actuator_dyntype", &PyMjModel::actuator_dyntype,          R"pbdoc( dynamics type (mjtDyn)                   (nu x 1))pbdoc")
    .def_readonly("actuator_gaintype", &PyMjModel::actuator_gaintype,          R"pbdoc( gain type (mjtGain)                      (nu x 1))pbdoc")
    .def_readonly("actuator_biastype", &PyMjModel::actuator_biastype,          R"pbdoc( bias type (mjtBias)                      (nu x 1))pbdoc")
    .def_readonly("actuator_trnid", &PyMjModel::actuator_trnid,          R"pbdoc( transmission id: joint, tendon, site     (nu x 2))pbdoc")
    .def_readonly("actuator_group", &PyMjModel::actuator_group,          R"pbdoc( group for visibility                     (nu x 1))pbdoc")
    .def_readonly("actuator_ctrllimited", &PyMjModel::actuator_ctrllimited,          R"pbdoc( is control limited                       (nu x 1))pbdoc")
    .def_readonly("actuator_forcelimited", &PyMjModel::actuator_forcelimited,          R"pbdoc( is force limited                         (nu x 1))pbdoc")
    .def_readonly("actuator_dynprm", &PyMjModel::actuator_dynprm,          R"pbdoc( dynamics parameters                      (nu x mjNDYN))pbdoc")
    .def_readonly("actuator_gainprm", &PyMjModel::actuator_gainprm,          R"pbdoc( gain parameters                          (nu x mjNGAIN))pbdoc")
    .def_readonly("actuator_biasprm", &PyMjModel::actuator_biasprm,          R"pbdoc( bias parameters                          (nu x mjNBIAS))pbdoc")
    .def_readonly("actuator_ctrlrange", &PyMjModel::actuator_ctrlrange,          R"pbdoc( range of controls                        (nu x 2))pbdoc")
    .def_readonly("actuator_forcerange", &PyMjModel::actuator_forcerange,          R"pbdoc( range of forces                          (nu x 2))pbdoc")
    .def_readonly("actuator_gear", &PyMjModel::actuator_gear,          R"pbdoc( scale length and transmitted force       (nu x 6))pbdoc")
    .def_readonly("actuator_cranklength", &PyMjModel::actuator_cranklength,          R"pbdoc( crank length for slider-crank            (nu x 1))pbdoc")
    .def_readonly("actuator_acc0", &PyMjModel::actuator_acc0,          R"pbdoc( acceleration from unit force in qpos0    (nu x 1))pbdoc")
    .def_readonly("actuator_length0", &PyMjModel::actuator_length0,          R"pbdoc( actuator length in qpos0                 (nu x 1))pbdoc")
    .def_readonly("actuator_lengthrange", &PyMjModel::actuator_lengthrange,          R"pbdoc( feasible actuator length range           (nu x 2))pbdoc")
    .def_readonly("actuator_user", &PyMjModel::actuator_user,          R"pbdoc( user data                                (nu x nuser_actuator))pbdoc")
    .def_readonly("sensor_type", &PyMjModel::sensor_type,          R"pbdoc( sensor type (mjtSensor)                  (nsensor x 1))pbdoc")
    .def_readonly("sensor_datatype", &PyMjModel::sensor_datatype,          R"pbdoc( numeric data type (mjtDataType)          (nsensor x 1))pbdoc")
    .def_readonly("sensor_needstage", &PyMjModel::sensor_needstage,          R"pbdoc( required compute stage (mjtStage)        (nsensor x 1))pbdoc")
    .def_readonly("sensor_objtype", &PyMjModel::sensor_objtype,          R"pbdoc( type of sensorized object (mjtObj)       (nsensor x 1))pbdoc")
    .def_readonly("sensor_objid", &PyMjModel::sensor_objid,          R"pbdoc( id of sensorized object                  (nsensor x 1))pbdoc")
    .def_readonly("sensor_dim", &PyMjModel::sensor_dim,          R"pbdoc( number of scalar outputs                 (nsensor x 1))pbdoc")
    .def_readonly("sensor_adr", &PyMjModel::sensor_adr,          R"pbdoc( address in sensor array                  (nsensor x 1))pbdoc")
    .def_readonly("sensor_cutoff", &PyMjModel::sensor_cutoff,          R"pbdoc( cutoff for real and positive; 0: ignore  (nsensor x 1))pbdoc")
    .def_readonly("sensor_noise", &PyMjModel::sensor_noise,          R"pbdoc( noise standard deviation                 (nsensor x 1))pbdoc")
    .def_readonly("sensor_user", &PyMjModel::sensor_user,          R"pbdoc( user data                                (nsensor x nuser_sensor))pbdoc")
    .def_readonly("numeric_adr", &PyMjModel::numeric_adr,          R"pbdoc( address of field in numeric_data         (nnumeric x 1))pbdoc")
    .def_readonly("numeric_size", &PyMjModel::numeric_size,          R"pbdoc( size of numeric field                    (nnumeric x 1))pbdoc")
    .def_readonly("numeric_data", &PyMjModel::numeric_data,          R"pbdoc( array of all numeric fields              (nnumericdata x 1))pbdoc")
    .def_readonly("text_adr", &PyMjModel::text_adr,          R"pbdoc( address of text in text_data             (ntext x 1))pbdoc")
    .def_readonly("text_size", &PyMjModel::text_size,          R"pbdoc( size of text field (strlen+1)            (ntext x 1))pbdoc")
    .def_readonly("text_data", &PyMjModel::text_data,          R"pbdoc( array of all text fields (0-terminated)  (ntextdata x 1))pbdoc")
    .def_readonly("tuple_adr", &PyMjModel::tuple_adr,          R"pbdoc( address of text in text_data             (ntuple x 1))pbdoc")
    .def_readonly("tuple_size", &PyMjModel::tuple_size,          R"pbdoc( number of objects in tuple               (ntuple x 1))pbdoc")
    .def_readonly("tuple_objtype", &PyMjModel::tuple_objtype,          R"pbdoc( array of object types in all tuples      (ntupledata x 1))pbdoc")
    .def_readonly("tuple_objid", &PyMjModel::tuple_objid,          R"pbdoc( array of object ids in all tuples        (ntupledata x 1))pbdoc")
    .def_readonly("tuple_objprm", &PyMjModel::tuple_objprm,          R"pbdoc( array of object params in all tuples     (ntupledata x 1))pbdoc")
    .def_readonly("key_time", &PyMjModel::key_time,          R"pbdoc( key time                                 (nkey x 1))pbdoc")
    .def_readonly("key_qpos", &PyMjModel::key_qpos,          R"pbdoc( key position                             (nkey x nq))pbdoc")
    .def_readonly("key_qvel", &PyMjModel::key_qvel,          R"pbdoc( key velocity                             (nkey x nv))pbdoc")
    .def_readonly("key_act", &PyMjModel::key_act,          R"pbdoc( key activation                           (nkey x na))pbdoc")
    .def_readonly("key_mpos", &PyMjModel::key_mpos,          R"pbdoc( key mocap position                       (nkey x 3*nmocap))pbdoc")
    .def_readonly("key_mquat", &PyMjModel::key_mquat,          R"pbdoc( key mocap quaternion                     (nkey x 4*nmocap))pbdoc")
#endif
    // names
    .def_readonly("name_bodyadr", &PyMjModel::name_bodyadr, R"pbdoc( body name pointers                       (nbody x 1))pbdoc")
    .def_readonly("name_jntadr", &PyMjModel::name_jntadr)          // joint name pointers                      (njnt x 1)
    .def_readonly("name_geomadr", &PyMjModel::name_geomadr)         // geom name pointers                       (ngeom x 1)
    .def_readonly("name_siteadr", &PyMjModel::name_siteadr)         // site name pointers                       (nsite x 1)
    .def_readonly("name_camadr", &PyMjModel::name_camadr)          // camera name pointers                     (ncam x 1)
    .def_readonly("name_lightadr", &PyMjModel::name_lightadr)        // light name pointers                      (nlight x 1)
    .def_readonly("name_meshadr", &PyMjModel::name_meshadr)         // mesh name pointers                       (nmesh x 1)
    .def_readonly("name_skinadr", &PyMjModel::name_skinadr)         // skin name pointers                       (nskin x 1)
    .def_readonly("name_hfieldadr", &PyMjModel::name_hfieldadr)       // hfield name pointers                     (nhfield x 1)
    .def_readonly("name_texadr", &PyMjModel::name_texadr)          // texture name pointers                    (ntex x 1)
    .def_readonly("name_matadr", &PyMjModel::name_matadr)          // material name pointers                   (nmat x 1)
    .def_readonly("name_pairadr", &PyMjModel::name_pairadr)         // geom pair name pointers                  (npair x 1)
    .def_readonly("name_excludeadr", &PyMjModel::name_excludeadr)      // exclude name pointers                    (nexclude x 1)
    .def_readonly("name_eqadr", &PyMjModel::name_eqadr)           // equality constraint name pointers        (neq x 1)
    .def_readonly("name_tendonadr", &PyMjModel::name_tendonadr)       // tendon name pointers                     (ntendon x 1)
    .def_readonly("name_actuatoradr", &PyMjModel::name_actuatoradr)     // actuator name pointers                   (nu x 1)
    .def_readonly("name_sensoradr", &PyMjModel::name_sensoradr)       // sensor name pointers                     (nsensor x 1)
    .def_readonly("name_numericadr", &PyMjModel::name_numericadr)      // numeric name pointers                    (nnumeric x 1)
    .def_readonly("name_textadr", &PyMjModel::name_textadr)         // text name pointers                       (ntext x 1)
    .def_readonly("name_tupleadr", &PyMjModel::name_tupleadr)        // tuple name pointers                      (ntuple x 1)
    .def_readonly("name_keyadr", &PyMjModel::name_keyadr)          // keyframe name pointers                   (nkey x 1)

    .def_readonly("names", &PyMjModel::names, R"pbdoc(names of all objects, 0-terminated       (nnames x 1))pbdoc")

      ;


    //////////////////////////////////////////////////////////////////////////////////


    
     py::class_<PyMjContact>(m, "mjContact")
      // contact parameters set by geom-specific collision detector
      .def_readonly("dist", &PyMjContact::dist,                       R"pbdoc( distance between nearest points; neg: penetration)pbdoc")
      .def_readonly("pos", &PyMjContact::pos,                         R"pbdoc( position of contact point: midpoint between geoms)pbdoc")
      .def_readonly("frame", &PyMjContact::frame,                     R"pbdoc( normal is in [0-2])pbdoc")
      // contact parameters set by mj_collideGeoms
      .def_readonly("includemargin", &PyMjContact::includemargin,     R"pbdoc( include if dist<includemargin margin-gap )pbdoc")
      .def_readonly("friction", &PyMjContact::friction,               R"pbdoc( tangent1, 2, spin, roll1, 2)pbdoc")
      .def_readonly("solref", &PyMjContact::solref,                   R"pbdoc( constraint solver reference)pbdoc")
      .def_readonly("solimp", &PyMjContact::solimp,                   R"pbdoc( constraint solver impedance)pbdoc")
       // internal storage used by solver
      .def_readonly("mu", &PyMjContact::mu,                           R"pbdoc( friction of regularized cone, set by mj_makeConstraint)pbdoc")
      .def_readonly("H", &PyMjContact::H,                             R"pbdoc( cone Hessian, set by mj_updateConstraint)pbdoc")
      // contact descriptors set by mj_collideGeoms
      .def_readonly("dim", &PyMjContact::dim,                         R"pbdoc( contact space dimensionality: 1, 3, 4 or 6)pbdoc")
      .def_readonly("geom1", &PyMjContact::geom1,                     R"pbdoc( id of geom 1)pbdoc")
      .def_readonly("geom2", &PyMjContact::geom2,                     R"pbdoc( id of geom 2)pbdoc")

      // flag set by mj_fuseContact or mj_instantianteEquality
      .def_readonly("exclude", &PyMjContact::exclude,                 R"pbdoc( 0: include, 1: in gap, 2: fused, 3: equality, 4: no dofs)pbdoc")
      
      // address computed by mj_instantiateContact
      .def_readonly("efc_address", &PyMjContact::efc_address,         R"pbdoc( address in efc; -1: not included, -2-i: distance constraint i)pbdoc")

;


//////////////////////////////////////////////////////////////////////////////////

//py::class_<mjData>(m, "mjData")
   
    py::class_<PyMjData>(m, "PyMjData")
    .def("add_elem", &PyMjData::add_elem)
    .def("get_contact", &PyMjData::get_contact)
#if 0
// constant sizes
   .def_readwrite("nstack", &mjData::nstack) // number of mjtNums that can fit in stack
   .def_readwrite("nbuffer", &mjData::nbuffer)  // size of main buffer in bytes
   .def_readwrite("pstack", &mjData::pstack) // first available mjtNum address in stack
// memory utilization stats
    .def_readwrite("maxuse_stack", &mjData::maxuse_stack)// maximum stack allocation
    .def_readwrite("maxuse_con", &mjData::maxuse_con)// maximum number of contacts
    .def_readwrite("maxuse_efc", &mjData::maxuse_efc) // maximum number of scalar constraints
#endif
    // diagnostics
    //.def_readwrite("warning", &PyMjData::warning)\
    // .def_readwrite("mjTimerStat", &PyMjData::mjTimerStat)
    // .def_readwrite("mjSolverStat", &PyMjData::mjSolverStat) // solver statistics per iteration
    .def_readwrite("solver_iter", &PyMjData::solver_iter) // number of solver iterations
    //.def_readwrite("solver_nnz", &PyMjData::solver_nnz) // number of non-zeros in Hessian or efc_AR
    //.def_readwrite("solver_fwdinv", &PyMjData::solver_fwdinv)// forward-inverse comparison: qfrc, efc
// variable sizes
    .def_readwrite("ne", &PyMjData::ne)  // number of equality constraints
    .def_readwrite("nf", &PyMjData::nf) // number of friction constraints
    .def_readwrite("nefc", &PyMjData::nefc) // number of constraints
    .def_readonly("ncon", &PyMjData::ncon) // number of detected contacts
// global properties

    .def_readwrite("time", &PyMjData::time_) // simulation time
            #if 0
    .def_readonly("energy", &mjData::energy) // potential, kinetic energy

    

//-------------------------------- end of info header
    // buffers
    //void*     buffer;               // main buffer; all pointers point in it    (nbuffer bytes)
    //.def_readonly("stack;                // stack buffer                             (nstack mjtNums)

    //-------------------------------- main inputs and outputs of the computation
#endif
    // state
    .def_readwrite("qpos", &PyMjData::qpos, R"pbdoc( position                                 (nq x 1))pbdoc")
    .def_readwrite("qvel", &PyMjData::qvel,   R"pbdoc( velocity                                 (nv x 1))pbdoc")

    .def_readwrite("act", &PyMjData::act)   // actuator activation                      (na x 1)
    .def_readwrite("qacc_warmstart", &PyMjData::qacc_warmstart) // acceleration used for warmstart          (nv x 1)
    // control
    .def_readwrite("ctrl", &PyMjData::ctrl)  // control                                  (nu x 1)
    .def_readwrite("qfrc_applied", &PyMjData::qfrc_applied,         R"pbdoc( applied generalized force                (nv x 1))pbdoc")
    .def_readwrite("xfrc_applied", &PyMjData::xfrc_applied,         R"pbdoc( applied Cartesian force/torque           (nbody x 6))pbdoc")
    // dynamics
    .def_readwrite("qacc", &PyMjData::qacc ,                        R"pbdoc( acceleration                             (nv x 1))pbdoc")
    .def_readwrite("act_dot", &PyMjData::act_dot,                     R"pbdoc( time-derivative of actuator activation   (na x 1))pbdoc")
#if 0
// mocap data
    .def_readwrite("mocap_pos", &mjData::mocap_pos)// positions of mocap bodies                (nmocap x 3)
    .def_readwrite("mocap_quat", &mjData::mocap_quat)// orientations of mocap bodies             (nmocap x 4)
     // user data
    //mjtNum*  userdata;              // user data, not touched by engine         (nuserdata x 1)

      // sensors
    .def_readwrite("sensordata", &mjData::sensordata) // sensor data array                        (nsensordata x 1)
#endif
        //-------------------------------- POSITION dependent

    // computed by mj_fwdPosition/mj_kinematics
    .def_readwrite("xpos", &PyMjData::xpos,                     R"pbdoc( Cartesian position of body frame         (nbody x 3))pbdoc")
    .def_readwrite("xquat", &PyMjData::xquat,                     R"pbdoc( Cartesian orientation of body frame      (nbody x 4))pbdoc")
    .def_readwrite("xmat", &PyMjData::xmat,                     R"pbdoc( Cartesian orientation of body frame      (nbody x 9))pbdoc")
    .def_readwrite("xipos", &PyMjData::xipos,                     R"pbdoc( Cartesian position of body com           (nbody x 3))pbdoc")
    .def_readwrite("ximat", &PyMjData::ximat,                     R"pbdoc( Cartesian orientation of body inertia    (nbody x 9))pbdoc")
    .def_readwrite("xanchor", &PyMjData::xanchor,                     R"pbdoc( Cartesian position of joint anchor       (njnt x 3))pbdoc")
    .def_readwrite("xaxis", &PyMjData::xaxis,                     R"pbdoc( Cartesian joint axis                     (njnt x 3))pbdoc")
    .def_readwrite("geom_xpos", &PyMjData::geom_xpos,                     R"pbdoc( Cartesian geom position                  (ngeom x 3))pbdoc")
    .def_readwrite("geom_xmat", &PyMjData::geom_xmat,                     R"pbdoc( Cartesian geom orientation               (ngeom x 9))pbdoc")
    .def_readwrite("site_xpos", &PyMjData::site_xpos,                     R"pbdoc( Cartesian site position                  (nsite x 3))pbdoc")
    .def_readwrite("site_xmat", &PyMjData::site_xmat,                     R"pbdoc( Cartesian site orientation               (nsite x 9))pbdoc")
#if 0
    .def_readwrite("cam_xpos", &PyMjData::cam_xpos)             // Cartesian camera position                (ncam x 3)
    .def_readwrite("cam_xmat", &PyMjData::cam_xmat)             // Cartesian camera orientation             (ncam x 9)
    .def_readwrite("light_xpos", &PyMjData::light_xpos)           // Cartesian light position                 (nlight x 3)
    .def_readwrite("light_xdir", &PyMjData::light_xdir)           // Cartesian light direction                (nlight x 3)
#endif

// computed by mj_fwdPosition/mj_comPos
    .def_readwrite("subtree_com", &PyMjData::subtree_com,                     R"pbdoc( center of mass of each subtree           (nbody x 3))pbdoc")
    .def_readwrite("cdof", &PyMjData::cdof,                     R"pbdoc( com-based motion axis of each dof        (nv x 6))pbdoc")
    .def_readwrite("cinert", &PyMjData::cinert,                     R"pbdoc( com-based body inertia and mass          (nbody x 10))pbdoc")

#if 0

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

#endif
    // computed by mj_fwdPosition/mj_transmission
    .def_readwrite("actuator_length", &PyMjData::actuator_length)      // actuator lengths                         (nu x 1)
    .def_readwrite("actuator_moment", &PyMjData::actuator_moment)      // actuator moments                         (nu x nv)



    // computed by mj_fwdPosition/mj_crb
    .def_readwrite("crb", &PyMjData::crb)                  // com-based composite inertia and mass     (nbody x 10)
    .def_readwrite("qM", &PyMjData::qM)                  // total inertia                            (nM x 1)

    // computed by mj_fwdPosition/mj_factorM
    .def_readwrite("qLD", &PyMjData::qLD)                  // L'*D*L factorization of M                (nM x 1)
    .def_readwrite("qLDiagInv", &PyMjData::qLDiagInv)            // 1/diag(D)                                (nv x 1)
    .def_readwrite("qLDiagSqrtInv", &PyMjData::qLDiagSqrtInv)        // 1/sqrt(diag(D))                          (nv x 1)

    // computed by mj_fwdPosition/mj_collision
    //.def_readwrite("contact", &PyMjData::contact)             // list of all detected contacts            (nconmax x 1)

    
#if 0
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

////////////////////////////////////////////////////////////////////////////

        
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
 m.def("mj_step", &py_mj_step, R"pbdoc( Advance simulation, use control callback to obtain external force and control.)pbdoc");

 
 // Advance simulation in two steps: before external force and control is set by user.

 m.def("mj_step1", &py_mj_step1, R"pbdoc( Advance simulation in two steps: before external force and control is set by user.)pbdoc");

 

// Advance simulation in two steps: after external force and control is set by user.
 m.def("mj_step2", &py_mj_step2, R"pbdoc( Advance simulation in two steps: after external force and control is set by user.)pbdoc");
 

// Forward dynamics: same as mj_step but do not integrate in time.
  m.def("mj_forward", &py_mj_forward, R"pbdoc( Forward dynamics: same as mj_step but do not integrate in time.)pbdoc");

  

// Inverse dynamics: qacc must be set before calling.
   m.def("mj_inverse", &py_mj_inverse, R"pbdoc(  Inverse dynamics: qacc must be set before calling.)pbdoc");

   
// Forward dynamics with skip; skipstage is mjtStage.
   m.def("mj_forwardSkip", &py_mj_forwardSkip, R"pbdoc(  Forward dynamics with skip; skipstage is mjtStage.)pbdoc");

// Inverse dynamics with skip; skipstage is mjtStage.
   m.def("mj_inverseSkip", &py_mj_inverseSkip, R"pbdoc(  Inverse dynamics with skip; skipstage is mjtStage.)pbdoc");


//---------------------- Printing -------------------------------------------------------

  m.def("mj_printModel", &py_mj_printModel, R"pbdoc(  print Model to file.)pbdoc");
  
  
  // Print data to text file.
   m.def("mj_printData", &py_mj_printData, R"pbdoc(  print Data to file.)pbdoc");

   #if 0  
// Print matrix to screen.
   m.def("mju_printMat", &py_mju_printMat);


// Print sparse matrix to screen.
      m.def("mju_printMatSparse", &py_mju_printMatSparse);
#endif


      
//---------------------- Components -----------------------------------------------------

// Run position-dependent computations.
  m.def("mj_fwdPosition", &py_mj_fwdPosition, R"pbdoc( Run position-dependent computations.)pbdoc");


// Run velocity-dependent computations.
    m.def("mj_fwdVelocity", &py_mj_fwdVelocity, R"pbdoc( Run velocity-dependent computations.)pbdoc");


// Compute actuator force qfrc_actuation.
 m.def("mj_fwdActuation", &py_mj_fwdActuation, R"pbdoc( Compute actuator force qfrc_actuation.)pbdoc");

// Add up all non-constraint forces, compute qacc_unc.
 m.def("mj_fwdAcceleration", &py_mj_fwdAcceleration, R"pbdoc( Add up all non-constraint forces, compute qacc_unc.)pbdoc");


// Run selected constraint solver.
  m.def("mj_fwdConstraint", &py_mj_fwdConstraint, R"pbdoc( Run selected constraint solver.)pbdoc");

// Euler integrator, semi-implicit in velocity.
  m.def("mj_Euler", &py_mj_Euler, R"pbdoc(  Euler integrator, semi-implicit in velocity.)pbdoc");
  
#if 0

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

#endif
// Check qpos, reset if any element is too big or nan.
    m.def("mj_checkPos", &py_mj_checkPos, R"pbdoc(  Check qpos, reset if any element is too big or nan.)pbdoc");


// Check qvel, reset if any element is too big or nan.
  m.def("mj_checkVel", &py_mj_checkVel, R"pbdoc(  Check qvel, reset if any element is too big or nan.)pbdoc");

// Check qacc, reset if any element is too big or nan.
  m.def("mj_checkAcc", &py_mj_checkAcc, R"pbdoc(  Check qacc, reset if any element is too big or nan.)pbdoc");


// Run forward kinematics.
    m.def("mj_kinematics", &py_mj_kinematics, R"pbdoc( Run forward kinematics.)pbdoc");

// Map inertias and motion dofs to global frame centered at CoM.
   m.def("mj_comPos", &py_mj_comPos, R"pbdoc(  Map inertias and motion dofs to global frame centered at CoM.)pbdoc");

#if 0

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

#endif
// Run collision detection.
    m.def("mj_collision", &py_mj_collision, R"pbdoc(  Run collision detection.)pbdoc");

#if 0
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


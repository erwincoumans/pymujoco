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

// #include "D:\dev\VisualLeakDetector\include\vld.h"

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
    
    


    PyMjModel(mjModel* m)
        :m_(m)
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
        qpos0 = py::array_t<mjtNum>(size_t(m_->nq), m_->qpos0);
        qpos_spring = py::array_t<mjtNum>(size_t(m_->nq), m_->qpos_spring);
        body_parentid = py::array_t<int>(size_t(m_->nbody), m_->body_parentid);
        body_parentid = py::array_t<int>(size_t(m_->nbody), m_->body_rootid);
        body_weldid = py::array_t<int>(size_t(m_->nbody), m_->body_weldid);
        
        
    }

    explicit operator const mjModel*() const { 
        return m_; 
    }

    virtual ~PyMjModel()
    {
        printf("destructor ~PyMjModel\n");
    }
};


void py_mj_deleteModel(PyMjModel* m)
{
    mj_deleteModel(m->m_);
    delete m;
}


struct PyMjData 
{
    mjData* d_;
    const mjModel* m_;

    py::array_t<float> qpos_;       
    

    PyMjData (mjData* d, const mjModel* m)
        :d_(d),
        m_(m)
    {
        qpos_ = py::array_t<mjtNum>(size_t(m_->nq), d_->qpos);
    }
    virtual ~PyMjData()
    {
        printf("destructor ~PyMjData\n");
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
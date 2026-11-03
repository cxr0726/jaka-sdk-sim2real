#include <mujoco/mujoco.h>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

static void require(bool ok, const char *reason)
{ if (!ok) throw std::runtime_error(reason); }
static bool compatible(const mjModel *m, int a, int b)
{ return (m->geom_contype[a]&m->geom_conaffinity[b]) || (m->geom_contype[b]&m->geom_conaffinity[a]); }
static int active_geom(const mjModel *m, const char *body, int type)
{
    int id=mj_name2id(m,mjOBJ_BODY,body);require(id>=0,"body missing");
    int result=-1;
    for (int g=0;g<m->ngeom;++g)
        if (m->geom_bodyid[g]==id && m->geom_contype[g]==type)
        { require(result<0,"ambiguous contact category");result=g; }
    require(result>=0,"active collision category missing");return result;
}
static void same(const mjtNum *a,const mjtNum *b,int n,const char *name)
{ for(int i=0;i<n;++i) require(std::abs(a[i]-b[i])<1.e-13,name); }

int main(int argc,char **argv)
{
    try
    {
        require(argc==3,"expected legacy and CAD XML paths");
        char error[2048]{};
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> old(mj_loadXML(argv[1],nullptr,error,sizeof(error)),mj_deleteModel);
        require(bool(old),error);
        std::unique_ptr<mjModel,decltype(&mj_deleteModel)> cad(mj_loadXML(argv[2],nullptr,error,sizeof(error)),mj_deleteModel);
        require(bool(cad),error);
        require(old->nq==cad->nq && old->nv==cad->nv && old->nbody==cad->nbody &&
                old->njnt==cad->njnt && old->nu==cad->nu && old->neq==cad->neq,"contact edit changed articulation");
        same(old->body_mass,cad->body_mass,old->nbody,"mass changed");
        same(old->body_inertia,cad->body_inertia,old->nbody*3,"inertia changed");
        same(old->body_pos,cad->body_pos,old->nbody*3,"body pose changed");
        same(old->body_quat,cad->body_quat,old->nbody*4,"body orientation changed");
        same(old->jnt_range,cad->jnt_range,old->njnt*2,"joint limits changed");
        same(old->dof_armature,cad->dof_armature,old->nv,"armature changed");
        same(old->dof_damping,cad->dof_damping,old->nv,"damping changed");
        same(old->dof_frictionloss,cad->dof_frictionloss,old->nv,"frictionloss changed");
        same(old->actuator_gear,cad->actuator_gear,old->nu*6,"transmission changed");
        same(old->eq_data,cad->eq_data,old->neq*mjNEQDATA,"parallel constraints changed");
        const int lf=active_geom(cad.get(),"Left_ankle_roll_Link",2);
        const int rf=active_geom(cad.get(),"Right_ankle_roll_Link",4);
        const int lm=active_geom(cad.get(),"Left_knee_Link",1);
        const int rm=active_geom(cad.get(),"Right_knee_Link",1);
        const int le=active_geom(cad.get(),"Left_knee_Link",8);
        const int re=active_geom(cad.get(),"Right_knee_Link",16);
        const int base=active_geom(cad.get(),"base_link",1);
        active_geom(cad.get(),"Left_ankle_pitch_Link",1);
        active_geom(cad.get(),"Right_ankle_pitch_Link",1);
        require(cad->geom_type[lf]==mjGEOM_MESH && cad->geom_type[rf]==mjGEOM_MESH,"feet are not whole meshes");
        require(compatible(cad.get(),lf,rf),"foot-foot collision disabled");
        require(compatible(cad.get(),lf,lm) && compatible(cad.get(),rf,rm),"own main shank collision disabled");
        require(compatible(cad.get(),lf,rm) && compatible(cad.get(),rf,lm) &&
                compatible(cad.get(),lf,re) && compatible(cad.get(),rf,le),"opposite shank collision disabled");
        require(!compatible(cad.get(),lf,le) && !compatible(cad.get(),rf,re),"internal ankle envelope still collides");
        const int floor=mj_name2id(cad.get(),mjOBJ_GEOM,"floor");require(floor>=0,"floor missing");
        for(int g:{base,lf,rf,lm,rm,le,re}) require(compatible(cad.get(),g,floor),"ground support disabled");
        require(compatible(cad.get(),base,lf)&&compatible(cad.get(),base,rf),"base-foot collision disabled");
        std::cout<<"CAD contact masks, foot geometry and unchanged dynamics passed\n";
        return 0;
    }
    catch(const std::exception &e){std::cerr<<e.what()<<'\n';return 1;}
}

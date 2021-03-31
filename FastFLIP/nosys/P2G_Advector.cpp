#include <zen/zen.h>
#include <zen/MeshObject.h>
#include <zen/VDBGrid.h>
#include <omp.h>
#include "FLIP_vdb.h"
// I finally decide put this P2G_Advector into FastFLIP, to reduce modularizing effort..
//static void Advect(float dt, openvdb::points::PointDataGrid::Ptr m_particles, openvdb::Vec3fGrid::Ptr m_velocity,
//				   openvdb::Vec3fGrid::Ptr m_velocity_after_p2g, float pic_component, int RK_ORDER);
namespace zenbase{
    
    struct P2G_Advector : zen::INode{
        virtual void apply() override {
            auto dt = std::get<float>(get_param("time_step"));
            auto dx = std::get<float>(get_param("dx"));
            auto smoothness = std::get<float>(get_param("pic_smoothness"));
            auto RK_ORDER = std::get<int>(get_param("RK_ORDER"));
            auto particles = get_input("Particles")->as<VDBPointsGrid>();
            auto velocity = get_input("Velocity")->as<VDBFloat3Grid>();
            auto velocity_after_p2g = get_input("PostAdvVelocity")->as<VDBFloat3Grid>();
            FLIP_vdb::Advect(dt, dx, particles->m_grid, velocity->m_grid, velocity_after_p2g->m_grid, smoothness, RK_ORDER);
        }
    };

static int defP2G_Advector = zen::defNodeClass<P2G_Advector>("P2G_Advector",
    { /* inputs: */ {
        "Particles", "Velocity", "PostAdvVelocity",
    }, 
    /* outputs: */ {
    }, 
    /* params: */ {
      {"float", "time_step", "0.04 0.0 none"},
      {"float", "dx", "0.01 0.0 none"},
      {"int", "RK_ORDER", "1 1 4"},
      {"float", "pic_smoothness", "0.02 0.0 1.0"} ,
    }, 
    
    /* category: */ {
    "FLIPSolver",
    }});

}

#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "slplanner.h"

#include "load_parameter.h"
#include "load_corridor.h"
#include "opencv_plot.h"

#define PLOT_LEVEL 1

static double ObsRadius = 0;
static double UAVRadius = 0;

int main( int argc, char **argv )
{
    std::string name = "test_slplanner_demo_by_corridor_with_yaw_node";
    ros::init(argc, argv, name );

    // ros::NodeHandle nh_private;

    ros::NodeHandle nh_private("~");

    std::string benchmarkpath;
    nh_private.getParam("benchmark", benchmarkpath );
    std::string parameterpath;
    nh_private.getParam("parameterpath", parameterpath );
    std::string solutionspath;
    nh_private.getParam("solutionspath", solutionspath );
    std::string mapname;
    nh_private.getParam("mapname", mapname );
    std::string outputpath = solutionspath  + mapname + "_result.yaml";
    std::cout << ANSI_COLOR_YELLOW << name << " is running" ANSI_COLOR_RESET << std::endl;


    std::string instancepath = benchmarkpath + mapname + ".yaml";
    // std::cout << "instancepath: " << instancepath << std::endl;
    InstanceLoader<Vec2i, Vec2f> instance(instancepath);
    if ( !instance.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load instance!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // instance.printInstanceInfo();


    ParameterLoader para(parameterpath);
    if ( !para.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load parameter!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // para.printParameterInfo();


    CopterGeo2d UAVGeoDes( UAVRadius );
    std::vector<Vec2f> obstacles;
    std::vector<Eigen::VectorXd> obstacles_kd;
    obstacles.resize(0);
    obstacles_kd.resize(0);
    MyKDTree obs_kd_tree( 2, obstacles_kd,  false, 10 );
    ObsGeo2d ObsGeoDes( obstacles, ObsRadius, obs_kd_tree );

    GlobalMap2d gmap( instance.dim(), instance.resolution(), instance.origin(), instance.data(),  UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();

    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();

    CopterState2d start( cp.cm, cp.yaw_use, Vec2f( instance.start(0), instance.start(1) ), M_PI / 2 );
    // // CopterState2d start( cp.cm, cp.yaw_use, Vec2f( instance.start(0), instance.start(1) ), 0 );
    CopterState2d goal( cp.cm, cp.yaw_use, Vec2f( instance.goal(0), instance.goal(1) ), 0 );
    // CopterState2d start( cp.cm, cp.yaw_use, Vec2f( 26.4, 0.7), -2.09 );
    // CopterState2d goal( cp.cm, cp.yaw_use, Vec2f( 14.0, 2.0 ), 1.05 );
    // start.printCopterStateInfo("start state info: ");
    // goal.printCopterStateInfo("goal state info: ");

    SLPlanner2d slsolver( gmap, cp, start, goal );

    Solution2d sol;
    Timer one_clock(true);
    bool res  = slsolver.search(sol);
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN "ExpandStateNodeNum: " << slsolver.getExpandStateNodeSet().size() << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN "ExpandStateNum: " << slsolver.getExpandStates().size() << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN "ExpandPrNum: " << slsolver.getExpandPrimitive().size() << ANSI_COLOR_RESET << std::endl;
        sol.printSolutionInfo();
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        return 0;
    }

    Trajectory2d traj(sol.prs);
    OpenCVPlot plot( gmap );
    plot.drawPoints( gmap.getOccupiedSpace(), black );
    plot.drawCircle( start.pos, blue, 5, 2);
    plot.drawCircle( goal.pos, cyan, 5, 2);


    // plot.drawPoints( slsolver.getExpandStateNodeSet(), grey, 3 );
    int plotStateNum = slsolver.getExpandStates().size();
    vec_E<vec_Vec2f> ntrias;
    Vec2f d1(0.3, 0);
    Vec2f d2(0.2, 0);
    for( int i = 0; i < plotStateNum; i++ )
    {
        double yaw = slsolver.getExpandStates()[i].yaw;
        Vec2f p1 = slsolver.getExpandStates()[i].pos;
        Mat2f Ryaw;
        Ryaw << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
        Vec2f p2 = p1 + Ryaw * d1;

        double yaw1 = yaw + cp.max_yaw / 2;
        double yaw2 = yaw - cp.max_yaw / 2;
        Mat2f Ryaw1, Ryaw2;
        Ryaw1 << cos(yaw1), -sin(yaw1), sin(yaw1), cos(yaw1);
        Ryaw2 << cos(yaw2), -sin(yaw2), sin(yaw2), cos(yaw2);
        Vec2f p3 = p1 + Ryaw1 * d2;
        Vec2f p4 = p1 + Ryaw2 * d2;

        vec_Vec2f tria;
        tria.push_back(p1);
        tria.push_back(p2);
        tria.push_back(p3);
        tria.push_back(p2);
        tria.push_back(p4);

        ntrias.push_back(tria);
    }
    plot.drawLineStrip(ntrias, grey, 1);


    // std::cout << "slsolver.getExpandStateNodeSet(): " << slsolver.getExpandStateNodeSet().size() << std::endl;


#if PLOT_LEVEL == 1
    plot.drawTraj(traj, red, 2);

    vec_E<vec_Vec2f> trias;
    const auto ws_yaw = traj.sample(20);
    Vec2f d(0.7, 0);
    for (const auto &w: ws_yaw)
    {
        double yaw = w.yaw;
        double yaw1 = yaw + cp.max_yaw;
        double yaw2 = yaw - cp.max_yaw;
        Mat2f Ryaw1, Ryaw2;
        Ryaw1 << cos(yaw1), -sin(yaw1), sin(yaw1), cos(yaw1);
        Ryaw2 << cos(yaw2), -sin(yaw2), sin(yaw2), cos(yaw2);
        Vec2f p1 = w.pos;
        Vec2f p2 = w.pos + Ryaw1 * d;
        Vec2f p3 = w.pos + Ryaw2 * d;
        Vec2f p4 = (p2 + p3) / 2;

        vec_Vec2f tria;
        tria.push_back(p1);
        tria.push_back(p2);
        tria.push_back(p3);
        tria.push_back(p1);
        tria.push_back(p4);

        trias.push_back(tria);
    }
    plot.drawLineStrip(trias, blue, 1);

    plot.show( name );
    plot.save( name + ".jpg");

#elif PLOT_LEVEL == 2
    plot.drawDynamicTraj( name, traj, red, 2 );

#elif PLOT_LEVEL == 3
    plot.drawDynamicProcess( name, traj, slsolver.getExpandStates(), slsolver.getExpandPrimitive(), slsolver.getExpandedActions(),  magenta,  1);
    plot.save( name + ".jpg");
#endif

    return 0;
}
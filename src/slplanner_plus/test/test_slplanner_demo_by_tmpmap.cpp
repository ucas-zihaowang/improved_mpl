
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
    std::string name = "test_slplanner_demo_by_tmpmap_node";
    ros::init(argc, argv, name );

    // ros::NodeHandle nh_private;

    ros::NodeHandle nh_private("~");

    std::string parameterpath;
    nh_private.getParam("parameterpath", parameterpath );
    std::string solutionspath;
    nh_private.getParam("solutionspath", solutionspath );
    std::string mapname = "tmpmap";
    std::string outputpath = solutionspath  + mapname + "_result.yaml";
    std::cout << ANSI_COLOR_YELLOW << name << " is running" ANSI_COLOR_RESET << std::endl;



    const Vec2f origin(0, 0);
    const Vec2i dim(400, 200);
    const double resolution = 0.05; // 20 * 10
    std::vector<signed char> mapdata(dim(0) * dim(1), 0);


    const Vec2f leftTop(8, 0.5);
    const Vec2f rightbottom(10, 8);
    Vec2i lefttop_grid;
    Vec2i rightbottom_grid;
    for( int i = 0; i < 2; i++ )
    {
        lefttop_grid(i) = std::round((leftTop(i) - origin(i)) / resolution - 0.5);
        rightbottom_grid(i) = std::round((rightbottom(i) - origin(i)) / resolution - 0.5);
    }
    for( int i = lefttop_grid(0); i < rightbottom_grid(0); i++ )
    {
        for( int j = lefttop_grid(1); j < rightbottom_grid(1); j++ )
        {
            mapdata[ j * dim(0) + i ] = 100;
        }
    }



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

    GlobalMap2d gmap( dim, resolution, origin, mapdata,  UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();


    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();


    CopterState2d start( cp.cm, cp.yaw_use, Vec2f(6, 1) );
    CopterState2d goal( cp.cm, cp.yaw_use, Vec2f(13, 8.5) );
    // start.printCopterStateInfo("start state info: ");
    // goal.printCopterStateInfo("goal state info: ");


    SLPlanner2d slsolver( gmap, cp, start, goal );


    Solution2d sol;
    Timer one_clock(true);
    bool res  = slsolver.search( sol );
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN "ExpandStateNodeNum: " << slsolver.getExpandStateNodeSet().size() << ANSI_COLOR_RESET << std::endl;
        std::cout << ANSI_COLOR_GREEN "ExpandStateNum: " << slsolver.getExpandStates().size() << ANSI_COLOR_RESET << std::endl;
//        std::cout << ANSI_COLOR_GREEN "ExpandPrNum: " << slsolver.getExpandPrimitive().size() << ANSI_COLOR_RESET << std::endl;
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
    // plot.drawPoints( slsolver.getExpandStateNodeSet(), grey, 3 );
    plot.drawCircle( start.pos, blue, 5, 2);
    plot.drawCircle( goal.pos, cyan, 5, 2);



#if PLOT_LEVEL == 1
    plot.drawTraj(traj, red, 2);
    plot.show( name );
    plot.save( name + ".jpg");

#elif PLOT_LEVEL == 2
    plot.drawDynamicTraj( name, traj, red, 2 );

#elif PLOT_LEVEL == 3
    plot.drawDynamicProcess( name, traj, slsolver.getExpandStates(), slsolver.getExpandPrimitive(), slsolver.getExpandedActions(),  magenta,  1);
#endif

    return 0;
}
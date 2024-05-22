
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


class StartGoalInstanceLoader
{
public:
    explicit StartGoalInstanceLoader(const std::string &file )
    {
        try
        {
            YAML::Node config = YAML::LoadFile(file);

            for (const auto &node : config["agents"])
            {
                const auto &start = node["start"];
                const auto &goal = node["goal"];

                starts.emplace_back( Vec3f(start[0].as<double>(), start[1].as<double>(), start[2].as<double>()) );
                goals.emplace_back( Vec3f(goal[0].as<double>(), goal[1].as<double>(), goal[2].as<double>()) );
            }

            agents_num = goals.size();

            exist_ = true;
        }
        catch (YAML::ParserException &e)
        {
            exist_ = false;
        }
    }

    bool exist() {
        return exist_;
    }
    int getAgentsNum() {
        return agents_num;
    }
    Vec3f getStarts( int i ) {
        return starts[i];
    }
    Vec3f getGoals( int i ) {
        return goals[i];
    }

private:
    bool exist_;

    std::vector<Vec3f> starts;
    std::vector<Vec3f> goals;
    int agents_num;
};

int main( int argc, char **argv )
{
    std::string name = "batchtest_slplanner_demo_corridor_with_yaw_node";
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
    instancepath = benchmarkpath + "batchinstance.yaml";
    // std::cout << instancepath << std::endl;
    StartGoalInstanceLoader instance2(instancepath);
    if ( !instance2.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load instance2!" ANSI_COLOR_RESET << std::endl;
        return -1;
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


    GlobalMap2d gmap( instance.dim(), instance.resolution(), instance.origin(), instance.data(),  UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();


    CopterParameter2d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();

    std::vector<CopterState2d> starts, goals;
    for( int i = 0; i < instance2.getAgentsNum(); i++ )
    {
        CopterState2d start( cp.cm, cp.yaw_use, instance2.getStarts(i).head<2>(),  instance2.getStarts(i)(2) );
        CopterState2d goal( cp.cm, cp.yaw_use, instance2.getGoals(i).head<2>(),  instance2.getGoals(i)(2) );
        // std::cout << "UAV " << i << ": " << std::endl;
        // start.printCopterStateInfo("start state info: ");
        // goal.printCopterStateInfo("goal state info: ");
        starts.push_back(start);
        goals.push_back(goal);
    }

    int batchsize = goals.size();
    for( int i = 0; i < batchsize; i++ )
    {
        std::cout << ANSI_COLOR_GREEN "UAV " << i << " Start to solve..." ANSI_COLOR_RESET << std::endl;

        SLPlanner2d slsolver( gmap, cp, starts[i], goals[i], false );

        Solution2d sol;
        Timer one_clock(true);
        bool res  = slsolver.search( sol );
        double spend_time = one_clock.Elapsed();
        if ( res )
        {
            std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
            //         std::cout << ANSI_COLOR_GREEN "ExpandStateNodeNum: " << slsolver.getExpandStateNodeSet().size() << ANSI_COLOR_RESET << std::endl;
            //         std::cout << ANSI_COLOR_GREEN "ExpandStateNum: " << slsolver.getExpandStates().size() << ANSI_COLOR_RESET << std::endl;
            // //        std::cout << ANSI_COLOR_GREEN "ExpandPrNum: " << slsolver.getExpandPrimitive().size() << ANSI_COLOR_RESET << std::endl;
            sol.printSolutionInfo();
        }
        else
        {
            std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        }

        std::cout << ANSI_COLOR_GREEN "UAV " << i << " End to solve!!!" ANSI_COLOR_RESET << std::endl;
    }
    return 0;
}
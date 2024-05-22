
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "slplanner.h"

#include "load_parameter.h"
#include "bag_reader.hpp"
#include <planning_ros_msgs/VoxelMap.h>
#include <planning_ros_utils/data_ros_utils.h>
#include <planning_ros_utils/primitive_ros_utils.h>

static double ObsRadius = 0;
static double UAVRadius = 0.1;

int main(int argc, char **argv)
{
    std::string name = "slplanner_voxelmap_node";
    ros::init(argc, argv, name );
    // ros::NodeHandle nh_private;
    ros::NodeHandle nh_private("~");

    ros::Publisher map_pub = nh_private.advertise<planning_ros_msgs::VoxelMap>("voxel_map", 1, true);
    ros::Publisher sg_pub = nh_private.advertise<sensor_msgs::PointCloud>("start_and_goal", 1, true);
    ros::Publisher cloud_pub = nh_private.advertise<sensor_msgs::PointCloud>("cloud", 1, true);
    ros::Publisher prs_pub = nh_private.advertise<planning_ros_msgs::PrimitiveArray>("primitives", 1, true);
    ros::Publisher traj_pub = nh_private.advertise<planning_ros_msgs::Trajectory>("trajectory", 1, true);
    // ros::Publisher refined_traj_pub = nh_private.advertise<planning_ros_msgs::Trajectory>( "trajectory_refined", 1, true);

    std::string benchmarkpath;
    nh_private.getParam("benchmark", benchmarkpath );
    std::string mapname;
    nh_private.getParam("mapname", mapname );
    std::string topicname;
    nh_private.getParam("topicname", topicname );

    std::string parameterpath;
    nh_private.getParam("parameterpath", parameterpath );

    double start_x, start_y, start_z;
    nh_private.param("start_x", start_x, 12.5);
    nh_private.param("start_y", start_y, 1.4);
    nh_private.param("start_z", start_z, 0.0);
    double goal_x, goal_y, goal_z;
    nh_private.param("goal_x", goal_x, 6.4);
    nh_private.param("goal_y", goal_y, 16.6);
    nh_private.param("goal_z", goal_z, 0.0);
    std::cout << ANSI_COLOR_YELLOW << name << " is running" ANSI_COLOR_RESET << std::endl;

    std::string instancepath = benchmarkpath + mapname;
    // std::cout << "instancepath: " << instancepath << std::endl;
    planning_ros_msgs::VoxelMap instance = read_bag<planning_ros_msgs::VoxelMap>(instancepath, topicname, 0).back();
    Vec3f ori(instance.origin.x, instance.origin.y, instance.origin.z);
    Vec3i dim(instance.dim.x, instance.dim.y, instance.dim.z);

    ParameterLoader para(parameterpath);
    if ( !para.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load parameter!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    // para.printParameterInfo();

    CopterGeo3d UAVGeoDes( UAVRadius );
    std::vector<Vec3f> obstacles;
    std::vector<Eigen::VectorXd> obstacles_kd;
    obstacles.resize(0); 
    obstacles_kd.resize(0); 
    MyKDTree obs_kd_tree( 2, obstacles_kd,  false, 10 );
    ObsGeo3d ObsGeoDes( obstacles, ObsRadius, obs_kd_tree );

    GlobalMap3d gmap( dim, instance.resolution, ori, instance.data,  UAVGeoDes, ObsGeoDes );
    // gmap.printGlobalMapInfo();

    CopterParameter3d cp( para.getCM(), para.getYawUse(), para.getU(), para.getUyaw(), para.getSampleNum(), para.getDt() );
    cp.setDynamicPara( para.getMaxVel(), para.getMaxAcc(), para.getMaxJrk(), para.getMaxYaw() );
    cp.setTolPara( para.getTolPos(), para.getTolVel(), para.getTolAcc(), para.getTolYaw() );
    cp.setCostWeightPara( para.getWJ(), para.getWT(), para.getWYaw() );
    cp.setPlannerPara( para.getMapType(), para.getHeurType(), para.getMPPenalty(), para.getYawMPPenalty() );
    cp.InitRelatedVariable();
    // cp.printCopterParameterInfo();

    CopterState3d start( cp.cm, cp.yaw_use, Vec3f(start_x, start_y, start_z) );
    CopterState3d goal( cp.cm, cp.yaw_use, Vec3f(goal_x, goal_y, goal_z) );

    SLPlanner3d slsolver( gmap, cp, start, goal );

    Solution3d sol;
    Timer one_clock(true);
    bool res  = slsolver.search( sol );
    double spend_time = one_clock.Elapsed();
    if ( res )
    {
        std::cout << ANSI_COLOR_GREEN "Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        // sol.printSolutionInfo();
    }
    else
    {
        std::cout << ANSI_COLOR_GREEN "Can Not Find the Solution: Take " <<  spend_time << "ms" ANSI_COLOR_RESET << std::endl;
        // return 0;
    }

    std_msgs::Header header;
    header.frame_id = std::string("voxelmap");
    instance.header = header;
    map_pub.publish(instance);

    sensor_msgs::PointCloud sg_cloud;
    sg_cloud.header = header;
    geometry_msgs::Point32 pt1, pt2;
    pt1.x = start_x, pt1.y = start_y, pt1.z = start_z;
    pt2.x = goal_x, pt2.y = goal_y, pt2.z = goal_z;
    sg_cloud.points.push_back(pt1), sg_cloud.points.push_back(pt2);
    sg_pub.publish(sg_cloud);

    sensor_msgs::PointCloud ps = vec_to_cloud( slsolver.getExpandStateNodeSet() );
    ps.header = header;
    cloud_pub.publish(ps);

    Trajectory3d traj(sol.prs);
    // Publish trajectory as primitives
    planning_ros_msgs::PrimitiveArray prs_msg = toPrimitiveArrayROSMsg( traj.getPrimitives() );
    prs_msg.header = header;
    prs_pub.publish(prs_msg);

    planning_ros_msgs::Trajectory traj_msg = toTrajectoryROSMsg(traj);
    traj_msg.header = header;
    traj_pub.publish(traj_msg);

    ros::spin();
    return 0;
}



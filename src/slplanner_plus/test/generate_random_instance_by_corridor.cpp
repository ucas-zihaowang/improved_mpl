
#include <ros/ros.h>
#include "common.h"
#include "copter_common.h"
#include "globalmap.h"

#include "load_corridor.h"
#include "opencv_plot.h"
#include <random>

static double ObsRadius = 0;
static double UAVRadius = 0.1;

float generateRandomFloat(float min, float max)
{

    std::random_device rd;


    std::mt19937 mt(rd());


    std::uniform_real_distribution<float> dist(min, max);


    return dist(mt);
}

int main( int argc, char **argv )
{
    std::string name = "generate_random_instance_by_corridor_node";
    ros::init(argc, argv, name );

    // ros::NodeHandle nh_private;
    ros::NodeHandle nh_private("~");
    std::string benchmarkpath;
    nh_private.getParam("benchmark", benchmarkpath );
    std::string mapname;
    nh_private.getParam("mapname", mapname );
    std::cout << ANSI_COLOR_YELLOW << name << " is running" ANSI_COLOR_RESET << std::endl;


    std::string instancepath = benchmarkpath + mapname + ".yaml";
    InstanceLoader<Vec2i, Vec2f> instance(instancepath);
    if ( !instance.exist() )
    {
        std::cout << ANSI_COLOR_RED "failed to load instance!" ANSI_COLOR_RESET << std::endl;
        return -1;
    }
    instance.printInstanceInfo();

    int dim_x = std::ceil( instance.dim()(0) * instance.resolution() );
    int dim_y = std::ceil( instance.dim()(1) * instance.resolution() );

    double rad[20];
    for( int i = 0; i < 12; i++ )
    {
        rad[i] = normalize_angle( M_PI * 2 * i / 12 );
    }
    for( int i = 0; i < 8; i++ )
    {
        rad[i+12] = normalize_angle( M_PI * 2 * i / 8 );
    }

    CopterGeo2d UAVGeoDes( UAVRadius );
    std::vector<Vec2f> obstacles;
    std::vector<Eigen::VectorXd> obstacles_kd;
    obstacles.resize(0);
    obstacles_kd.resize(0);
    MyKDTree obs_kd_tree( 2, obstacles_kd,  false, 10);
    ObsGeo2d ObsGeoDes( obstacles, ObsRadius, obs_kd_tree );
    GlobalMap2d gmap( instance.dim(), instance.resolution(), instance.origin(), instance.data(),  UAVGeoDes, ObsGeoDes );
    gmap.printGlobalMapInfo();


    srand( (unsigned)time( NULL ) );
    Vec3f start{Vec3f::Zero()}, goal{Vec3f::Zero()};
    do
    {
        start(0) = std::round ( generateRandomFloat( 0 + instance.origin()(0), dim_x + instance.origin()(0) ) * 10 ) / 10 ;
        start(1) = std::round ( generateRandomFloat( 0 + instance.origin()(1), dim_y + instance.origin()(1) ) * 10 ) / 10 ;
        start(2) = std::round ( rad[ rand() % 20 ] * 100 ) / 100;
    } while ( gmap.isOccupied( gmap.toGrid(Vec2f( start(0), start(1) )) ) );
    do
    {
        goal(0) = std::round ( generateRandomFloat( 0 + instance.origin()(0), dim_x + instance.origin()(0) ) * 10 ) / 10;
        goal(1) = std::round ( generateRandomFloat( 0 + instance.origin()(1), dim_y + instance.origin()(1) ) * 10 ) / 10;
        goal(2) = std::round ( rad[ rand() % 20 ] * 100 ) / 100;
    } while ( gmap.isOccupied( gmap.toGrid(Vec2f( goal(0), goal(1) )) ) );

    std::cout << "Generate Start and Goal" << std::endl;
    std::cout << "start: " << start.transpose() << std::endl;
    std::cout << "goal: " << goal.transpose() << std::endl;
    std::cout << "Finished" << std::endl;


    OpenCVPlot plot( gmap );
    plot.drawPoints( gmap.getOccupiedSpace(), black );
    plot.drawCircle( start.head<2>(), blue, 5, 2);
    plot.drawCircle( goal.head<2>(), cyan, 5, 2);
    plot.show( name );

    return 0;
}
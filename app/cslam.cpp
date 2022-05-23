#include "mapOptimization.h"
#include "graphComm.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cslam_node");

    ROS_INFO("\033[1;32m---->\033[0m Cooperative Mapping Started.");

    ros::NodeHandle nh;

    int idRobot = nh.param<int>("idRobot", 0);
    int nRobots = nh.param<int>("nRobots", 1);
    int baseIdx = nh.param<int>("baseIdx", 10000);
    std::string modality = nh.param<std::string>("modality", "sim");
    std::string base_addr = nh.param<std::string>("base_addr", "192.168.0.");

    // segment_eigen parameters
    n_nearest_neighbours = nh.param<int>("n_nearest_neighbours", 3);
    feature_distance_threshold = nh.param<double>("feature_distance_threshold", 5);
    gc_resolution = nh.param<double>("gc_resolution", 0.3);
    gc_min_cluster_size = nh.param<int>("gc_min_cluster_size", 6);
    segmentation_mode = nh.param<int>("segmentation_mode", 2);

    TypeExperiment typeExperiment;
    if (modality != "sim" && modality != "real" && modality != "bag"){
        std::cerr << "Unknown modality: " << modality << std::endl;
        exit(0);
    } else {
        std::cerr << "Starting cslam in modality: " << modality << std::endl;
        if (modality == "sim")
        typeExperiment = SIM;
        else if (modality == "real")
        typeExperiment = REAL;
        else
        typeExperiment = BAG;
    }

    mapOptimization MO(nh, idRobot, baseIdx);

    //network connection
    GraphComm gc(&MO, idRobot, nRobots, base_addr, typeExperiment);
    gc.init_network(&nh);

    std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    std::thread computeFeatureMatrixThread(&mapOptimization::featureMatrixThread, &MO);

    ros::WallRate rate(20);
    while (ros::ok())
    {
        ros::spinOnce();

        MO.run();

        rate.sleep();
    }

    loopthread.join();
    visualizeMapThread.join();
    computeFeatureMatrixThread.join();

    return 0;
}

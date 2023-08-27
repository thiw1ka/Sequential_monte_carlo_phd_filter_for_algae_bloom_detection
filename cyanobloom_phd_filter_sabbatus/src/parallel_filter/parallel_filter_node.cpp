#include "parallel_filter/parallel_filter.hpp"

int main (int argc, char *argv[]){
    printf("\n [parallel filter node ] Paralle filter node started...\n");
    ros::init(argc, argv, "parallel_filter_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    try
    {
        unique_ptr<bloom_filter::parallel_filter> pf (new bloom_filter::parallel_filter(nh));
        // string folPath = "/home/thiw1ka/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test/";
        // string filename = "initial_ground_truth.txt";
        // shared_ptr<bloom_filter::phdfilter> bf=pf;
        // std::vector<bloom_filter::gaussian_component> test;
        // test.push_back(bloom_filter::gaussian_component(Eigen::Vector2d(-100,-10),1.0));
        // pf->print_list(test);
        // bf->print_list(test);//print used in parallel filter because of the virtual function
        ros::waitForShutdown();
    }
    catch(const std::exception& e)
    {
        std::cout << "[parallel_filter Object] ERROR. Filter crashed. Reason: " <<e.what() <<std::endl;
        std::cerr << e.what() << '\n';
    }
    
    printf("\n [parallel filter node ] Paralle filter node shutting down...\n");


    return 0;
}
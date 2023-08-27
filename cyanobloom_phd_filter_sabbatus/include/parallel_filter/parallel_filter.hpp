#include "cyanob_phd_filter_sabbatus/bloom_phd_filter.hpp"
#include "python2.7/pyconfig.h"
// #include </usr/include/python2.7/pyconfig.h>
// export CPLUS_INCLUDE_PATH="$CPLUS_INCLUDE_PATH:/usr/include/python2.7"
#include <boost/python.hpp>
#include "diagnostics/diagnostics.hpp"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <unordered_map>
#include <condition_variable>
#include <mutex>
#include <future>

#ifndef PARALLEL_FILTER_HPP
#define PARALLEL_FILTER_HPP
namespace bloom_filter{

    struct parallel_filter: public phdfilter {

        // friend class genetic_algorithm;

        parallel_filter ();

        parallel_filter(ros::NodeHandle _n);

        parallel_filter (ros::NodeHandle n, std::vector<std::tuple<std::string, double> > var_list);

        parallel_filter(/*ros::NodeHandle n,*/ std::string name, 
                        std::vector<std::tuple<std::string, double> > var_list,
                        std::promise<int> p); //to tell main process that paralle filter established correctly

        int argv = 0;
        char** argc = {};


        /*static variables*/

        static std::string diagnos_topic_name_; //topic name shoudl be same for all filters
        static std::string initial_particle_list_dir; //original path where it will copy from
        static std::string output_folder;
        std::mutex copyFile_mutex; //lock for static strings
        /**
         * @brief This cv condition define how the node exits. 
         * we want to make sure it is exiting when rosbag is completed. So we can avoid definign no of runs
         * node will wait till cv is notified to call ros::shut_down()
         * 
         */
        static bool is_exiting_true;
        std::mutex cond_var_lock;
        static std::condition_variable exiting_condition;

        std::mutex callback_lock_;
        // bool is_callback_running_ = false;

        /*inform all exiting condition is met*/
        void set_exiting_condition_true();

        /*intialize pyobject env*/
        void init_pyobjects ();

        void setGroundTruthFilePath (std::string path);

        void initStaticVariables();

        friend diagnostics;
        double _noise,_pruneWeight, _AllmeasSize;

        /*variabel list*/
        double wind_speed_,wind_direction_, tempreture_;
        
        string _fname,_folderpath;

        /*python object var*/
        const string _pythonFileName = "time_update_parallel";
        const string  _pythonclassName = "time_update_parallel";// ,_pythonclassName;
        string _TuFilepath;
        
        std::string _submtn,_pubtn; // topic for subcribing and advertising
        PyObject* pmodule;
        unique_ptr<phdfilter> phd_ptr;

        ros::NodeHandle* _nh;

        bool _measurementupdateactivated;

        std::vector<gaussian_component> _Allmesurements;

        ros::Subscriber _sub_SDiagnostics, sub_exit_signal_;
        ros::Publisher _pub, result_pub_;        

        void cb_systemD(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

        //virtual function from the measurement update
        void callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg);

        void Sub_diagnosticNode(const cyanob_phd_filter::diagnostics::ConstPtr& msg){};
 
        void print_list(std::vector<gaussian_component>&);

        void callback_camera_meas2(const sensor_msgs::PointCloud::ConstPtr& msg);

        string time_update(void);

        /*name of the node*/
        std::string name;

        /**
         * @brief information to exit the program. send by the parent which is genetic algo
         * 
         * @param msg 
         * msg.message == "shutdown" will call rosshutdown for this node
         */
        void callback_exit_info(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg);






        ~parallel_filter(void);


        
    };




}

#endif
#include "parallel_filter/parallel_filter.hpp"
using namespace std;
namespace bloom_filter{

    std::string parallel_filter::diagnos_topic_name_ = "/system_Diagnostics";
    std::string parallel_filter::initial_particle_list_dir = "/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/results/ground_truth/";
    std::string parallel_filter::output_folder = "/home/thivanka/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/results/filter_output/";
    // std::mutex parallel_filter::copyFile_mutex;
    // std::mutex parallel_filter::cond_var_lock;
    std::condition_variable parallel_filter::exiting_condition;
    bool parallel_filter::is_exiting_true = false; ///TODO Turn to atomic


    void parallel_filter::initStaticVariables() {

    }


    parallel_filter::parallel_filter() {
        std::printf("[parallel_filter] parallel_filter::parallel_filter() default init.. \n");
        //this makesure the static variables are intitated at the begining
    }

    /*Current Version*/
    parallel_filter::parallel_filter (ros::NodeHandle n, std::vector<std::tuple<std::string, double> > var_list) : _nh(new auto(n)) {
        name = ros::this_node::getNamespace();
        name.erase(0,1); //erasing the / in the first letter
        std::printf("[parallel filter] started. name: %s \n", name.c_str());

        /*getting params*/
        bool is_groundtruth_DIR = _nh->getParam     ("/INITAL_PATICLE_LIST_DIR_",initial_particle_list_dir);
        bool is_result_DIR      = _nh->getParam     ("/RESULT_DIR", output_folder);
        std::printf("[genetic_algorithm] paths. \n GROUND_TRUTH_DIR_(%s),\n RESULT_DIR_(%s)  \n",
                            initial_particle_list_dir.c_str(), output_folder.c_str());

        /*subcribing to the exit signal topic*/
        sub_exit_signal_ = _nh->subscribe("/filter_shutdown_info_topic", 2, &bloom_filter::parallel_filter::callback_exit_info, this);
        std::printf("[parallel_filter] - listning to - /filter_shutdown_info_topic - for exit signal \n");

        /*copying variables*/
        std::printf("[parallel_filter] - varaible Init: ");
        for (auto v : var_list) {
            std::cout << std::get<0>(v) << ": " << std::to_string(std::get<1>(v)) <<", ";
            /*this is done for now. I know it is pethetic coding*/
            if (std::get<0>(v) == "pd") {
                _Pd = std::get<1>(v);
                set_Pd(_Pd);
            }
            // else if (std::get<0>(v) == "ps") {}/*no ps*/ ///TODO PS Implement
            else if (std::get<0>(v) == "wind_heading") {wind_direction_ = std::get<1>(v);}
            else if (std::get<0>(v) == "wind_speed") {wind_speed_ = std::get<1>(v);}
            else if (std::get<0>(v) == "tempreture") {tempreture_ = std::get<1>(v);}
        }
        std::cout << std::endl;

        /*external ros params*/
        std::string group_ns = ros::this_node::getName();
        _nh->param <string>   (group_ns + "/meas_topic_name", _submtn, "topic_not_set");//camera results
        _nh->param <string>   (group_ns +"/diaganostic_topicName", diagnos_topic_name_);//system diagnostic topic name set as static
        _nh->param <string>   (group_ns +"/results_publising_topic_name", _pubtn, "filter_results");//topic name to publish results

        std::printf("[parallel_filter] meas_topic_name(%s) \n", _submtn.c_str());
        std::printf("[parallel_filter] diaganostic_topicName(%s) \n", diagnos_topic_name_.c_str());
        std::printf("[parallel_filter] results_publising_topic_name(%s) \n", _pubtn.c_str());

        ///TODO figure out what this is
        set_prune_weight(); //default is 0.001 lu
        _noise = 15; //noise was 15 in the launchfile
        _measurementupdateactivated = false;

        /*creating a folder to store filter results*/
        _folderpath = output_folder + name;
        readfile recordparam;
        recordparam.createFolder(_folderpath);
        _folderpath = _folderpath + "/";
        std::printf("[parallel_filter] folder created in \n, (%s) \n", _folderpath.c_str());

        /*initial particle list file name*/
        _fname = "initial_ground_truth.txt";
        
        /*copy ground truth file to new folder*/
        {
            /*lock guard since all threads access to same file to copy*/
            std::lock_guard <std::mutex> lg(copyFile_mutex);
            std::string source = initial_particle_list_dir + _fname;
            printf("[parallel_filter] reading groundtruth from: %s", source.c_str());
            bool isCopiesSuccessful = recordparam.copyFile (source, _folderpath);
            if(!isCopiesSuccessful) {
                printf("[parallel_filter] Error copying initial particle lists \n");
                _nh->~NodeHandle();
                ros::requestShutdown();
            }
        }     

       /*This part will save the filter parameters into a textfile*/ 
        std::string filterparams[7] = { "Noise =" + to_string(_noise),
                                        "Wind F =" + to_string(wind_speed_),
                                        "Wind angle =" + to_string(wind_direction_),
                                        "Tempreture =" + to_string(tempreture_),
                                        "GroundTFolder =" + initial_particle_list_dir,
                                        "Pd = " + to_string(_Pd),
                                        "Pruning Weight = " + to_string(_pruneW)
                                      };
        // readfile recordparam;
        recordparam.save_PPHDFilterConst(_folderpath+"___Filter_Parameters___",5,filterparams);
        _isDroneAltitudesound = true;
        _isMUrequested = false;
        init_pyobjects (); //intiating python object 
        _belief.clear();//stores previous estimates which reads via file
        _sub_SDiagnostics = _nh->subscribe (diagnos_topic_name_, 2, &bloom_filter::parallel_filter::callback_sys_diag, this);
        _sub              = _nh->subscribe (_submtn, 100,&bloom_filter::parallel_filter::callback_camera_meas2,this);
        result_pub_       = _nh->advertise <sensor_msgs::PointCloud> (_pubtn,10); //publisher for results.
        
        std::printf("[parallel_filter] constructor successfull name(%s) belief_size(%i) \n", name.c_str(), int(_belief.size()));
        
        /*reporting back that tread is ready*/
        // p.set_value(1);
        // std::printf("[parallel_filter] p.set_value(1) \n");
        /**
         * here cv wait. it first aquire the lock and check the value of the condition.
         * if not set -> it will release the lock for other threads -> reaquire again later and continue until bool == true
         * if set it will exit the wait.
         */
        std::printf("[parallel_filter] (%s) waiting till exit condition set in CV variable \n", name.c_str());
        std::unique_lock <std::mutex> exit_condition_lock(cond_var_lock);
        exiting_condition.wait(exit_condition_lock, []{return is_exiting_true;});
        std::printf("[parallel_filter] (%s) Exit signal received. variable is set. informing node to shutting down\n", name.c_str());
        exit_condition_lock.unlock();
        _nh->~NodeHandle();
        ros::requestShutdown();
        std::printf("[parallel_filter] (%s) ~Nodehandle. requested shutdown() \n", name.c_str());
        // delete _nh; //double corruption
    }
 
    /*OLD ONE NOT USED*/
    /*create filter object with parent nodehandle_name + / + name*/
    parallel_filter::parallel_filter(/*ros::NodeHandle n,*/ std::string name,
                                     std::vector<std::tuple<std::string, double> > var_list,
                                     std::promise<int> p
                                     ) /*: _nh(n, name)*/ {
        std::printf("[parallel_filter] parallel filter initating. name(%s) \n",name.c_str());
        std::printf("[parallel_filter] is ros node started (%i) \n", int(ros::isStarted()));
        /*initiating ros node*/
        int argv = 0;
        char** argc = {};
        ros::M_string remappings = {};
        // ros::init(remappings,"testing");
        ros::init(remappings, name, ros::InitOption::AnonymousName); //meaning I have to shutdown the node from inside.
        ros::NodeHandle n1;

        std::cout << "[parallel_filter] nod name " << ros::this_node::getName() <<std::endl;
        // ros::NodeHandle n(name.c_str());

        // std::printf("[parallel_filter] ros::NodeHandle n(name); \n");
        // ros::AsyncSpinner spinner(0);
        // std::printf("[parallel_filter] ros::AsyncSpinner spinner(0); \n");
        // spinner.start();
        // std::printf("[parallel_filter] spinner.start() \n");

    //     /*copying variables*/
    //     std::printf("[parallel_filter] (%s) - varaible Init: ", name.c_str());
    //     for (auto v : var_list) {
    //         std::cout << std::get<0>(v) << ": " << std::to_string(std::get<1>(v)) <<", ";
    //         /*this is done for now. I know it is pethetic coding*/
    //         if (std::get<0>(v) == "pd") {
    //             _Pd = std::get<1>(v);
    //             set_Pd(_Pd);
    //         }
    //         // else if (std::get<0>(v) == "ps") {}/*no ps*/
    //         else if (std::get<0>(v) == "wind_heading") {wind_direction_ = std::get<1>(v);}
    //         else if (std::get<0>(v) == "wind_speed") {wind_speed_ = std::get<1>(v);}
    //         else if (std::get<0>(v) == "tempreture") {tempreture_ = std::get<1>(v);}
    //     }
    //     std::cout << std::endl;


    //     ///TODO figure out what this is
    //     set_prune_weight(); //default is 0.001 lu
    //     _noise = 15; //noise was 15 in the launchfile

    //     /*creating a folder to store filter results*/
    //     _folderpath = output_folder + name;
    //     readfile recordparam;
    //     recordparam.createFolder(_folderpath);
    //     _folderpath = _folderpath + "/";
    //     std::printf("[parallel_filter] folder created in \n, (%s) \n", _folderpath.c_str());

    //     /*initial groundtruth file name*/
    //     _fname = "initial_ground_truth.txt";
        
    //     /*copy ground truth file to new folder*/
    //     {
    //         /*lock guard since all threads access to same file to copy*/
    //         std::lock_guard <std::mutex> lg(copyFile_mutex);
    //         std::string source = initial_particle_list_dir + _fname;
    //         recordparam.copyFile (source, _folderpath);
    //     }     

    //    /*This part will save the filter parameters into a textfile*/ 
    //     std::string filterparams[7] = { "Noise =" + to_string(_noise),
    //                                     "Wind F =" + to_string(wind_speed_),
    //                                     "Wind angle =" + to_string(wind_direction_),
    //                                     "Tempreture =" + to_string(tempreture_),
    //                                     "GroundTFolder =" + initial_particle_list_dir,
    //                                     "Pd = " + to_string(_Pd),
    //                                     "Pruning Weight = " + to_string(_pruneW)
    //                                   };
    //     // readfile recordparam;
    //     recordparam.save_PPHDFilterConst(_folderpath+"___Filter_Parameters___",5,filterparams);
    //     _isDroneAltitudesound = true;
    //     _isMUrequested = false;
    //     init_pyobjects (); //intiating python object 
    //     _belief.clear();//stores previous estimates which reads via file
    //     _sub_SDiagnostics   =_nh->subscribe (diagnos_topic_name_, 2, &bloom_filter::parallel_filter::callback_sys_diag, this);
    //     std::printf("[parallel_filter] constructor successfull name(%s) belief_size(%i) \n", name.c_str(), int(_belief.size()));
        
        /*reporting back that tread is ready*/
        p.set_value(1);
        std::printf("[parallel_filter] p.set_value(1) \n");
        /**
         * here cv wait. it first aquire the lock and check the value of the condition.
         * if not set -> it will release the lock for other threads -> reaquire again later and continue until bool == true
         * if set it will exit the wait.
         */
        std::unique_lock <std::mutex> exit_condition_lock(cond_var_lock);
        std::printf("[parallel_filter] exit_condition_lock \n");
        exiting_condition.wait(exit_condition_lock, []{return is_exiting_true;});
        std::printf("[parallel_filter] node exiting (%s) \n", name.c_str());
        exit_condition_lock.unlock();
        std::printf("[parallel_filter] exit_condition_lock.unlock(); \n");
        // std::unique_lock <std::mutex> callback_lk(callback_lock_);
        // ros::requestShutdown();
        // ros::shutdown();
        // ros::waitForShutdown();
        // return 0;

        // spinner.stop();
        // _nh->~NodeHandle();
        std::printf("[parallel_filter] end of filter (%s) \n", name.c_str());
    }

    parallel_filter::parallel_filter(ros::NodeHandle n):_nh(new auto(n)) {
        _measurementupdateactivated = false;
        _pruneWeight = 0.1; //Prune weight limit to time update function
        _AllmeasSize = 0; //std::vector size for _allMeasurement that save measurements.
        set_Pd(0.7);// setting the probability of detection. default value is =0.6
        set_prune_weight();//default is 0.001
        string _sysdiagTopicname; 
        string _GroundTruthFolder;
        _nh->param<double>   ("parallelfilter/noise", _noise, 0.0);
        _nh->param<double>   ("parallelfilter/wind", wind_speed_, 0.0);
        _nh->param<double>   ("parallelfilter/wind_angle", wind_direction_, 0.0);//starting north clockwise
        _nh->param<double>   ("parallelfilter/temp", tempreture_, 0.0);
        _nh->param<string>   ("diaganostic_topicName", _sysdiagTopicname, "/system_Diagnostics");//system diagnostic topic name
        _nh->param<string>   ("parallelfilter/file_path", _fname, "initial_ground_truth.txt");
        _nh->param<string>   ("parallelfilter/folder_path", _folderpath, "~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/test/");
        _nh->param<string>   ("meas_topic_name", _submtn, "/bloom_locations_topic");//camera results
        _nh->param<string>   ("parallelfilter/results_topicName", _pubtn, "filter_results");//topic name to publish results
        _nh->param<string>   ("bloomLoc_fpath", _GroundTruthFolder, "Error");//Ground Truth - which blooms were used

       /*This part will save the filter parameters into a textfile*/ 
        string filterparams[5] = {  "Noise =" + to_string(_noise),
                                    "Wind F =" + to_string(wind_speed_),
                                    "Wind angle =" + to_string(wind_direction_),
                                    "Tempreture =" + to_string(tempreture_),
                                    "GroundTFolder =" + _GroundTruthFolder
                                };
        readfile recordparam;
        recordparam.save_PPHDFilterConst(_folderpath+"___Filter_Parameters___",5,filterparams);
        _isDroneAltitudesound=true;
        _isMUrequested=false;

        /*----Python interpriter initiating---------*/
        init_pyobjects ();
        // _pythonFileName = "time_update_parallel";//python file name that contain time update class
        // _pythonclassName = "time_update_parallel";//python class name
        // Py_Initialize();
        // PyErr_Print();
        // cout << "PFCPP->tu initalized" << endl;
        // PyRun_SimpleString(
        //     "import sys \n"
        //     "import os \n"
        //     "sys.path.append(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/cyanobloom_phd_filter_sabbatus/scripts') )\n"
        // );
        // cout << "PFCPP->tu path appended" << endl;
        // PyErr_Print();
        // pmodule = PyImport_Import(PyString_FromString(_pythonFileName.c_str())); //importing timeupdate class
        // if (pmodule == NULL){
        //     cout<<"PFCPP->pyimport failed!!!"<<endl;
        //     PyErr_Print();
        //     exit(-1);
        // }
        // cout<<"PFCPP->tu class successfully imported"<<endl;
        // /*----Python interpriter initiating end---------*/

        _belief.clear();//stores previous estimates which reads via file
        cout<<"PFCPP->Measurement update node ready.. "<<_belief.size()<<endl;
        // bloom_filter::phdfilter* phdptr;

        _sub                =_nh->subscribe (_submtn, 100,&bloom_filter::parallel_filter::callback_camera_meas2,this);
        _sub_SDiagnostics   =_nh->subscribe (_sysdiagTopicname,2,&bloom_filter::parallel_filter::callback_sys_diag,this);
        _pub                =_nh->advertise <sensor_msgs::PointCloud> (_pubtn,10,true); //publisher for results.
        // _sub_dronepose=_n.subscribe(_sub_dpose, 10, &bloom_filter::phdfilter::callback_dronepose, this);
        // _pub_diagnos=_n.advertise<cyanob_phd_filter::diagnostics>("system_Diagnostics",5,true);
        //cout<<"PFCPP->Measurement update end.. "<<_belief.size()<<endl;
    }

    void bloom_filter::parallel_filter::init_pyobjects () {
        /*locking incase there is a data race to create the object*/
        static std::mutex pyObject_lock;
        std::lock_guard <std::mutex> lock_pyobject(pyObject_lock);

        std::printf("[parallel_filter::init_pyobjects] Initiating python object \n");
        /*----Python interpriter initiating---------*/
        // const string _pythonFileName = "time_update_parallel";//python file name that contain time update class
        // const string _pythonclassName = "time_update_parallel";//python class name
        Py_Initialize();
        PyErr_Print();
        cout << "PFCPP->tu initalized" << endl;
        PyRun_SimpleString(
            "import sys \n"
            "import os \n"
            "sys.path.append(os.path.expanduser('~/catkin_ws/src/uri_soft_wip/cyanobloom_sabbatus/cyanobloom_phd_filter_sabbatus/scripts') )\n"
        );
        cout << "PFCPP->tu path appended" << endl;
        PyErr_Print();
        pmodule = PyImport_Import(PyString_FromString(_pythonFileName.c_str())); //importing timeupdate class
        if (pmodule == NULL){
            cout<<"PFCPP->pyimport failed!!!"<<endl;
            PyErr_Print();
            exit(-1);
        }
        cout<<"PFCPP->tu class successfully imported"<<endl;
        /*----Python interpriter initiating end---------*/
    }

    void bloom_filter::parallel_filter::set_exiting_condition_true() {
        std::printf("[set_exiting_condition_true] (%s) set_exiting_condition_true(). aquiring lock \n", name.c_str());
        std::unique_lock <std::mutex> exit_condition_lock(cond_var_lock);
        std::printf("[set_exiting_condition_true] (%s)  lock aquired. setting:-> is_exiting_true = true\n", name.c_str());
        is_exiting_true = true;
        std::printf("[set_exiting_condition_true] (%s)  unlocking and notifying all waiting for CV variable. \n", name.c_str());
        exiting_condition.notify_all();
        exit_condition_lock.unlock();
    }

    void bloom_filter::parallel_filter::callback_camera_meas2(const sensor_msgs::PointCloud::ConstPtr& msg) {
        //cout<<"PFCPP->inside callback camera.. "<<_belief.size()<<endl;
        // std::printf("[parallel_filter::callback_camera_meas2] _isMUrequested(%i), seq(%i), measurement size(%i) \n",
        //                                                         int(_isMUrequested),int(msg->header.seq), int(msg->points.size()));
        if(_isMUrequested == true) {
            std::vector<gaussian_component> * _meas_ptr, pointsInsideImg, * msupdatelistptr;
            _meas_ptr = &_mesurements;
            _meas_ptr->clear();
            this->_msg_filter_results.points.clear();
            // this->_msg_filter_results.points.clear()

            std::vector<int> listwithrandomNumbers;
            /*removing percentage of incoming measurements RANDOMLY*/
            if(msg->points.size() > 4){
                int pointsLength= msg->points.size();
                int percentage=(pointsLength-4)*(_Pd);//because i am removing 1-pd percentage by erasing only this much
                cout<<"[measurements size] - "<<pointsLength<<", [percentage] - "<<percentage<<", [Pd]- "<<1-_Pd <<endl;
                for (int i=4; i<pointsLength; ++i) listwithrandomNumbers.push_back(i); // 1 2 3 4 5 6 7 8 9
                /*Removing RANDOM percentage of camera detected measurements*/
                std::random_shuffle ( listwithrandomNumbers.begin(), listwithrandomNumbers.end() );
                listwithrandomNumbers.erase (listwithrandomNumbers.begin(),listwithrandomNumbers.begin()+percentage); //removing the from begin() -> percentage
                listwithrandomNumbers.shrink_to_fit();
                std::sort(listwithrandomNumbers.begin(), listwithrandomNumbers.end());
            }
            int count = 0,count_rand = 0; //becayse for loop doesnt count
            for (const geometry_msgs::Point32& _iter:msg->points){
                // std::printf("[callback_camera_meas2] count(%i), count_rand(%i) \n",int(count), int(count_rand) );
                /*recording four corners of the image*/
                if (count<4){
                    _imgcornerpts[count]=_iter;
                    // count++;//counter will advance until 4.
                    // cout<<"printing corner , "<<count<<", "<<_iter<<endl;
                }

                else{
                    /*after four corners of the image*/
                    if(count_rand < listwithrandomNumbers.size() && listwithrandomNumbers[count_rand] == count ) {
                        //if index inside random vector(count that is equal to pd percentage) == current index --> remove that measurement
                        // cout<<"[ignored]random index "<<listwithrandomNumbers[count_rand]<<", counter -"<<count<<endl;
                        count_rand++;
                        count++;
                        continue; //here pd percentage of measurements were removed
                        //cout<<"continue doesnt work" <<endl;
                    }
                    /*basically you are putting 1-pd percentage of measurements. NOT REMOVING 1-PD ITS Opposite */
                    // cout<<"random index "<<listwithrandomNumbers[count_rand]<<", counter -"<<count<<endl;
                    _meas_ptr->push_back(gaussian_component{Eigen::Vector2d(_iter.x,_iter.y)});
                    _Allmesurements.push_back(gaussian_component{Eigen::Vector2d(_iter.x,_iter.y)}); //save measurements to the container
                }
                this->_msg_filter_results.points.push_back(_iter); //pushing all points into filter results msg
                count++;//counter will advance until 4.
            }

            //adding details where measurements ends
            this->_msg_filter_results.header.frame_id = "meas_ends="+to_string(count);
            std::printf("[callback_camera_meas2] count(%i) Header all raw points end(%s)\n",count,this->_msg_filter_results.header.frame_id.c_str() );
            /*adding the prior estimates*/
            for(gaussian_component _iter:_belief){
                this->_msg_filter_results.points.push_back(_iter.get_geomtryPt());
                count++;
            }
            //adding details(index) where measurement points ends
            this->_msg_filter_results.header.frame_id+=",prior="+to_string(count);
            std::printf("[callback_camera_meas2] count(%i) where geometry points end(%s)\n",count,this->_msg_filter_results.header.frame_id.c_str() );

            pointsInsideImg.clear();
            extract_relvant_points(pointsInsideImg);//extracting points that are inside the current image
            measurement_update(pointsInsideImg);//do meas update to extracted points and save it 
            
            double c1=count;
            /*adding the posterior estimates to filter result msg*/
            for(gaussian_component _iter:pointsInsideImg){
                this->_msg_filter_results.points.push_back(_iter.get_geomtryPt());
                count++;
            }
            /*adding details where measurements ends*/
            this->_msg_filter_results.header.frame_id+=",post="+to_string(count);
            cout<<"[callback_camera_meas2] number of filter estimates = "<<(count-c1)<<", belief size ="<<_belief.size()<<endl;

            /*publish results to a topic*/
            cout<<"[callback_camera_meas2] camera call back end to publish. haeder: "<< this->_msg_filter_results.header.frame_id<<endl;
            result_pub_.publish(this->_msg_filter_results);
            cout<<"[callback_camera_meas2] camera call back end published"<<endl;
        }
    }

    //virtual function overlaod from filte_measurement_update
    void parallel_filter::callback_sys_diag(const cyanob_phd_filter::diagnostics::ConstPtr& msg) {
        // std::lock_guard<std::mutex> callLock(callback_lock_); // locks when callback is running
        std::printf("[parallel_filter] new systemMsg received. time_update(%s), measurement_update(%s), simulation(%s) \n",
                    msg->time_update.c_str(), msg->measurement_update.c_str(), msg->simulation.c_str());
        cyanob_phd_filter::diagnostics newmsg;
        newmsg = *msg; //taking a copy of the latest msg

        if (msg->time_update == "activate" && _measurementupdateactivated == false) {
            _TuFilepath = time_update(); //call time update functiona and retrive the path
            //cout<<"measurement update msg received. status = "<<msg->measurement_update<<endl;
            cout<<"[parallel_filter] PFCPP->begin measurement update"<<endl;
            _Allmesurements.clear(); //clear the measurements recorder
            if(_AllmeasSize > 0) _Allmesurements.reserve(_AllmeasSize); //_AllmeasSize is the size of last container
            _initial_points_file.reset();
            try{
                _measurementupdateactivated=true; //provide MU to run
                _initial_points_file.reset(new readfile(&_belief,_TuFilepath));
                _isMUrequested=true; //giving permission to do MU
                //newmsg.measurement_update="running";
                // newmsg.simulation="activate";
                // _pub_diagnos.publish(newmsg);
            }
            catch (...){
                cout<<"PFCPP->error catched in call back system diagnose inside paralle_filter.cpp"<<endl;
                newmsg.measurement_update="PFCPP->file error in measurement update";
                //_pub_diagnos.publish(newmsg);
            }
        }

        else if(msg->measurement_update== "stop" && msg->simulation== "complete"){//this should come from the simulation
            
            cout<<"PFCPP->measurement update status = "<<msg->measurement_update<<endl;
            cout<<"PFCPP->received stop msg"<<endl;
            cout<<"PFCPP->Saving MU points to a file"<<endl;
            _isMUrequested=false; //stopping mu

            _initial_points_file->write2file(&_belief); //save results. filename _mu
            /*save all camera measurements as _AllCamRes*/
            _initial_points_file->write2file(&_Allmesurements,true);//save raw measurements provided by camera
            _AllmeasSize=_Allmesurements.size();
            cout<<"PFCPP-> measurments vector size = "<<_AllmeasSize<<endl;
            ///TODO change in write2file -> erase file  before write. Do not append it will cause error in time update when reading the file
            
            /*Modify file path by incrementing one*/
            string fpth (_initial_points_file->get_fpath());
            std::size_t pos = fpth.find_last_of("/");
            fpth.erase (0, pos+1); //erasing system path and have only the name of the file inside the result folder.
            _fname = fpth;//TU will use this file name in next time
            cout << "PFCPP->path-> " << fpth << endl;

            // newmsg.filename=fpth;
            // newmsg.measurement_update="ready";
            // newmsg.bloom_pub="activate";
            // _pub_diagnos.publish(newmsg);
             _measurementupdateactivated = false;
            // is_callback_running_ = false;
        }
    }

    void parallel_filter::print_list(std::vector<gaussian_component>& l) {
        cout<<"PFCPP->printing inside the parallel filter"<<endl;

    }

    string parallel_filter::time_update(void) {
        cout << "[parallel_filter::time_update] starting======================" << endl;
        PyObject* myclass = PyObject_GetAttrString (pmodule, _pythonclassName.c_str());
        if (myclass == NULL) {
            cout << "PFCPP->pyimport failed!!!" << endl;
            PyErr_Print();
            exit(-1);
        }
        PyObject* args = PyTuple_Pack(  7,
                                        PyString_FromString (_folderpath.c_str()),
                                        PyFloat_FromDouble  (_pruneWeight),
                                        PyString_FromString (_fname.c_str()),
                                        PyFloat_FromDouble  (_noise),
                                        PyFloat_FromDouble  (wind_speed_),
                                        PyFloat_FromDouble  (wind_direction_),
                                        PyFloat_FromDouble  (tempreture_)
                                    );
        PyErr_Print();
        //cout<<"PFCPP->tuple"<<endl;
        PyObject* results = PyObject_CallObject (myclass, args);
        PyErr_Print();
        //cout<<"PFCPP->objeect called"<<endl;
        PyObject* filepath = PyObject_CallMethod (results, "get_path", NULL);
        PyErr_Print();
        //cout<<"PFCPP->tu class results requested"<<endl;
        const char* r = PyString_AsString (filepath);
        std::string str(r);
        cout << "PFCPP->filename returned from TU =" << str << endl;

        //cleaning up python objects in memory
        Py_DECREF(filepath);
        PyErr_Print();
        Py_DECREF(results);
        Py_DECREF(args);
        Py_DECREF(myclass);
        PyErr_Print();
        cout << "[parallel_filter::time_update] end==========================" << endl;
        return str;
    }

    void parallel_filter::setGroundTruthFilePath (std::string path) {
        std::printf("[setGroundTruthFolderPath] setting groundtruth path : \n (%s) \n", path.c_str());
        initial_particle_list_dir = path;
    }

    void bloom_filter::parallel_filter::callback_exit_info(const diagnostic_msgs::DiagnosticStatus::ConstPtr& msg) {
        printf("[callback_exit_info] Node(%s), callback_exit_info msg received on the topic. \n", name.c_str());
        if(msg->message == "shutdown"){
            printf("[callback_exit_info] Node(%s) exit signal(%s) received. setting exit condition! \n", name.c_str(), msg->message.c_str());
            set_exiting_condition_true();
        }
    }

    parallel_filter::~parallel_filter(void) {
        cout<<"[~parallel_filter] parallel filter destructor called"<<endl;
        Py_DECREF(pmodule);
        PyErr_Print();
        Py_Finalize();//this will call destructor for TU class
        cout<<"[~parallel_filter] python interpretor closed"<<endl;
    }
}


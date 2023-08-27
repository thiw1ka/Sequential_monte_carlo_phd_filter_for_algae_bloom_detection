#include "blooms_publisher_node/bloom_locations_publisher.hpp"


namespace bloom_filter{

    blooms_creator::blooms_creator (ros::NodeHandle _n) : _npl(_n) {
        cout<<"[bloom_location_pub] bloom point publisher launched.."<<endl;
        string _sysdiagTopicname;
		_n.param<int>    ("cycles",_cycles,5);//number of times filtering is perfomed
        _n.param<string> ("bloomLoc_fpath", _bloomloc_fpath,"~/catkin_ws/src/uri_soft_wip/cyanobloom/cyanob_phd_filter/results/bloom_coordinates_northeast_wind_drift/");
		_n.param<string> ("init_filename", _init_filename,"bloom_coordinates_northeast_wind_drift_with_noise_and_reflection0.txt");
		_n.param<string> ("bloom_topic", _topicname,"bloom_publisher");
        _n.param<string> ("diaganostic_topicName", _sysdiagTopicname, "system_Diagnostics");//system diagnostic topic name
        // _outsidefile = new readfile();
        std::thread t1 (&readfile::setDirectoryPathForBloomsFilesWithDates, &files, _bloomloc_fpath);
        // std::thread t1 = _outsidefile->setDirectoryPathForBloomsFilesWithDates(_bloomloc_fpath);
        // files.setDirectoryPathForBloomsFilesWithDates(_bloomloc_fpath);
        _fcounter = 0;

        _pub = _n.advertise <sensor_msgs::PointCloud> (_topicname, 5, true);        
        _pub_diagnostic = _n.advertise <cyanob_phd_filter::diagnostics> (_sysdiagTopicname, 5, true);
        _sub_diagnostics = _n.subscribe (_sysdiagTopicname, 2, &blooms_creator::callback_diagnostics, this);
        _pc_msg.header.frame_id = "List of blooms locations for gazebo";

        t1.join();
        cout<<"bloom creator launched.."<<endl;
        cout<<"file name ="<< _init_filename<<endl;
        cout<<"folder path = "<<_bloomloc_fpath<<endl;
        cout<<"bloom publisher waiting for activation..."<<endl;
        ///TODO first read the list of file, how many, may be keep them  in the memory
        ///TODO publisher must publish in 1hz
    }

    blooms_creator::blooms_creator (ros::NodeHandle n, 
                                    string _fname,
                                    string _foldPath,
                                    string topname
                                    ) : _npl(n), _topicname(topname) {

        unique_ptr<readfile> newfile(new readfile(&_pc_msg, _foldPath+_fname));
        _pub=_npl.advertise<sensor_msgs::PointCloud>(_topicname,5,true);
        _pc_msg.header.frame_id="List of blooms locations-"+_fname;
        _timer= _npl.createTimer(ros::Duration(1),&blooms_creator::publish_to_topic,this);
        cout<<"blooms file : "<<_foldPath+_fname<<endl;      
    }

    blooms_creator::blooms_creator (ros::NodeHandle n,
                                    std::string _topic,
                                    std::string _fpath, 
                                    double rate 
                                    ) : _npl(n) ,_topicname (_topic) {
        _outsidefile = new readfile(&_pc_msg,_fpath);
        _pub=_npl.advertise<sensor_msgs::PointCloud>(_topicname,5,true);
        _pc_msg.header.frame_id="List of blooms locations for gazebo";
        _timer= _npl.createTimer(ros::Duration(rate),&blooms_creator::publish_to_topic,this);
        cout<<"list of points are publising under the topic name : "<<_topic<<endl;
        cout<<"Publising rate is "<<rate<<endl;       
    }

    sensor_msgs::PointCloud blooms_creator::get_points () {
        return _pc_msg;    
    }

    void blooms_creator::publish_to_topic (const ros::TimerEvent&) {
        _pub.publish(_pc_msg);
    }

    void blooms_creator::callback_diagnostics(const cyanob_phd_filter::diagnostics::ConstPtr& msg){
        ///TODO read each bloom file and publish in the topic 
        //system initiated via receiving req-status. first file publsihed     
        if(msg->bloom_pub == "request_status" && _fcounter == 0) {
            std::printf("[BloomLPub] status(%s) \n", msg->bloom_pub.c_str());
            cyanob_phd_filter::diagnostics newmsg;
            newmsg = *msg; //taking a copy of the latest msg
            _pc_msg = files.get_next_file();
            _timer= _npl.createTimer(ros::Duration(1),&blooms_creator::publish_to_topic,this);
            newmsg.time_update="request_status";
            string fpath = files.getCurrentFilePath();
            newmsg.bloom_pub = "published ("+std::to_string(_fcounter)+") :" + fpath;
            _pub_diagnostic.publish(newmsg);
            _fcounter++;
        }

        /*recurisive state*/
        if (msg->bloom_pub == "activate") {
            std::printf("[BloomLPub] status(%s) \n", msg->bloom_pub.c_str());
            cyanob_phd_filter::diagnostics newmsg;
            newmsg = *msg; //taking a copy of the latest msg
            /*stoping timer and resetting*/
            _timer.stop();
            _pc_msg = files.get_next_file();
            _timer.setPeriod (ros::Duration(1), true);
            _timer.start();
            newmsg.time_update="activate";
            //newmsg.simulation="activate";
            string fpath = files.getCurrentFilePath();
            newmsg.bloom_pub = "published ("+std::to_string(_fcounter)+") :" + fpath;
            _pub_diagnostic.publish(newmsg);
            _fcounter++;
        }
    }


    blooms_creator::~blooms_creator () {
        cout << "bloom creator destructor called..." << endl;
        _timer.~Timer();
    }
}


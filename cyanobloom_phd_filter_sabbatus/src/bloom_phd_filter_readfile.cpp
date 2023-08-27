#include "cyanob_phd_filter_sabbatus/bloom_phd_filter_readfile.hpp"

namespace bloom_filter{

    void readfile::record2file() {
        cout<<"[readfile] Readfile class object constructed"<<endl;
    }
    
    /*Constructor::Initiate using a list*/
    readfile::readfile(std::vector<gaussian_component>* list,std::string path):_filepath(path),_ptr2list(list){
        //std::cin>>*this; //obtaining the outside file name
        //readfile(path);
        validate_file();
        record2list(list);// recording to a gaussian list
    }

    /*Constructor::initiate using direct file path*/
    /*record points to the internal list*/
    readfile::readfile(std::string filename):_filepath(filename){
        validate_file();
        record2list(&_belief_bar_list);// recording to a gaussian list
    }

    /*constructor for pointcloud msg*/
    /*read file path from the command line*/
    readfile::readfile(sensor_msgs::PointCloud* ptcloud) : _pointcld_ptr(ptcloud){
        std::cin >>* this; //obtaining the outside file name
        validate_file();
        record2PC();
    }

    /*file path given in the launch file*/
    /*record to a point cloud*/
    readfile::readfile(sensor_msgs::PointCloud* ptcloud, std::string filename) : 
                                    _pointcld_ptr(ptcloud), _filepath(filename) {
        validate_file();
        record2PC();//record to a point cloud
    }

    void readfile::validate_file () {
            std::cout<<"validating the file path.."<<std::endl;
            this->_file.open(_filepath);
            //checking for error
                if(_file.fail()){
                    std::cerr << "[file opening error] "<<_filepath<<std::endl;
                    std::cerr << "file opening error. Exiting..."<<std::endl;
                    throw 2;
                    std::exit(1);
                }
           std::cout<<"File validated.."<<std::endl;
    }

     template<typename T>
     void readfile::get_data(T* list_ptr) { 
         cout<<"The data type is = "<< typeid(*list_ptr).name() <<endl;
         //cout<<"the type is point cloud = "<<( typeid(sensor_msgs::PointCloud) == typeid(list) )<<endl;
         list_datatype ldtype;
         if(typeid(*list_ptr) == typeid(vector<gaussian_component>)) {
             ldtype = gc;
             list_ptr->clear();
         }
         else if (typeid(*list_ptr) == typeid(sensor_msgs::PointCloud)) {
             ldtype = pc;
             //list_ptr->points.clear();
         }
         else {
             std::cerr << "Unknown data container. check {readfile::get_data function declaration}. exiting.. " << '\n';
             exit(1);
         }
        double x,y,w;
        string line;
        try{
            while(!this->_file.eof()){
                //_file>>x;
                //_file>>y;
                w=1;
                getline(_file,line);
                std::size_t notint =line.find("X");
                if (notint!=std::string::npos) continue;
                cout<< "the line = "<<line<<endl;
                std::string::size_type sz;     // alias of size_t
                try {
                    // if no double read a exception thrown 
                    x = std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() << '\n';
                    continue;
                }
                line.erase(0,sz+1);
                y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    w=std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "No weight value presented " << w << '\n';
                }
                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop
                if(_file.fail()){_file.clear();throw 01;}
                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:
                switch (ldtype)
                {
                    case gc:
                        list_ptr->push_back(gaussian_component(Eigen::Vector2d(x,y),w));
                        break;
                    case pc:
                        //list_ptr->points.push_back(geometry_msgs::Point32(x,y,w));
                        break;
                }
                //std::cout<<"from file x = "<<x<<" , y = "<<y<<std::endl;
            }
        }catch(...){
            std::cerr<<"Error!!! invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }
        //std::cout<<"end of the file. Read points = "<<ptr->size()<<std::endl;
         //return true;
     }
    
    /*record to a gaussian components list/vector*/
    void readfile::record2list(std::vector<gaussian_component>* ptr){
        cout<<"recording to a list inside readfile class"<<endl;
        ptr->clear();
        double x,y,w;
        string line;
        try{
            while(!this->_file.eof()){
                //_file>>x;
                //_file>>y;
                w=1;
                getline(_file,line);
                std::size_t notint =line.find("X");
                if (notint!=std::string::npos) continue;
                //cout<< "the line = "<<line<<endl;
                std::string::size_type sz;     // alias of size_t
                try {
                    // if no double read a exception thrown 
                    x = std::stod (line,&sz); //read number untill a space is found.
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() <<", line: "<< line << '\n';
                    continue;
                }
                line.erase(0,sz+1);//erasing till x value
                y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    w=std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    //std::cerr << "No weight value presented " << w << '\n';
                }
                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop
                if(_file.fail()){_file.clear();throw 01;}
                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:
                ptr->push_back(gaussian_component(Eigen::Vector2d(x,y),w));
                //std::cout<<"from file x = "<<x<<" , y = "<<y<<std::endl;
            }
        }catch(...){
            std::cerr<<"Error!!! invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }
        std::cout<<"end of the file. Read points = "<<ptr->size()<<std::endl;
    }

    /*recording to a point cloud*/
    void readfile::record2PC(){
        //get_data(_pointcld_ptr);
        std::printf("[readfile] record2PC initiated \n");
        _pointcld_ptr -> points.clear();
        geometry_msgs::Point32 point;
        point.z = 1;
        string line;
        try{
            while(!this->_file.eof()){
                //_file>>point.x;
                //_file>>point.y;
                point.z = 1;
                getline(_file, line);

                /*checking for the first line*/
                std::size_t notint = line.find("X");
                bool isThisFirstLine (line.find("date:") != std::string::npos); //checking word date
                if (notint != std::string::npos || isThisFirstLine) {
                    std::printf("[readfile] record2pc -> first line skipped \n");    
                    continue;
                }

                //cout<< "the line = "<<line<<endl;
                std::string::size_type sz;     // alias of size_t
                try {
                    // if no double read a exception thrown 
                    point.x = std::stod (line,&sz);
                }
                catch (const std::invalid_argument& ia) {
                    std::cerr << "skipping line, No double value present " << ia.what() << '\n';
                    continue;
                }
                line.erase(0,sz+1);//erasing till x value
                point.y=std::stod (line,&sz);
                line.erase(0,sz+1);
                try {
                    // if no double read a exception thrown 
                    point.z=std::stod (line,&sz);//weight
                }
                catch (const std::invalid_argument& ia) {
                    //std::cerr << "No weight value presented. weight = " << point.z << '\n';
                }
                if(this->_file.eof()==true) break; //if end of line flag raised it will exit the loop
                if(_file.fail()){_file.clear();throw 01;}
                if ( (_file.rdstate() & std::ifstream::badbit ) != 0 ){throw 0;}
                save_xy:
                _pointcld_ptr->points.push_back(point);
                // std::cout<<"x = "<<point.x<<" , y = "<<point.y<<", z = "<<point.z <<std::endl;
            }
        }
        catch(int i){
            cerr<<"Error!!!!!! "<< i <<endl<<"Invalid character present in the file. All should be type double... "<<endl;
            std::exit(1);
        }
        // cout<<"[readfile] successfully read "<< _pointcld_ptr->points.size() <<" points."<<endl;
    }

    void readfile::write2file (const std::vector<gaussian_component>* list, bool isrecordMeasurement) {
         _file.close();
         string previousfilepath; //this to save original file path when saving all camera measurements
         std::size_t pos = _filepath.find_last_of("-");
        if(isrecordMeasurement==false){ _filepath.insert(pos,"_mu");}
        else{
            previousfilepath=_filepath;
             _filepath.insert(pos,"_AllCamRes");
        }
        // cout<<"file path = "<<_filepath<<endl;
         std::cout<<"Saving filter results.."<<std::endl;
         this->_file.open (_filepath,std::fstream::out | std::fstream::trunc);//all previous content deleted. use app to append.
         auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
         //_file<<"==================================="<<endl;
        if(_file.is_open()){
            cout<<"open successfully"<<endl;
            _file<<"X Y W"<<endl;
            for (int i=0;i<list->size();i++){
                 _file<<list->at(i).mean.x()<<" "<<list->at(i).mean.y()<<" "<< list->at(i)._w<<endl;
                 //insert weight if want
            }
            _file<<"X The results saved at "<<ctime(&t_c )<<endl;
            //_file<<"==================================="<<endl;   
         _file.close();
        if(isrecordMeasurement==true) {
            _filepath=previousfilepath;
        }
         cout<<"[readfileCpp] measurement update results saved successfully........"<<endl;
        }
        else {cout<<"[readfileCpp] file opening error. results not saved..........."<<endl;}
    }

    string readfile::get_fpath() { //return the file path
        return _filepath;
    }

    /*For parallel filter to save constants*/
    void readfile::save_PPHDFilterConst (std::string PathnFilename, int size,string arrayofsentences[]) {
        _filepath=PathnFilename;
         std::cout<<"ReadF->Saving filter parameters.."<<std::endl;
         _file.open(_filepath,std::fstream::out|std::fstream::trunc);//all previous content deleted. use app to append.
         auto t_c = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
         //_file<<"==================================="<<endl;
        if(_file.is_open()) {
            cout<<"ReadF->open successfully"<<endl;
            _file<<"Filer parameters"<<endl;
            _file<<"==================================="<<endl; 
            for (int i=0;i<size;i++){

                 _file<<arrayofsentences[i]<<endl;
            }
            _file<<"X The results saved at "<<ctime(&t_c )<<endl;
            //_file<<"==================================="<<endl;   
         _file.close();
         cout<<"parameters saved successfully........"<<endl;
        }
        else {cout<<"ReadF->file opening error. param not saved..........."<<endl;}
    }

    void readfile::setDirPath (std::string dir_path) {
        std::printf("[readfile::setDirPath] setting Dir path. \n");
        dir_Path_ = dir_path; //at the end / should be included
        /*checking if path is valid*/
        if(!experimental::filesystem::exists(dir_Path_)){
                std::cerr << "[readfile] Directory path not valid. \npath: "<<dir_Path_<<std::endl;
                std::cerr << "[readfile] file opening error. Exiting..."<<std::endl;
                throw std::invalid_argument("[Directory not existing. May be simulation for all humans inside the population didnt ran.]");
                std::exit(1);
            }
        std::cout<<"[readfile] Directory path valid."<<std::endl;
    }

    int readfile::year_Of_DataSet = 0000;

    std::vector<std::string> readfile::GetListOfAllFilesInDirectory () {
        std::printf("[readfile::GetListOfAllFilesInDirectory] getting all the files to a map container.\n");
        /*checking validity of the file*/
        if(!experimental::filesystem::exists(dir_Path_)) {
            std::cerr << "[readfile] Directory path not valid. \npath: "<<dir_Path_<<std::endl;
            std::cerr << "[readfile] file opening error. Exiting..."<<std::endl;
            throw 2;
            std::exit(1);
        }
        std::vector<std::string> result;
        result.reserve(30);
        /*save paths of all the files in a unordered map*/
        for (auto const & i : experimental::filesystem::directory_iterator{dir_Path_}) {
            std::string path = i.path();
            // auto txt_part = path.substr( path.find_last_of('.')+1, path.back());
            //making sure only txt files are recorded
            if(path.find(".txt")  != std::string::npos) result.push_back(i.path());
            else {std::cout << "[GetListOfAllFilesInDirectory] Not a txt file: " << path.substr(path.find_last_of('/')+1, path.back()) << std::endl; }
        }
        std::printf("[readfile::GetListOfAllFilesInDirectory] completed. there were %i files \n", int(result.size()));
        return std::move(result);
    }
    
    /******************************************/
    /*add filepath based on the file type*/
    void readfile::FileClusterOfEachFilterTimeStep::addToStorage (std::string file_path) {
        // std::printf("[FileClusterOfEachFilterTimeStep::addToStorage] \n");
        auto extracted_file_name = file_path.substr(file_path.find_last_of('/')+1 ,file_path.back());
        // std::printf("[FileClusterOfEachFilterTimeStep::addToStorage] filename(%s) \n", extracted_file_name.c_str());
        /*checking if key exits*/
        // if(list_of_files.find()) here 
        // list_of_files.find()
        if(extracted_file_name.find("AllCamRes")  != std::string::npos) {
            std::printf("[addToStorage] Saved to [AllCamRes] -> filename (%s) \n", extracted_file_name.c_str());
            list_of_files["time_update"] = file_path;
        }
        else if (extracted_file_name.find("mu")   != std::string::npos) {
            std::printf("[addToStorage] Saved to [mu] -> filename (%s) \n", extracted_file_name.c_str());
            list_of_files["meas_update"] = file_path;
        }
        else if (extracted_file_name.find("WOIP") != std::string::npos) {
            std::printf("[addToStorage] Saved to [WOIP] -> filename (%s) \n", extracted_file_name.c_str());
            list_of_files["mu_weight_norm_with_added_noise"] = file_path; }
        else { 
            list_of_files["time_update"] = file_path; 
            std::printf("[addToStorage] Saved to [time_update] -> filename (%s) \n", extracted_file_name.c_str());    
        }
    }

    /*get appropriate file*/
    std::string readfile::FileClusterOfEachFilterTimeStep::getFilePath(std::string key) {
        /*
        list of keys
            "time_update", "meas_update", "all_cam_meas_points",mu_weight_norm_with_added_noise
        */
        try {       
            return list_of_files.at(key);
        }
        catch(const std::exception& e) {
            std::cerr <<"[getFilePath] Error finding key:"<< key << e.what() << '\n';
            std::exit(1);
        }
    }
    /*******************************************/

    /*save in a map*/
    void readfile::getFilesClusteredIntoFilterTimeSteps (std::vector<std::string>& list) {
        /**
         * files shoud be named as filename-1.txt. - should be infront of number and .text should be after that
         * 
         */
        std::printf("[getFilesClusteredIntoFilterTimeSteps] started. \n");
        population_file_cluster_ptr.reset( new std::map <int, readfile::FileClusterOfEachFilterTimeStep >);
        /*extracting filter run index from the file name*/
        for (int i = 0; i < list.size(); i++) {
            // std::cout << "file " << list.at(i) <<std::endl;
            std::string s = list.at(i);
            /*getting the int value from filename. eg: filename-11.txt */
            auto startpos_of_num = s.find_last_of('-'); // finding -
            auto endpos_of_num = s.find_last_of('.'); //finding .
            auto indexLength = s.find_last_of('.') - s.find_last_of('-'); //geting the length of digits
            if (startpos_of_num == std::string::npos || endpos_of_num == std::string::npos || indexLength <= 0) { //if any - or . coudnt find.
                std::printf("[getFilesClusteredIntoFilterTimeSteps] Error finding index from file name. startpos_of_num(%i), endpos_of_num(%i), indexLength(%i), file name (%s). May be this is the init file \n",
                                                                int(startpos_of_num), int(endpos_of_num), int(indexLength), list.at(i).c_str());
                continue; //ignore this file
            }
            int idx = std::stoi(s.substr(s.find_last_of('-')+1, indexLength));
            /*if index is not exist*/
            if(!population_file_cluster_ptr->count(idx)) {
                std::cout << "[getFilesClusteredIntoFilterTimeSteps] cluster map entry created. Index : "<<idx<<std::endl;
                population_file_cluster_ptr->operator[](idx) = FileClusterOfEachFilterTimeStep();
            } 
            population_file_cluster_ptr->at(idx).addToStorage(list.at(i));
        }
        if(population_file_cluster_ptr->empty()) {
            std::printf("[getFilesClusteredIntoFilterTimeSteps] ERROR ZERO file read. May be filter didnot ran fully? zero files saved as filter output.\n");
            // throw std::invalid_argument("[getFilesClusteredIntoFilterTimeSteps] No filter output files exists. exiting");
            // exit(0);
        }
        std::printf("[getFilesClusteredIntoFilterTimeSteps] ended. All files grouped by filter run. saved into clusters\n");
    }

    bool readfile::updateGroundTruthFiles(std::string filepath) {
        std::printf("[updateGroundTruthFiles] reading all ground truth files into a map started\n");
        try {        
            setDirPath (filepath); //setting path
        }
        catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
            return false; //not successful in reading
        }
        auto all_files = GetListOfAllFilesInDirectory (); //getting all files text files
        std::printf("[updateGroundTruthFiles] completed. read (%i) files.\n", int(all_files.size()));
        if(all_files.empty()) {
            std::printf("[updateGroundTruthFiles] Error read zero files. exiting \n");
            return false;
        }
        getFilesClusteredIntoFilterTimeSteps(all_files);
        return true;
    }

    /*reading all files inside the population folder*/
    bool readfile::SetPopulationFiles (std::string folderPath) {
        std::printf("[SetPopulationFiles] setting filter output folderpath, getting all files and grouping by filter run started. path: %s\n", folderPath.c_str());
        try {
            setDirPath (folderPath); //setting path
        }
        catch(const std::exception& e) {
            std::cerr << e.what() << '\n';
            return false; //not successful in reading
        }
        auto all_files = GetListOfAllFilesInDirectory (); //getting all files text files
        if(all_files.empty()) {
            std::printf("[SetPopulationFiles] ERROR read zero files.\n");
            return false;
        }
        getFilesClusteredIntoFilterTimeSteps(all_files); //group files based on the filter time step
        if(population_file_cluster_ptr->empty()) {
            std::printf("[getFilesClusteredIntoFilterTimeSteps] ERROR ZERO file read. May be filter didnot ran fully? zero files saved as filter output.\n");
            return false;
            // throw std::invalid_argument("[getFilesClusteredIntoFilterTimeSteps] No filter output files exists. exiting");
            // exit(0);
        }
        std::printf("[SetPopulationFiles] completed\n");
        return true;
    }

    int readfile::get_filter_timestep_count () {
        std::printf("[readfile::get_filter_timestep_count] counting the filter runs that is clustered. count(%i)\n", int(population_file_cluster_ptr->size()));
        return population_file_cluster_ptr->size();
    }

    sensor_msgs::PointCloud readfile::get_population_next_file (std::string filekey, int filter_timestep) {
        // "time_update", "meas_update", "all_cam_meas_points", "mu_weight_norm_with_added_noise", for groundtruth use ->"time_update"
        std::printf("[get_population_next_file] getting file. file_key(%s), filter_timestep(%i)", filekey.c_str(), filter_timestep);
        auto filepath = population_file_cluster_ptr->at(filter_timestep).getFilePath(filekey);
        std::printf(", filepath (%s) \n", filepath.c_str());
        _filepath = filepath;
        validate_file ();
        _pointcld_ptr = &_pc_msg;
        string line;
        try {
            while(!this->_file.eof()) {
                getline(_file, line);
                // cout << "[readfile] "<<line << endl;
                break;
            }
        }
        catch(int i) {
            cerr<<"[readfile] Error in readfile::get_next_file() : "<< i <<endl;
            std::exit(1);
        }
        record2PC();
        _file.close();
        _pointcld_ptr -> header.frame_id="file type = "+filekey + ", file path" +_filepath;
        std::printf("[get_population_next_file] %s \n", _pointcld_ptr -> header.frame_id.c_str());
        return *_pointcld_ptr;
    }

    /****************************************************************************************/
    /*this is for drew 2016 data set with dated file*/
    /*set directory path and update list of files*/
    void readfile::setDirectoryPathForBloomsFilesWithDates (std::string dir_path) {
        setDirPath(dir_path);
        // std::thread t1(&readfile::GetListOfDatedFilesInDirectory, this);
        GetListOfDatedFilesInDirectory();
    }

    void readfile::GetListOfDatedFilesInDirectory () {
        //extracting the date and time from path
        std::printf("[readfile] getting list of files inside the directory\n");
        auto date = [](std::string s) {
            //2016-10-31-2300 -->key is 1610312300
            auto datestring = s.substr(s.find_last_of('/')+1,s.back());
            auto n= std::remove(datestring.begin(), datestring.end(), '-');
            datestring.erase(n, datestring.end());
            year_Of_DataSet = std::stoi(datestring.substr(0,4));
            datestring.erase(0,2); //keeping the last two digit of year
            return std::stoi(datestring);
        };
        /*save paths of all the files in a unordered map*/
        for (auto const & i : experimental::filesystem::directory_iterator{dir_Path_}) {
            //2016-10-31-2300 -->key is 1610312300
            auto name = date(i.path()); 
            list_of_files[name] = i.path();
        }
        _pointcld_ptr = &_pc_msg;
        // for (auto i:list_of_files) {
        //     std::printf("key: %i \n", i.first);
        // }
        cout << "[readfile] year_Of_DataSet "<<year_Of_DataSet <<endl;
        cout<< "[readfile] files quantity = " << list_of_files.size() << std::endl;
    }

    sensor_msgs::PointCloud readfile::get_next_file() {
        // static int counter = 0;
        std::printf("[readfile] get_next_file initiated\n");
        static auto it = list_of_files.begin();
        _filepath = it->second;
        ++it;
        ///TODO set end condition
        validate_file ();
        string line;
        try{
            while(!this->_file.eof()){
                getline(_file, line);
                cout << "[readfile] "<<line << endl;
                break;
            }
        }
        catch(int i){
            cerr<<"[readfile] Error in readfile::get_next_file() : "<< i <<endl;
            std::exit(1);
        }
        record2PC();
        _file.close();
        _pointcld_ptr -> header.frame_id="blooms fpath-" +_filepath;
        std::printf("[readfile] %s \n", _pointcld_ptr -> header.frame_id.c_str());
        return *_pointcld_ptr;
    }
    /********************************************************************************************/
    string readfile::getCurrentFilePath () {
        std::printf("[readfile] requested Current file path: %s\n", _filepath.c_str());
        return _filepath;
    }

    void readfile::createFolder (std::string name) {
        std::printf("[readfile::createFolder] creating a folder \n");
        // std::string path = dir_Path_ += name;
        if(std::experimental::filesystem::create_directory(name)){
            std::printf("[readfile::createFolder] folder successfully created \n");
        }
        else {
            std::printf("[readfile::createFolder] folder exists. \n");
            // throw std::invalid_argument("[readfile::createFolder] unsuccessfull in creating folder");;
        }
    }

    bool readfile::copyFile (std::string sourcePath, std::string destinationPath) {
        // std::string sourcePath = "path_to_source/file.txt";
        // std::string destinationPath = "path_to_destination/file.txt";
        std::printf("[readfile::copyFile] file copying \n");
        std::printf("[readfile::copyFile] copy from: (%s) \n copy to: (%s) \n",
                                            sourcePath.c_str(), destinationPath.c_str());
        try {
            //update_existing =Replace the existing file only if it is older than the file being copied 
            std::experimental::filesystem::copy(sourcePath, destinationPath, std::experimental::filesystem::copy_options::update_existing);
            std::cout << "[readfile::copyFile]  File copied successfully." << std::endl;
            return true;
        } catch (const std::experimental::filesystem::filesystem_error& e) {
            std::cout << "[readfile::copyFile]  Error copying file: " << e.what() << std::endl;
            return false;
        }
    }

    readfile::~readfile() {
        cout<<"[readfile] destructor called from readfile..... "<<endl;
        // _belief_bar_list.~vector();
        //if(_pointcld_ptr) delete  _pointcld_ptr;
        //if(_ptr2list) delete _ptr2list;
        if(_file.is_open()) _file.close();
    }
}


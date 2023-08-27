


#include <iostream>
#include <chrono>
#include <ctime>
#include <fstream>
#include <istream>
#include <typeinfo>  
#include "bloom_phd_filter_gaussianMixture.hpp"
#include <dirent.h>
#include <experimental/filesystem>
#include <algorithm>
#include <thread>
#include <mutex>
#include <unordered_map>

#ifndef BLOOM_PHD_FILTER_READFILE_HPP
#define BLOOM_PHD_FILTER_READFILE_HPP
using namespace std;

namespace bloom_filter{

    class readfile {
        
    public:

        std::fstream _file;

        std::string _filepath; //file path name

        sensor_msgs::PointCloud* _pointcld_ptr; //store to point cloud msg

        std::vector<gaussian_component> _belief_bar_list;

        std::vector<gaussian_component>* _ptr2list; //pointer to outside list

        inline friend std::istream& operator>> (std::istream& in, readfile& p);

        void record2list(std::vector<gaussian_component>* ptr);// read data and store 

        virtual void validate_file ();//read the file and check 

        void record2PC();// store to a point cloud

        template<typename T>
        void get_data(T* list_ptr);
        //template<> void get_data(sensor_msgs::PointCloud*);

        enum list_datatype {gc,pc}; //gc gaussian, pc pointcloud msg

        /**
         * @brief this is to contain all files that associated with a single filter run/time update
         * eg: timeupdate.txt, MU.txt, ...
         * key is the name of the file
         * value is the path to the file 
         */
        struct FileClusterOfEachFilterTimeStep {
            /*
                 WOIP      - last mu result ->  weights normalized + added noise
                 just number - Tu points (WOIP + new birth particles) 
                _AllCamRes  -  all camera measurements
                _mu         - filter output. result
            */

            /*Constructor*/
            FileClusterOfEachFilterTimeStep () {};  

            /*add to correct storage*/
            void addToStorage(std::string file_path); 

            std::string getFilePath(std::string key);
            
            private:
                // std::string groundtruth = "";
                std::map<std::string, std::string> list_of_files = { {"time_update", ""},
                                                                    {"meas_update", ""}, 
                                                                    {"all_cam_meas_points", ""},
                                                                    {"mu_weight_norm_with_added_noise", ""},
                                                                    };
        };

        /**
         * @brief update ground truth files for 
         * 
         * @param filepath 
         */
        bool updateGroundTruthFiles(std::string filepath);

        /**
         * @brief set population folder and get all the files arranged into filter runs
         * groud by filter runs eg:[1] will have time update, mu, all measuremnts.
         * @param folderPath 
         */
        bool SetPopulationFiles (std::string folderPath);

        std::unique_ptr< std::map <int, readfile::FileClusterOfEachFilterTimeStep > > population_file_cluster_ptr;

        /*get no of filter trial or cluster count*/
        int get_filter_timestep_count ();

        /**
         * @brief 
         * 
         * @param filekey "time_update", "meas_update", "all_cam_meas_points", "mu_weight_norm_with_added_noise", for groundtruth use ->"time_update";
         * @param filter_timestep 
         * @return sensor_msgs::PointCloud 
         */
        sensor_msgs::PointCloud get_population_next_file (std::string filekey, int filter_timestep);

        /**
         * @brief cluster each file related to a single filter trial.
         * clustered based on the last int value.
         * eg: tu_1.txt, mu_1.txt.... ->map[1] = FileClusterOfEachFilterTimeStep{name,path,name,path}
         * 
         */
        void getFilesClusteredIntoFilterTimeSteps (std::vector<std::string>& list);


        /**
         * @brief Getting all files in the directory to a map
         * 
         */
        std::vector<std::string> GetListOfAllFilesInDirectory ();

        /**
         * @brief Get the List Of Files In Directory object
         * 
         */

        void GetListOfDatedFilesInDirectory ();

        /**
         * @brief Set the Directory Path and update the list of files
         * 
         * @param dir_path 
         */
        void setDirectoryPathForBloomsFilesWithDates (std::string dir_path);

        /*only setting direc path*/
        void setDirPath (std::string dir_path);

        experimental::filesystem::path dir_Path_;

        static int year_Of_DataSet;

        /**
         * @brief save paths for all files inside the dir
         * key - 1610312300 --> YYMMDDHHMM
         * 
         */
        std::map<int, experimental::filesystem::path> list_of_files;

        /**
         * @brief Point cloud msg to save points
         * 
         */
        sensor_msgs::PointCloud _pc_msg;

        /**
         * @brief Get the next file object
         * 
         * @return sensor_msgs::PointCloud* 
         */
        sensor_msgs::PointCloud get_next_file ();

        /**
         * @brief return current file path
         * 
         */
        string getCurrentFilePath ();

        /**
         * @brief Creating a folder in given path;
         * 
         * @param name 
         */
        void createFolder (std::string name);

        /**
         * @brief copy file from one directory to another
         * 
         * @param sourcePath 
         * @param destinationPath
         * @return bool; if not copied return false 
         */
        bool copyFile (std::string sourcePath, std::string destinationPath);

        


        readfile() {}

        /*Initiate using a list and file path*/
        //used in the cyanob filter
        readfile(std::vector<gaussian_component>* list,std::string path);

        /*initiate using direct file path*/
        readfile(std::string filename);

        /*read file path from command line*/
        readfile(sensor_msgs::PointCloud* ptcloud);

        /*file path given in the launch file*/
        readfile(sensor_msgs::PointCloud* ptcloud,std::string filename);

        // readfile(std::string dir_path);


        /*For parallel filter to save constants*/
        void save_PPHDFilterConst (std::string PathnFilename, int size, string arrayofsentences[]);
   
        ~readfile();

        void write2file(const std::vector<gaussian_component>* list,bool isrecordMeasurement=false);// write back to a text file
    
        string get_fpath(); //return the file path

        virtual void record2file();//write data to a file
    };



    //Cin overload
    std::istream& operator>> (std::istream& in, readfile& p){

        std::cout<<"enter file name : ";

        in>>p._filepath;

        return in;

    }
}

    

#endif
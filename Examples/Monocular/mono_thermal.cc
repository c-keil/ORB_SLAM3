/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

/*
This script is used to load termal image datasets
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps,
                vector<string> &vstrKP, vector<string> &vstrDesc);

int main(int argc, char **argv)
{
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_termal path_to_vocabulary path_to_settings path_to_sequence_folder path_to_timestamps_file" << endl;
        return 1;
    }

    const int num_seq = 1;
    string file_name = string(argv[3]);
    string timestamp_file = string(argv[4]);
    cout << "file name: " << file_name << endl;

    // Load all sequences:
    int seq;
    vector<string> strImageFilenames, strKpFilenames, strDescFilenames;
    vector<double> TimestampsCam;

    int nImages;

    int tot_images = 0;
    cout << "Loading images...";
    LoadImages(file_name, timestamp_file, strImageFilenames, TimestampsCam, strKpFilenames, strDescFilenames);
    cout << "LOADED!" << endl;

    nImages = strImageFilenames.size();
    tot_images += nImages;

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout.precision(17);


    int fps = 20;
    float dT = 1.f/fps;
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, false);
    float imageScale = SLAM.GetImageScale();

    double t_resize = 0.f;
    double t_track = 0.f;

    // Main loop
    cv::Mat im;
    int proccIm = 0;
    for(int ni=0; ni<nImages; ni++, proccIm++)
    {

        // Read image from file
        im = cv::imread(strImageFilenames[ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
        double tframe = TimestampsCam[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: "
                    <<  strImageFilenames[ni] << endl;
            return 1;
        }

        if(imageScale != 1.f)
        {
// #ifdef REGISTER_TIMES
// #ifdef COMPILEDWITHC11
//             std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
// #else
//             std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
// #endif
// #endif
            int width = im.cols * imageScale;
            int height = im.rows * imageScale;
            cv::resize(im, im, cv::Size(width, height));
// #ifdef REGISTER_TIMES
// #ifdef COMPILEDWITHC11
//             std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
// #else
//             std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
// #endif
//             t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
//             SLAM.InsertResizeTime(t_resize);
// #endif
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        // cout << "tframe = " << tframe << endl;
        SLAM.TrackMonocular(im,tframe); 

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
        t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
        SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = TimestampsCam[ni+1]-tframe;
        else if(ni>0)
            T = tframe-TimestampsCam[ni-1];

        //std::cout << "T: " << T << std::endl;
        //std::cout << "ttrack: " << ttrack << std::endl;

        if(ttrack<T) {
            //std::cout << "usleep: " << (dT-ttrack) << std::endl;
            usleep((T-ttrack)*1e6); // 1e6
        }
    }

    string kf_file_submap =  "./SubMaps/kf_SubMap_" + std::to_string(seq) + ".txt";
    string f_file_submap =  "./SubMaps/f_SubMap_" + std::to_string(seq) + ".txt";
    SLAM.SaveTrajectoryEuRoC(f_file_submap);
    SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file_submap);

    // cout << "Changing the dataset" << endl;

    // SLAM.ChangeDataset();

    // Stop all threads
    SLAM.Shutdown();

    // // /ave camera trajectory
    // if (bFileName)
    // {
    //     const string kf_file =  "kf_" + string(argv[argc-1]) + ".txt";
    //     const string f_file =  "f_" + string(argv[argc-1]) + ".txt";
    //     SLAM.SaveTrajectoryEuRoC(f_file);
    //     SLAM.SaveKeyFrameTrajectoryEuRoC(kf_file);
    // }
    // else
    // {
    SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
    // }

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps,
                vector<string> &vstrKP, vector<string> &vstrDesc)
{
    cout << "Reading times/file names from: '" << strPathTimes << "'" << endl;
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            vstrKP.push_back(strImagePath + "/keypoints/" + ss.str() + "_kp.txt");
            vstrDesc.push_back(strImagePath + "/descriptors/" + ss.str() + "_desc.txt");
            double t;
            ss >> t;
            vTimeStamps.push_back(t*1e-9);

        }
    }
}

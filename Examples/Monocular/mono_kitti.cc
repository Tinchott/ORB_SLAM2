/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <sys/stat.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<iomanip>
#include<unistd.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgcodecs/legacy/constants_c.h>

#include"System.h"

using namespace std;

void LoadImages(const string &strSequence, vector<string> &vstrImageFilenames,
                vector<double> &vTimestamps);

inline bool exists_test3 (const std::string& name);         

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./mono_kitti path_to_vocabulary path_to_settings path_to_sequence" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    LoadImages(string(argv[3]), vstrImageFilenames, vTimestamps);

    int nImages = vstrImageFilenames.size();

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat im;
    for(int ni=0; ni<nImages; ni++)
    {
        // Read image from file
        im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(im.empty())
        {
            cerr << endl << "Failed to load image at: " << vstrImageFilenames[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the image to the SLAM system
        SLAM.TrackMonocular(im,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    //Guardo la nube de puntos que se genera frame a frame en el siguiente directorio, bajo el siguiente formato:
    std::string test_name = "/home/tincho/Escritorio/TestFrame/TestFile" + std::to_string(ni) + ".txt";
    SLAM.SaveCurrentMapPoints(test_name);
    //La siguiente línea exporta la Tcw https://github.com/raulmur/ORB_SLAM2/issues/262
    cv::Mat Tcw = SLAM.TrackMonocular(im,tframe);
    if(!Tcw.empty()){
	string nombreKeyFrame("/home/tincho/Escritorio/TestTrajFrame.txt");
	ofstream file_KeyFrame;

	file_KeyFrame.open(nombreKeyFrame, std::ios_base::app);
	file_KeyFrame << setprecision(9) << Tcw.at<float>(0,0) << " " << Tcw.at<float>(0,1) << " " << Tcw.at<float>(0,2) << " " 
	<< Tcw.at<float>(0,3) << " " << Tcw.at<float>(1,0) << " " << Tcw.at<float>(1,1) << " " << Tcw.at<float>(1,2) << " " 
	<< Tcw.at<float>(1,3) << " " << Tcw.at<float>(2,0) << " " << Tcw.at<float>(2,1) << " " << Tcw.at<float>(2,2) << " " 
	<< Tcw.at<float>(2,3) << " " << Tcw.at<float>(3,0) << " " << Tcw.at<float>(3,1) << " " << Tcw.at<float>(3,2) << " " 
	<< Tcw.at<float>(3,3) << " " << ni << " " << tframe <<  std::endl;
    }else{
    	string nombreKeyFrame("/home/tincho/Escritorio/TestTrajFrame.txt");
	ofstream file_KeyFrame;

	file_KeyFrame.open(nombreKeyFrame, std::ios_base::app);
	file_KeyFrame << "" << std::endl;
	
    }
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;
/*Esto lo modifico yo
    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");    
*/
    // Save camera trajectory (agrego para ver si toma el cambio)
    SLAM.SaveKeyFrameTrajectoryTUM("/home/tincho/Escritorio/CameraTrajectory.txt");
    
    SLAM.SaveMapPoints("/home/tincho/Escritorio/OrbMapPoints.txt");
    return 0;
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    ifstream fNames;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    string strPathImgNameFile = strPathToSequence + "/img_name.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);
    
    //Me fijo si existe un archivo con los nombres de las imágenes. En dicho caso, utilizo esos nombres para cargar las imágenes del ORBSLAM
    if( exists_test3(strPathImgNameFile) ){
       //Dado que existe un archivo con los nombres de las imágenes, utilizo esos nombres
       int i=0;
       cout << "Utilizando nombres de imagenes dados en 'img_name.txt'" << endl;
       fNames.open(strPathImgNameFile.c_str());
       while(!fNames.eof() && i<nTimes)
       {
          string s;
          getline(fNames,s);
          if( !s.empty() ){
             vstrImageFilenames[i] = strPrefixLeft + s;
          }
          i++;
       }
    }
    else{
       //Dado que no existe un archivo con los nombres de las imágenes, utilizo los nombres por defecto
       for(int i=0; i<nTimes; i++)
       {
           stringstream ss;
           ss << setfill('0') << setw(6) << i;
           vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
       }
    }
}
//Función para ver si existe un archivo (la tomo de aquí: https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exists-using-standard-c-c11-14-17-c#:~:text=on%20this%20post.-,inline%20bool%20exist(const%20std%3A%3Astring%26%20name)%20%7B,The%20file%20was%20not%20found.)
inline bool exists_test3 (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

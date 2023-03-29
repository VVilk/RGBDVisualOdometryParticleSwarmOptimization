#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>



//#include "opencv2/core.hpp"
//#include "opencv2/features2d.hpp"
//include "opencv2/highgui.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/calib3d/calib3d.hpp>
//#include "opencv/cv.h"

//#define private public
//#include "opencv2/features2d/src/kaze/AKAZEFeatures.h"
//#undef private ale tylko jesli znam scierzke!!!

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//#include "odometria/odometrie.cpp"   includuja to ransaci
#include "ransaci/ransaci.cpp"
#include "../akaze-master-autor/src/lib/AKAZE.h"
//#include "AKAZE.h"
//#include "../akaze-master-autor/src/lib/AKAZEConfig.h"

//#include "akaze/akaze.cpp"
using namespace cv;

//Parametry kalibracyjne
// openni default
//double fx = 570.3422241210938;
//double fy = 570.3422241210938;
//double cx = 314.5;
//double cy = 235.5;

// fr3
/*
double fx = 535.4;
double fy = 539.2; 
double cx = 320.1;
double cy = 247.6;
double fx_depth = 567.6 ;
double fy_depth = 570.2 ;
double cx_depth = 324.7 ;
double cy_depth = 250.1 ;
double factor_depth = 5000.0;
*/
/*//fr2
double fx = 520.9;
double fy = 521.0; 
double cx = 325.1;
double cy = 249.7;
double fx_depth = 580.8;
double fy_depth = 581.8;
double cx_depth = 308.8;
double cy_depth = 253.0;
double factor_depth = 5000.0;
*/

//fr1
/*
double fx = 517.3;
double fy = 516.5; 
double cx = 318.6;
double cy = 255.3;
double fx_depth = 591.1;
double fy_depth = 590.1;
double cx_depth = 331.0;
double cy_depth = 234.0;
double factor_depth = 5000.0;
*/


//dataset 1 kin 1 putkk inferencje

double fx = 532.77250;
double fy = 531.85745; 
double cx = 312.23007;
double cy = 260.11708;
double fx_depth = 578.852799192617;
double fy_depth = 574.608347232532;
double cx_depth = 316.003688477099;
double cy_depth = 246.041146276871;
double factor_depth = 5000.0;

//dataset 1 kin 1 putkk
/*
double fx = 532.77250;
double fy = 531.85745; 
double cx = 312.23007;
double cy = 260.11708;
double fx_depth = 578.852799192617;
double fy_depth = 574.608347232532;
double cx_depth = 316.003688477099;
double cy_depth = 246.041146276871;
double factor_depth = 1000.0;
*/


//double fx = 517.3;
//double fy = 516.5;
//double cx = 318.6;
//double cy = 255.3;
//skala dla 16 bit PNG messor
//double factor=1000.0;


void wyswietl_macierz_transponowana(CvMat* Macierz);

void mask_no_depth (std::vector <Point2f> *keypoints,Mat& DepthImg, char* maska, int* liczba_inlierow)
{
  int i = 0;
  for (std::vector< Point2f >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    if( *(maska+i) != 0)
    {
      if((DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))  == 0) 
        {
          *(maska+i) = 0;
          (*liczba_inlierow) = (*liczba_inlierow) - 1;
          //std:: cout<<"maska 0 D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))<<std::endl;
        }
      /* else
       {
        std:: cout<<"maska 1 D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))<<std::endl;
       } */
    } 
     ++i;
     ++keyIt;
  }   
}

void depth_to_xyz_mat(std::vector <Point2f> *keypoints,Mat& DepthImg, CvMat* XYZ)
//void depth_to_xyz_mat(std::vector <DMatch> *keypoints,Mat& DepthImg, CvMat* XYZ)
{
int i = 0;
double X,Y,Z;
  for (std::vector< Point2f >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    Z = (DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))/factor_depth;
    X = ((int)(*keypoints)[i].x - cx_depth)*Z / fx_depth;
    Y = ((int)(*keypoints)[i].y - cy_depth)*Z / fy_depth;

    CV_MAT_ELEM(*XYZ, double, i, 0) = X;
    CV_MAT_ELEM(*XYZ, double, i, 1) = Y;
    CV_MAT_ELEM(*XYZ, double, i, 2) = Z;
  
    //std::cout<<"x: "<<(unsigned int)(*keypoints)[i].x<<" y: "<< (unsigned int)(*keypoints)[i].y<< "z: "<<Z<<" ddd: "<<ddd<<std::endl;
    //std::cout<<"\nX= "<<X<<" Y= "<<Y<<" Z= "<<Z;
     ++i;
     ++keyIt; 
  }  
  //wyswietl_macierz(XYZ);
 
}

void depth_to_xyz_mat_maska(std::vector <Point2f> *keypoints,Mat& DepthImg, CvMat* XYZ, char* maska)
//void depth_to_xyz_mat(std::vector <DMatch> *keypoints,Mat& DepthImg, CvMat* XYZ)
{
int i = 0;
int j = 0;
double X,Y,Z;
  for (std::vector< Point2f >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    if( *(maska+j) == 1) // ostatnie elementy maski moga byc zle bo maska 2 nadpisuje tylko pocztkowe elementy maski
    {
      Z=(DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x))/factor_depth;
      /*
      if((DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x)) <= 0.0001)
      {
        std:: cout<<"D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x))<<std::endl;
      }
      if(Z >= 100)
      {
        std::cout<<"D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x))<<std::endl;
      }
      */
      X = ((int)(*keypoints)[j].x - cx_depth)*Z / fx_depth;
      Y = ((int)(*keypoints)[j].y - cy_depth)*Z / fy_depth;

      CV_MAT_ELEM(*XYZ, double, i, 0) = X;
      CV_MAT_ELEM(*XYZ, double, i, 1) = Y;
      CV_MAT_ELEM(*XYZ, double, i, 2) = Z;
  
      //std::cout<<"\nX= "<<X<<" Y= "<<Y<<" Z= "<<Z;

       ++i;

    } 
     ++j;
     ++keyIt;
  }  
  //std::cout<<" i w depth maska" <<i<<" ";
  //wyswietl_macierz(XYZ);
 
}

void depth_to_xyz_mat_trans(std::vector <Point2f> *keypoints,Mat& DepthImg, CvMat* XYZ)
{
int i = 0;
double X,Y,Z;
  for (std::vector< Point2f >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    Z=(DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))/factor_depth;
    X = ((int)(*keypoints)[i].x - cx_depth)*Z / fx_depth;
    Y = ((int)(*keypoints)[i].y - cy_depth)*Z / fy_depth;

    CV_MAT_ELEM(*XYZ, double, 0, i) = X;
    CV_MAT_ELEM(*XYZ, double, 1, i) = Y;
    CV_MAT_ELEM(*XYZ, double, 2, i) = Z;
  
    //std::cout<<"\nX= "<<X<<" Y= "<<Y<<" Z= "<<Z;

     ++i;
     ++keyIt; 
  }
  //wyswietl_macierz(XYZ);
}


void depth_to_xyz_mat_maska_trans(std::vector <Point2f> *keypoints,Mat& DepthImg, CvMat* XYZ, char* maska)
{
int i = 0;
int j = 0;
double X,Y,Z;
  for (std::vector< Point2f >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    if( *(maska+j) != 0) 
    {
      if ( *(maska+j) !=1)
      {
        std::cout<<"maska !=0 ale nie == 1: "<< *(maska+j)<<std::endl;
      }
      //std::cout<< (*keypoints)[i].y <<" "<< (*keypoints)[i].x<<" "<<i << " "<< std::distance( keypoints->begin(), keyIt )<<std::endl;
      Z=(DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x))/factor_depth;
      /*
      if((DepthImg.at<unsigned short int>((int)(*keypoints)[j].y, (int)(*keypoints)[j].x)) <= 0.0001)
      {
        std::cout<<"D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))<<std::endl;
      }
      if(Z >= 100)
      {
        std::cout<<"D at: "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[i].y, (int)(*keypoints)[i].x))<<std::endl;
      }
      */
      X = ((int)(*keypoints)[j].x - cx_depth)*Z / fx_depth;
      Y = ((int)(*keypoints)[j].y - cy_depth)*Z / fy_depth;

      CV_MAT_ELEM(*XYZ, double, 0, i) = X;

      CV_MAT_ELEM(*XYZ, double, 1, i) = Y;
      CV_MAT_ELEM(*XYZ, double, 2, i) = Z;
  
      //std::cout<<"\nX= "<<X<<" Y= "<<Y<<" Z= "<<Z;
      //std::cout<<"y " <<(int)(*keypoints)[j].y- cy_depth<<"x "<< (int)(*keypoints)[j].x- cx_depth<<"z "<<Z<<std::endl;
      //std::cout<<"numer: punktu w liczeniu"<<j<<std::endl
       ++i;

    } 
    else
    {
      //std::cout<< (int)*(maska+j)<< " ";
    }
     ++j;
     ++keyIt;
  }
  //wyswietl_macierz(XYZ);
}

std::vector<DMatch> knnmatch(Mat descriptors_1, Mat descriptors_2)
{  
  //BruteForceMatcher with knnMatch + symmetrical matching scheme
  BFMatcher matcher( NORM_L2 );
  //FlannBasedMatcher matcher(
  //                           new flann::LshIndexParams(20, 10, 2), // Default index parameters);
  //                           // new KDTreeIndexParams( 16 ),  // Index using 16 kd-trees 4 is default
  //                           new flann::SearchParams(32,0,true)
  //                          );
  
  std::vector<DMatch> matches;
  std::vector<std::vector < DMatch > >  matches1, matches2;
  matcher.knnMatch(descriptors_1,descriptors_2,matches1,2);
  matcher.knnMatch(descriptors_2,descriptors_1,matches2,2);

  for (std::vector<std::vector < DMatch > > ::const_iterator matIt1=matches1.begin();matIt1!=matches1.end();++matIt1)
  {
   if (matIt1->size() < 2) continue;
   for (std::vector<std::vector < DMatch > > ::const_iterator matIt2=matches2.begin();matIt2!=matches2.end();++matIt2) 
   {
    if (matIt2->size() < 2)continue;

    if (((*matIt1)[0].queryIdx == (*matIt2)[0].trainIdx) &&  ((*matIt2)[0].queryIdx == (*matIt1)[0].trainIdx)) 
    {
     matches.push_back(DMatch((*matIt1)[0].queryIdx,(*matIt1)[0].trainIdx,(*matIt1)[0].distance));
     break;
    }
   }
  }
  return matches;
}




void delete_keypoints_without_depth(std::vector <KeyPoint> *keypoints,Mat& DepthImg)
{
  int i=0;
  //int usunieto = 0;
  for (std::vector< KeyPoint >::iterator keyIt= keypoints->begin(); keyIt != keypoints->end();)
  {
    if((DepthImg.at<unsigned short int>((int)(*keypoints)[i].pt.y, (int)(*keypoints)[i].pt.x))  == 0) 
    {
      keyIt=keypoints->erase(keyIt);
      //++usunieto;
    }
    else
    {
      //std::cout<<"Zostało x: "<<(unsigned int)(*keypoints)[i].pt.x<<" y: "<< (unsigned int)(*keypoints)[i].pt.y<<"depth : "<<(DepthImg.at<unsigned short int>((int)(*keypoints)[i].pt.y, (int)(*keypoints)[i].pt.x))<<std::endl;

     ++keyIt; 
     ++i;
    }
  }
  //std::cout<<"USUNIETO: "<<usunieto<<std::endl;
}


//
//  DO zrobienia w mainie: przyjac poprzednie zmienne wedlug wynikow optymalizacji, usunac je z pso i dodac kcontrast_percentile
//


/** @function main */
void cala_petla(std::string* folder, std::string* result_folder_nadany, std::string nazwa, std::vector< std::pair<double, std::string> >* rgb_stamps_and_filenames, std::vector< std::pair<double, std::string> >* depth_stamps_and_filenames, double error_threshold_nadany, double pewnosc_1_nadana, double error_threshold2_nadany,  double pewnosc_2_nadana, float detector_threshold, float akaze_kcontrast_percentile, int flann_param_, int* status)
//void cala_petla(std::string* folder, std::string* result_folder_nadany, std::string nazwa, std::vector< std::pair<double, std::string> >* rgb_stamps_and_filenames, std::vector< std::pair<double, std::string> >* depth_stamps_and_filenames, double error_threshold_nadany, double pewnosc_1_nadana, double error_threshold2_nadany,  double pewnosc_2_nadana, float detector_threshold, int flann_param_, int* status)
{
  *status = 1;
  std::string result_folder =  *result_folder_nadany;
  double error_threshold =  error_threshold_nadany ;
  double pewnosc_1 = pewnosc_1_nadana;
  double error_threshold2 = error_threshold2_nadany;
  double pewnosc_2 = pewnosc_2_nadana ;
  //inicjalizacja czytania folderow 
  timeval timer;
  double time_1, time_2, time_3, time_4, time_5, time_6, time_7,
          time_8, time_9, time_10, time_11, time_12, time_13, time_14;
  double time_cumulative_1 = 0, time_cumulative_2 = 0, time_cumulative_3 = 0, time_cumulative_4 = 0,
         time_cumulative_5 = 0, time_cumulative_6 = 0, time_cumulative_7 = 0, time_cumulative_8 = 0,
         time_cumulative_9 = 0, time_cumulative_10 = 0;
  gettimeofday(&timer, NULL);
  time_1= timer.tv_sec+(timer.tv_usec/1000000.0);
  //FILE *file_rgb_txt;
  //FILE *file_depth_txt;
  //int znak_plik;
  int liczba_inlierow = 0, liczba_inlierow_kumulatywna = 0;
  int liczba_outlairow = 0, liczba_outlairow_kumulatywna = 0;
  int point_count_kumulatywna = 0;
  char ransac_fail = 0;
  int punkty = 0;

  //int a1 =0, a2 =0 , a3 = 0, a4= 0,a5=0, a6=0, a7=0, a8 = 0;
  Mat img_rgb_1; //pozostalosc, wczytuje w grey scale
  Mat img_rgb_2;
  Mat img_d_1;
  Mat img_d_2;
  // oryginalna biblioteka AKAZE zyczy sobie obrazy czarno biale w 32f
  //Mat img_1;
  //Mat img_2;
  Mat img_1_32;
  Mat img_2_32;
  
  AKAZEOptions options;

  CvMat* Translation = cvCreateMat( 3, 1, CV_64FC1);
  CvMat* Rotation = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* Rotacja_ogolna = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* Translacja_ogolna = cvCreateMat( 3, 1, CV_64FC1);

  CV_MAT_ELEM(* Rotacja_ogolna, double, 0, 0) = 1;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 0, 1) = 0;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 0, 2) = 0;

  CV_MAT_ELEM(* Rotacja_ogolna, double, 1, 0) = 0;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 1, 1) = 1;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 1, 2) = 0;

  CV_MAT_ELEM(* Rotacja_ogolna, double, 2, 0) = 0;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 2, 1) = 0;
  CV_MAT_ELEM(* Rotacja_ogolna, double, 2, 2) = 1;

  CV_MAT_ELEM(* Translacja_ogolna, double, 0, 0) = 0;
  CV_MAT_ELEM(* Translacja_ogolna, double, 1, 0) = 0;
  CV_MAT_ELEM(* Translacja_ogolna, double, 2, 0) = 0;

  double A_init[4][4]= {1, 0, 0, 0, 
                        0, 1, 0, 0,              
                        0, 0, 1, 0,              
                        0, 0, 0, 1};
  Mat A(4,4, DataType<double>::type ,&A_init);
  //Mat C = (Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);   moze tak?
  double A_init2[4][4]= {1, 0, 0, 0, 
                         0, 1, 0, 0,              
                         0, 0, 1, 0,              
                         0, 0, 0, 1};
  Mat A_lokalne(4,4, DataType<double>::type ,&A_init2);

  //Mat C = (Mat_<double>(3,3) << 0, -1, 0, -1, 5, -1, 0, -1, 0);   moze tak?

  //do ransaca
  Eigen::Matrix3d Rotation_best;
  Eigen::Vector3d translation_best;

 

  //otwieram plik do zapisu oszacowanej trajektorii
  std::ofstream estimated_trajectory_file_txt( (result_folder + "trajectory_" + nazwa +".txt").c_str());
  std::ofstream statystyki( (result_folder + "statystyki_" + nazwa +".txt").c_str());
  std::ofstream statystyki_kumulatywne( (result_folder + "statystyki_kumulatywne_" + nazwa +".txt").c_str());

  std::cout<<"zapisuje trajektorie i statystyki do folderu: "<<  (result_folder).c_str();

  statystyki<< "# wczytywanie danych## ekstrakcja cech## usuwanie cech dla danych bez glebi## deskrypcja## matching## sprawdzenie podobienstwa- similarity check## pobieranie odleglosci## ransac## odom2## zapisywanie wynikow"<<std::endl;



  estimated_trajectory_file_txt<< std::fixed<< std::setprecision(6)   
                               << (*rgb_stamps_and_filenames)[0].first<< " "
                               << "0.000000 "<< "0.000000 " << "0.000000 "
                               << "0.000000 "<< "0.000000 " << "0.000000 "<< "1.000000"
                               << std::endl;

  img_rgb_1 = imread (*folder + "/" + (*rgb_stamps_and_filenames)[0].second, IMREAD_GRAYSCALE);
  img_rgb_1.convertTo(img_1_32, CV_32F, 1.0/255.0, 0);
  img_d_1 = imread (*folder + "/" + (*depth_stamps_and_filenames)[0].second, IMREAD_ANYDEPTH);
  
  // im wiekszy prog tym mniej keypointow
  //float akaze_thresh = detector_threshold; //zaczynamy od 1000 i co 50 w dół jak ransac pada
  //Ptr<AKAZE> detector = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, akaze_thresh);
  options.dthreshold = detector_threshold;
  options.kcontrast_percentile = akaze_kcontrast_percentile;
  //options.kcontrast_percentile = 0.7; //domyslne
  options.img_width = img_rgb_1.cols;
  options.img_height = img_rgb_1.rows;

  libAKAZE::AKAZE akaze_detector_descryptor(options);

  std::vector<KeyPoint> keypoints_1, keypoints_2;
  Mat descriptors_1, descriptors_2;

  //detector->detect( img_rgb_1, keypoints_1 );
  //detector->compute( img_rgb_1, keypoints_1, descriptors_1 );

  akaze_detector_descryptor.Create_Nonlinear_Scale_Space(img_1_32);
  akaze_detector_descryptor.Feature_Detection(keypoints_1);
  delete_keypoints_without_depth(&keypoints_1, img_d_1);
  akaze_detector_descryptor.Compute_Descriptors(keypoints_1, descriptors_1);

  gettimeofday(&timer, NULL);
  time_2= timer.tv_sec+(timer.tv_usec/1000000.0);
  std::cout<< "Czas inicjalizacji: "<< time_2 - time_1<<std::endl;
  //std::cout<< klatki_rgb.size()<< std::endl;
  std::cout<<"numer_klatki:\n";

  for (unsigned int numer_klatki=1 ; numer_klatki < (*rgb_stamps_and_filenames).size(); ++numer_klatki)
  {
     gettimeofday(&timer, NULL);
     time_3= timer.tv_sec+(timer.tv_usec/1000000.0);
     //flush by wypluwal na ekran
     std::cout<<numer_klatki<<" "<<std::flush;

     //Mat img_rgb_1 = imread( aktualna_klatka_rgb.c_str(), IMREAD_GRAYSCALE );
     //Mat img_rgb_2 = imread( poprzednia_klatka_rgb.c_str(), IMREAD_GRAYSCALE );
     
     img_2_32 = img_1_32;
     //img_rgb_1 = imread (klatki_rgb[numer_klatki], IMREAD_GRAYSCALE);
     img_rgb_1 = imread (*folder + "/" + (*rgb_stamps_and_filenames)[numer_klatki].second, IMREAD_GRAYSCALE); // na tych samych danych
     img_rgb_1.convertTo(img_1_32, CV_32F, 1.0/255.0, 0);

     img_d_2 = img_d_1;
     img_d_1 = imread (*folder + "/" + (*depth_stamps_and_filenames)[numer_klatki].second, IMREAD_ANYDEPTH);

     //std::cout<<"Macierz glebi 1: \n"<<img_d_1;


     //cv::imshow("img_rgb_2", img_rgb_2);
     //cv::waitKey(1);
    // z http://answers.opencv.org/question/27787/imshow-and-waitkey/      Note on documentation: "This function is the only method in HighGUI that can fetch and handle events, so it needs to be called periodically for normal event processing unless HighGUI is used within an environment that takes care of event processing." Since this is the only method, you are pretty much stuck.


     //By sprawdzić typ danych. 2 odpowiada CV_16U
     //std::cout<<"\nTyp danych depth: "<< img_d_1.depth() << " fesgsgesg\n";
     //namedWindow("Dispaly depth window", WINDOW_AUTOSIZE);
     //imshow("Dispaly depth window",img_d_1);
 
     if( !img_rgb_1.data || !img_rgb_2.data )
     {  } // tu bylo return -1 ale ze to jest teraz w funkcji to usunolem

     ////-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
     gettimeofday(&timer, NULL);
     time_4= timer.tv_sec+(timer.tv_usec/1000000.0);

     keypoints_2.erase( keypoints_2.begin(), keypoints_2.end() );
     keypoints_2 = keypoints_1; 
     keypoints_1.erase( keypoints_1.begin(), keypoints_1.end() );
     //detector->detect( img_rgb_1, keypoints_1 );
  
     akaze_detector_descryptor.Create_Nonlinear_Scale_Space(img_1_32);
     akaze_detector_descryptor.Feature_Detection(keypoints_1);

     delete_keypoints_without_depth(&keypoints_1, img_d_1);
     delete_keypoints_without_depth(&keypoints_2, img_d_2);

     gettimeofday(&timer, NULL);
     time_5= timer.tv_sec+(timer.tv_usec/1000000.0);

     ////bylo detectAndCompute

     gettimeofday(&timer, NULL);
     time_6 = timer.tv_sec+(timer.tv_usec/1000000.0);

     descriptors_2.release();
     descriptors_2 = descriptors_1;
     descriptors_1.release();
     //detector->compute( img_rgb_1, keypoints_1, descriptors_1 );
     akaze_detector_descryptor.Compute_Descriptors(keypoints_1, descriptors_1);
     //detector->compute( img_rgb_2, keypoints_2, descriptors_2 );

     gettimeofday(&timer, NULL);
     time_7 = timer.tv_sec+(timer.tv_usec/1000000.0);

     ////-- Step 2: Matching descriptor vectors with a brute force matcher
     //BFMatcher matcher(NORM_L2);
     std::vector< DMatch > matches;
     //matcher.match( descriptors_1, descriptors_2, matches );
     matches = knnmatch(descriptors_1, descriptors_2); 
    
     gettimeofday(&timer, NULL);
     time_8 = timer.tv_sec+(timer.tv_usec/1000000.0);


     //-- Step 2: Matching descriptor vectors with a brute force matcher
     //NORM_L2 dobra dla SIFT SURF i podobnych
     //NORM_HAMMING dla ORB BRIEF BRISK itd.
     //Jesli ORB korzysta z WTA_K == 3 lub 4 powinno sie skorzystac z NORM_HAMMING2  // http://docs.opencv.org/modules/features2d/doc/feature_detection_and_description.html
     // Second param is boolean variable, crossCheck which is false by default. If it is true, Matcher returns only those matches with value (i,j) such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa. That is, the two features in both sets should match each other. It provides consistant result, and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
     // NORM, crossCheck
     //BFMatcher matcher_Brisk(NORM_L2);
     //BFMatcher matcher_Brisk(NORM_L2, true);

     //std::vector< DMatch > good_matches;
     //matcher_Brisk.match( descriptors_Brisk_1, descriptors_Brisk_2, matches_Brisk );

     int point_count=matches.size();
     std::vector<Point2f> points1;
     points1.reserve(point_count);
     points1.erase( points1.begin(), points1.end() );
     std::vector<Point2f> points2;
     points2.reserve(point_count);
     points2.erase( points2.begin(), points2.end() );

     // CV_16U
     CvMat* points_mat_1 = cvCreateMat(point_count, 3, CV_64F);
     CvMat* points_mat_transposed_1 = cvCreateMat(3, point_count, CV_64F);
     CvMat* points_mat_2 = cvCreateMat(point_count, 3, CV_64F);

     // nie udokumentowana metoda
     //std::vector<Point2f> points11; 
     //KeyPoint::convert(keypoints_1, points11);

     for(int i=0; i<point_count;i++)
     {
        //queryIdx wskazuje na descriptors1
        //trainIdx - wskazuje na descryptors2
        int x = keypoints_1[matches[i].queryIdx].pt.x; //lewy 1 obraz
        int y = keypoints_1[matches[i].queryIdx].pt.y;

        points1.push_back(cv::Point2f(x,y));

        x = keypoints_2[matches[i].trainIdx].pt.x;
        y = keypoints_2[matches[i].trainIdx].pt.y;
 	
        points2.push_back(cv::Point2f(x,y));
     }
     gettimeofday(&timer, NULL);
     time_9 = timer.tv_sec+(timer.tv_usec/1000000.0);

     depth_to_xyz_mat_trans (&points1, img_d_1, points_mat_transposed_1);
     cvTranspose( points_mat_transposed_1, points_mat_1);
     depth_to_xyz_mat (&points2, img_d_2, points_mat_2);

     //std::cout <<"\n "<< points2_Brisk[1].x << " "<< points2_Brisk[1].y<<std::endl;
     //RANSAC
     //std::vector< DMatch > gg;
     //gg = lmeds(matches, points1,points2);

     //int good_point_count=good_matches_Brisk.size();
     // ZAMIANA PRZY OPENGV

     gettimeofday(&timer, NULL);
     time_10 = timer.tv_sec+(timer.tv_usec/1000000.0);
 
     char maska[point_count];
     Eigen::VectorXd wyniki_best_ransac_1(point_count);
     wyniki_best_ransac_1.setConstant(10000);
     //std::cout<<"wyniki_best_ransac_1 :\n"<<wyniki_best_ransac_1<<std::endl;
     //
     error_threshold = error_threshold_nadany;
     pewnosc_1 = pewnosc_1_nadana;
     //std::cout<<"r1 "<<std::endl;
     do 
     {
     // std::cout<<"ransac 1!!!!!"<< std::endl;
       ransac_v2(points_mat_transposed_1, points_mat_2, point_count, &maska[0],error_threshold, pewnosc_1, 315, &liczba_inlierow, &ransac_fail);
       punkty = point_count;
       //std::cout<<"ransac 1!!!!!"<< std::endl;
       if (ransac_fail == 1)
       {
         //error_threshold += 0.000001;
         error_threshold += 0.04;
         std::cout<<"Ransac FAIL, zmieniam error threshold 1 dla wektorów wodzących o 1mm "<< error_threshold<< " mam inlairow "<< liczba_inlierow<<"inlierow a point count to" << point_count<<"\n";
       }

     // else
     // {
     //   std::cout<<" ok\n";
     // }
     }while(ransac_fail == 1); 
     //std::cout<<"kkkkk ";
     wyniki_best_ransac_1.setConstant(10000);

     CvMat* points_ransac1_mat_transposed_1 = cvCreateMat(3, liczba_inlierow, CV_64F);
     CvMat* points_ransac1_mat_2 = cvCreateMat(liczba_inlierow, 3, CV_64F);
     int j=0;
     for(int i=0; j < liczba_inlierow; ++i)
     {
       if(maska[i] == 1)
       {
         CV_MAT_ELEM(* points_ransac1_mat_transposed_1, double, 0, j) = cvmGet(points_mat_transposed_1, 0, i);
         CV_MAT_ELEM(* points_ransac1_mat_transposed_1, double, 1, j) = cvmGet(points_mat_transposed_1, 1, i);
         CV_MAT_ELEM(* points_ransac1_mat_transposed_1, double, 2, j) = cvmGet(points_mat_transposed_1, 2, i);

         CV_MAT_ELEM(* points_ransac1_mat_2, double, j, 0) = cvmGet(points_mat_2, i, 0);
         CV_MAT_ELEM(* points_ransac1_mat_2, double, j, 1) = cvmGet(points_mat_2, i, 1);
         CV_MAT_ELEM(* points_ransac1_mat_2, double, j, 2) = cvmGet(points_mat_2, i, 2);
         ++j;
       }
     }

     //float mnoznik = 0.1; //mnożnik- może błędziemy zwiększać zmniejszać błąd jak się uda
     //error_threshold2 = 0.0001;//error_threshold * mnoznik;// zmieniamy dopuszczalny błąd
     error_threshold2 = error_threshold2_nadany;//error_threshold * mnoznik;// zmieniamy dopuszczalny błąd
     int liczba_punktow = liczba_inlierow;
     pewnosc_2 = pewnosc_2_nadana;
     //std::cout<<"r1r2 " <<std::endl;
     do 
     {
       ransac_v2(points_ransac1_mat_transposed_1, points_ransac1_mat_2, liczba_punktow, &maska[0],error_threshold2, pewnosc_2, 450,&liczba_inlierow, &ransac_fail);
       std::cout<<"kabsch";        
       //ransac_v2_rotacja_eigensolver(&bearing_Vectors_1_ransac1, &bearing_Vectors_2_ransac1, liczba_punktow, &maska[0],error_threshold2, 0.80, 1000,&liczba_inlierow, &ransac_fail);
       //ransac_v2_rotacja_kneip(&bearing_Vectors_1_ransac1, &bearing_Vectors_2_ransac1, liczba_punktow, &maska[0],error_threshold2, 0.80, 1000,&liczba_inlierow, &ransac_fail);
       //ransac_v2_rotacja_nister(&bearing_Vectors_1_ransac1, &bearing_Vectors_2_ransac1, liczba_punktow, &maska[0],error_threshold2, 0.80, 1000,&liczba_inlierow, &ransac_fail);
       //ransac_v2_rotacja_eight(&bearing_Vectors_1_ransac1, &bearing_Vectors_2_ransac1, liczba_punktow, &maska[0],error_threshold2, 0.80, 1000,&liczba_inlierow, &ransac_fail);
       //ransac_v2_rotacja_nonlinear(&bearing_Vectors_1_ransac1, &bearing_Vectors_2_ransac1, liczba_punktow, &maska[0],error_threshold2, 0.80, 1000,&liczba_inlierow, &ransac_fail);

       if (ransac_fail == 1)
       {
         //mnoznik += 0.1;
         //error_threshold2 += 0.000001;// zmieniamy dopuszczalny błąd
         error_threshold2 += 0.002;// zmieniamy dopuszczalny błąd
         //std::cout<<"Ransac FAIL, zmieniam mnoznik "<<mnoznik<<" o 0.1 w gore. treshold teraz wynosi "<< error_threshold2<< " mam "<< liczba_inlierow<<"inlierow a point count to" << point_count<<"\n";
         std::cout<<"Ransac2 FAIL, zmieniam threshold error o 0.002 w gore. treshold teraz wynosi "<< error_threshold2<< " mam "<< liczba_inlierow<<"inlierow a point count to" << point_count<<"\n";
       }
     }while(ransac_fail == 1); 
     //std::cout<<" mam "<< liczba_inlierow<<"inlierow a point count to" << point_count<<"\n";
 
     //std::cout<<"po r " <<std::endl;
     CvMat* points_inliers_mat_transposed_1 = cvCreateMat(3, liczba_inlierow, CV_64F);
     CvMat* points_inliers_mat_2 = cvCreateMat(liczba_inlierow, 3, CV_64F);

     j=0;
     for(int i=0; j < liczba_inlierow; ++i)
     {
       if(maska[i] == 1)
       {
         CV_MAT_ELEM(* points_inliers_mat_transposed_1, double, 0, j) = cvmGet(points_ransac1_mat_transposed_1, 0, i);
         CV_MAT_ELEM(* points_inliers_mat_transposed_1, double, 1, j) = cvmGet(points_ransac1_mat_transposed_1, 1, i);
         CV_MAT_ELEM(* points_inliers_mat_transposed_1, double, 2, j) = cvmGet(points_ransac1_mat_transposed_1, 2, i);

         CV_MAT_ELEM(* points_inliers_mat_2, double, j, 0) = cvmGet(points_ransac1_mat_2, i, 0);
         CV_MAT_ELEM(* points_inliers_mat_2, double, j, 1) = cvmGet(points_ransac1_mat_2, i, 1);
         CV_MAT_ELEM(* points_inliers_mat_2, double, j, 2) = cvmGet(points_ransac1_mat_2, i, 2);
         ++j;
       }
     }
     
     cvReleaseMat(&points_ransac1_mat_transposed_1);
     cvReleaseMat(&points_ransac1_mat_2);

     gettimeofday(&timer, NULL);
     time_11 = timer.tv_sec+(timer.tv_usec/1000000.0);

     //std::cout<<"rotacja kneip przed \n";

     //std::cout<<"r2o " <<std::endl;

     kabsch(points_inliers_mat_transposed_1, points_inliers_mat_2, liczba_inlierow, Rotation, Translation);
     //std::cout<<"kabschs skala:  "<<skala<<"## " <<std::endl;

     cvReleaseMat(&points_inliers_mat_transposed_1);
     cvReleaseMat(&points_inliers_mat_2);
     cvReleaseMat(&points_mat_1);
     cvReleaseMat(&points_mat_transposed_1);
     cvReleaseMat(&points_mat_2);

     /*
     int count = 0;
     for (maska_ofset= 0 ; maska_ofset < ssss; ++maska_ofset)
     {
       if (maska_2[maska_ofset] == 1)
       {
         ++count;
       }
     }

    std::cout<<"przed mask depth maska_2 count to: "<< count<<" ";
     count =0;
     for (maska_ofset= 0 ; maska_ofset < point_count; ++maska_ofset)
     {
       if (maska[maska_ofset] == 1)
       {
         ++count;
       }
     }
     std::cout<<"przed mask depth count to: "<< count<< " a liczba_inlierow to: "<< liczba_inlierow<< std::endl;
     */ 


//     if (((glebia_skala == 0) || (numer_klatki < 2)) &&(argv[3][0] != '9')) //czyli bez kabscha
     gettimeofday(&timer, NULL);
     time_12 = timer.tv_sec+(timer.tv_usec/1000000.0);

     A_lokalne.at<double>(0,0) = cvmGet(Rotation, 0, 0);
     A_lokalne.at<double>(0,1) = cvmGet(Rotation, 0, 1);
     A_lokalne.at<double>(0,2) = cvmGet(Rotation, 0, 2);

     A_lokalne.at<double>(1,0) = cvmGet(Rotation, 1, 0);
     A_lokalne.at<double>(1,1) = cvmGet(Rotation, 1, 1);
     A_lokalne.at<double>(1,2) = cvmGet(Rotation, 1, 2);

     A_lokalne.at<double>(2,0) = cvmGet(Rotation, 2, 0);
     A_lokalne.at<double>(2,1) = cvmGet(Rotation, 2, 1);
     A_lokalne.at<double>(2,2) = cvmGet(Rotation, 2, 2);

     A_lokalne.at<double>(0,3) = cvmGet(Translation, 0, 0);
     A_lokalne.at<double>(1,3) = cvmGet(Translation, 1, 0);
     A_lokalne.at<double>(2,3) = cvmGet(Translation, 2, 0);

     A=A*A_lokalne;


     cvGEMM (Rotacja_ogolna, Translation, 1, Translacja_ogolna, 1, Translacja_ogolna, 0);
     cvMatMul(Rotacja_ogolna,Rotation,Rotacja_ogolna);


     /*
     double RotationTab1[9];

     RotationTab1[0] = A.at<double>(0,0);
     RotationTab1[1] = A.at<double>(1,0);
     RotationTab1[2] = A.at<double>(2,0);

     RotationTab1[3] = A.at<double>(0,1);
     RotationTab1[4] = A.at<double>(1,1);
     RotationTab1[5] = A.at<double>(2,1);

     RotationTab1[6] = A.at<double>(0,2);
     RotationTab1[7] = A.at<double>(1,2);
     RotationTab1[8] = A.at<double>(2,2);
     */

     double RotationTab2[9];

     RotationTab2[0] = cvmGet(Rotacja_ogolna, 0, 0);
     RotationTab2[1] = cvmGet(Rotacja_ogolna, 1, 0);
     RotationTab2[2] = cvmGet(Rotacja_ogolna, 2, 0);

     RotationTab2[3] = cvmGet(Rotacja_ogolna, 0, 1);
     RotationTab2[4] = cvmGet(Rotacja_ogolna, 1, 1);
     RotationTab2[5] = cvmGet(Rotacja_ogolna, 2, 1);

     RotationTab2[6] = cvmGet(Rotacja_ogolna, 0, 2);
     RotationTab2[7] = cvmGet(Rotacja_ogolna, 1, 2);
     RotationTab2[8] = cvmGet(Rotacja_ogolna, 2, 2);
 

/*
     RotationTab[0] = cvmGet(Rotacja_ogolna, 0, 0);
     RotationTab[3] = cvmGet(Rotacja_ogolna, 1, 0);
     RotationTab[6] = cvmGet(Rotacja_ogolna, 2, 0);

     RotationTab[1] = cvmGet(Rotacja_ogolna, 0, 1);
     RotationTab[4] = cvmGet(Rotacja_ogolna, 1, 1);
     RotationTab[7] = cvmGet(Rotacja_ogolna, 2, 1);

     RotationTab[2] = cvmGet(Rotacja_ogolna, 0, 2);
     RotationTab[5] = cvmGet(Rotacja_ogolna, 1, 2);
     RotationTab[8] = cvmGet(Rotacja_ogolna, 2, 2);
*/
    //  Eigen::Map<Eigen::Matrix3f> eigeniRotmat (RotationMat.ptr<float>(), RotationMat.rows, RotationMat.cols);//( RotationMat.data() ); 
    //std::cout<<"Rotacja ogolna: \n";
     Eigen::Map<Eigen::Matrix3d> eigenRot( RotationTab2 ); 
     //std::cout <<"EIGEN ROT: \n"<<eigenRot << std::endl;
     Eigen::Quaterniond q(eigenRot);
     //std::cout<< q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
     estimated_trajectory_file_txt<< std::fixed<< std::setprecision(6) 
                                  << (*rgb_stamps_and_filenames)[numer_klatki].first<< " "
                                  //<< 0.000000<< " "
                                  //<< 0.000000<< " "
                                  //<< 0.000000<< " " 
                                  <<  cvmGet(Translacja_ogolna, 0, 0)<<" "
                                  <<  cvmGet(Translacja_ogolna, 1, 0)<<" "
                                  <<  cvmGet(Translacja_ogolna, 2, 0)<<" "
                                  << q.x()<< " "<<q.y()<< " "<< q.z()<< " "<< q.w()<< std::endl;
     //wyswietl_macierz(Rotation);
     //wyswietl_macierz(Translation);
     //wyswietl_macierz(points_mat_1);
  
     gettimeofday(&timer, NULL);
     time_13 = timer.tv_sec+(timer.tv_usec/1000000.0);

     liczba_outlairow = punkty - liczba_inlierow;

     statystyki << std::fixed<< std::setprecision(6)<<time_4 - time_3<< " "
               << time_5 - time_4<< " "<< time_6 - time_5<<" "<<time_7 - time_6<< " "
               << time_8 - time_7<< " "<< time_9 - time_8<<" "<<time_10 - time_9<< " "
               << time_11 - time_10<< " "<< time_12 - time_11<<" "<<time_13 - time_12<< " "
               << punkty<< " "<< liczba_inlierow<<" "<<liczba_outlairow <<std::endl;
    
     time_cumulative_1 += time_4 - time_3; // wczytywanie obrazow
     time_cumulative_2 += time_5 - time_4; //czas detekcji
     time_cumulative_3 += time_6 - time_5; //usuwanie punktow bez glebi
     time_cumulative_4 += time_7 - time_6; //czas deskrypcji
     time_cumulative_5 += time_8 - time_7; //matching knn
     time_cumulative_6 += time_9 - time_8; // podział punktow na dwa wektory
     time_cumulative_7 += time_10 - time_9; // przepisywanie do macierzy
     time_cumulative_8 += time_11 - time_10; // ransac
     time_cumulative_9 += time_12 - time_11; // kabsch
     time_cumulative_10 += time_13 - time_12; // przeliczanie i inne przed zapisem statysyuk
    
     point_count_kumulatywna += punkty;
     liczba_outlairow_kumulatywna += liczba_outlairow;
     liczba_inlierow_kumulatywna += liczba_inlierow;

     statystyki_kumulatywne << std::fixed<< std::setprecision(6)<<time_cumulative_1<< " "
                            <<time_cumulative_2<< " "<<time_cumulative_3<< " "<<time_cumulative_4<< " "
                            <<time_cumulative_5<< " "<<time_cumulative_6<< " "<<time_cumulative_7<< " "
                            <<time_cumulative_8<< " "<<time_cumulative_9<< " "<<time_cumulative_10<< " "
                            <<point_count_kumulatywna<< " "<< liczba_inlierow_kumulatywna<< " "
                            <<liczba_outlairow_kumulatywna<< std::endl;
     //-- Draw matches
     //cvReleaseMat(&Rotacja_transpose);
     // cvReleaseMat(&points_mat_transposed_1);
     //cvReleaseMat(&points_mat_2);
     gettimeofday(&timer, NULL);

  /*  //wczytywanie danych
    std::cout<< time_4 - time_3<<"\n";
 
    //ekstracja cech
    std::cout<< time_5 - time_4<<"\n";
    
    //usuwanie cech dla danych bez glebi
    std::cout<< time_6 - time_5<<"\n";

    //deskrypcja
    std::cout<< time_7 - time_6<<"\n";

    //matching
    std::cout<< time_8 - time_7<<"\n";

    //sprawdzenie podobienstwa- similarity check
    std::cout<< time_9 - time_8<<"\n";

    // pobieranie odleglosci
    std::cout<< time_10 - time_9<<"\n";

    //ransac
    std::cout<< time_11 - time_10<<"\n";

    //kabsvh
    std::cout<< time_12 - time_11<<"\n";

    //zapisywanie wynikow
    std::cout<< time_13 - time_12<<"\n";
*/
  }

  //calosc
  gettimeofday(&timer, NULL);
  time_14 = timer.tv_sec+(timer.tv_usec/1000000.0);
  std::cout<< "\n Calkowity obieg petli\a"<<time_14 - time_1<<" ";
  std::cout<< "FPS: "<< ((*rgb_stamps_and_filenames).size()-1)/(time_14-time_1)<<std::endl;  // -1 bo licze pary klatek
  //std::cout<< "error_threshold co 0.001 "<< a1<< " " << a2<< " " << a3<< " " << a4<< " " << a5<< " " << a6<< " " << a7<< " " << a8<< " "  <<std::endl;
  statystyki <<std::endl << std::endl<< std::endl<<"FPS: "<<((*rgb_stamps_and_filenames).size()-1)/(time_14-time_1)<< std::endl;

  cvReleaseMat(&Rotation);
  cvReleaseMat(&Rotacja_ogolna);
  cvReleaseMat(&Translation);
  cvReleaseMat(&Translacja_ogolna);

  estimated_trajectory_file_txt.close();
  statystyki.close();
  statystyki_kumulatywne.close();
  *status = 2;
}


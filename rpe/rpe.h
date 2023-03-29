#ifndef RPE_H
#define RPE_H
#include <stdio.h>
#include <iostream>
#include <vector>
#include <stdlib.h>
#include <cmath>
#include <cstring>
#include <fstream>

#include<eigen3/Eigen/StdVector>
//#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/LU>

void associate_stare (std::vector < std::pair <double, std::string>  > rgb,  std::vector < std::pair <double, std::string>  > depth,  std::vector < std::pair <double, std::string>  >& rgb_matched,  std::vector < std::pair <double, std::string>  >& depth_matched);

void associate_delta (std::vector<double> stampy, double delta, std::vector < std::pair < unsigned int, unsigned int> >& delta_stamps);

void powiaz_delta (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > est,  std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt, double delta, double m_roznica_czasu, std::vector < std::tuple < unsigned int, unsigned int, unsigned int, unsigned int> >& delta_stamps);

void associate (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > est,  std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt,  std::vector<double> & est_matched, std::vector<double> & gt_matched, int pierwszy = 0, int ostatni = -1);

double max_roznica_czasu (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt);

void readFile_timestamp (std::string file , std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >& trans_kwat);

void transform44p1_stare (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >  & trans_kwat, std::vector < std::pair <double, Eigen::Matrix4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Matrix4d> >  >& T44p1);

void transform44p1 (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >  & trans_kwat, std::vector < std::pair <double, Eigen::Matrix4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Matrix4d> >  >& T44p1, std::vector<double> vec_matched);

void transform44p1_1macierz (Eigen::Matrix<double,8,1> & trans_kwat, Eigen::Matrix4d& T44p1);

void ominus(Eigen::Matrix<double,4,4>& A, Eigen::Matrix<double,4,4>& B, Eigen::Matrix<double,4,4>& Wynik);

void readme_rpe();

double rpe(std::string gt_txt, std::string szac_txt);
//int rpe( int argc, char** argv);



#endif // RPE_H

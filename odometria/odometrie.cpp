#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>

#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/highgui.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/calib3d/calib3d.hpp>
#include "opencv/cv.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry> 


void wyswietl_macierz(CvMat* Macierz)
{
	int wiersze = Macierz->rows;
	int kolumny = Macierz->cols;
	for(int i=0; i< wiersze; i++)
	{		
		for(int j=0; j<kolumny; j++)
		{
			std::cout<<cvmGet(Macierz,i,j)<<"\t";
		}
		std::cout<< std::endl;
	}
	std::cout<< std::endl;
}

void wyswietl_macierz_transponowana(CvMat* Macierz)
{
        int wiersze = Macierz->rows;
        int kolumny = Macierz->cols;
        for(int i=0; i< kolumny; i++)
        {
                for(int j=0; j<wiersze; j++)
                {
                        std::cout<<cvmGet(Macierz,j,i)<<"\t";
                }
                std::cout<< std::endl;
        }
        std::cout<< std::endl;
}


int losuj(int max)
{
  unsigned int 
    num_bins = (unsigned long) max +1, 
    num_rand = (unsigned long) RAND_MAX +1, 
    bin_size = num_rand / num_bins,
    defect   = num_rand % num_bins;

  long x;
  do 
  {
    x = random();
  }
  while (num_rand - defect <= (unsigned long)x);
  return (int)x/bin_size;
}

/*
W necie ktos napisal
http://stackoverflow.com/questions/910423/which-is-faster-comparison-or-assignment

Well, since you say you're sure that this matters you should just write a test program and measure to find the difference.

Comparison can be faster if this code is executed on multiple variables allocated at scattered addresses in memory. With comparison you will only read data from memory to the processor cache, and if you don't change the variable value when the cache decides to to flush the line it will see that the line was not changed and there's no need to write it back to the memory. This can speed up execution.

*/
// dla malej ilosci liczb do losowania bez powtorzen takie podejscie jest szybsze
void losuj_3(int max, int* l_1, int* l_2, int* l_3) 
{
  int zajete_elementy_konca = 0;
  *l_1 = losuj(max); //zakladam, że zostalo przesuniete do zakresu [0, max-1]
  if (*l_1 == max)
  {
    ++zajete_elementy_konca;
  }
  *l_2 = losuj(max-1);
  if (*l_2 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_2)
  {
     *l_2 = max - zajete_elementy_konca;
     ++zajete_elementy_konca;
  }
  *l_3 = losuj(max-2);
  if (*l_1 == *l_3)
  {
     *l_3 = max - zajete_elementy_konca;
  }
  else if (*l_2 == *l_3)
  {
     *l_3 = max - zajete_elementy_konca;
  }
}

void losuj_5(int max, int* l_1, int* l_2, int* l_3, int* l_4, int* l_5) 
{
  int zajete_elementy_konca = 0;
  *l_1 = losuj(max); //zakladam, że zostalo przesuniete do zakresu [0, max-1]
  if (*l_1 == max)
  {
    ++zajete_elementy_konca;
  }
  *l_2 = losuj(max-1);
  if (*l_2 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_2)
  {
     *l_2 = max - zajete_elementy_konca;
     ++zajete_elementy_konca;
  }
  *l_3 = losuj(max-2);
  if (*l_3 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_3)
  {
    *l_3 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_3)
  {
    *l_3 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  *l_4 = losuj(max-3);
  if (*l_4 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  *l_5 = losuj(max-4);
  if (*l_5 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_4 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
}


void losuj_8(int max, int* l_1, int* l_2, int* l_3, int* l_4, int* l_5, int* l_6, int* l_7, int* l_8) 
{
  int zajete_elementy_konca = 0;
  *l_1 = losuj(max); //zakladam, że zostalo przesuniete do zakresu [0, max-1]
  if (*l_1 == max)
  {
    ++zajete_elementy_konca;
  }
  *l_2 = losuj(max-1);
  if (*l_2 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_2)
  {
     *l_2 = max - zajete_elementy_konca;
     ++zajete_elementy_konca;
  }
  *l_3 = losuj(max-2);
  if (*l_3 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_3)
  {
    *l_3 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_3)
  {
    *l_3 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  *l_4 = losuj(max-3);
  if (*l_4 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_4)
  {
    *l_4 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  *l_5 = losuj(max-4);
  if (*l_5 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_4 == *l_5)
  {
    *l_5 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  *l_6 = losuj(max-5);
  if (*l_6 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_6)
  {
    *l_6 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_6)
  {
    *l_6 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_6)
  {
    *l_6 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_4 == *l_6)
  {
    *l_6 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_5 == *l_6)
  {
    *l_6 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }


  *l_7 = losuj(max-6);
  if (*l_7 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_4 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_5 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_6 == *l_7)
  {
    *l_7 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }

  
  *l_8 = losuj(max-6);
  if (*l_7 == max - zajete_elementy_konca)
  {
    ++zajete_elementy_konca;
  }
  else if (*l_1 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_2 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_3 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_4 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_5 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_6 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
  else if (*l_7 == *l_8)
  {
    *l_8 = max - zajete_elementy_konca;
    ++zajete_elementy_konca;
  }
}


void kabsch(CvMat* points_mat_1, CvMat* points_mat_2, int point_count, CvMat* Rotation, CvMat* Translation)
{
/*
std::cout<<"points mat 1:\n\n";
wyswietl_macierz(points_mat_1);
std::cout<<"points mat 2:\n\n";
wyswietl_macierz(points_mat_2);
*/
  //centroids
  //P  
  double centroid_P_x = 0;
  double centroid_P_y = 0;
  double centroid_P_z = 0; 

  double centroid_Q_x = 0;
  double centroid_Q_y = 0;
  double centroid_Q_z = 0; 

  CvMat* Centroid_P = cvCreateMat( 3, 1, CV_64FC1);
  CvMat* Centroid_Q = cvCreateMat( 3, 1, CV_64FC1);

  //wyswietl_macierz(points_mat_2);

  for(int i = 0; i < point_count; ++i)
  {
    centroid_P_x += cvmGet(points_mat_1,0,i);
    centroid_P_y += cvmGet(points_mat_1,1,i);
    centroid_P_z += cvmGet(points_mat_1,2,i);
  }
  for(int i = 0; i < point_count; ++i)
  {
    centroid_Q_x += cvmGet(points_mat_2,i,0);
    centroid_Q_y += cvmGet(points_mat_2,i,1);
    centroid_Q_z += cvmGet(points_mat_2,i,2);
  }
  centroid_P_x =centroid_P_x/point_count;
  centroid_P_y =centroid_P_y/point_count;
  centroid_P_z =centroid_P_z/point_count;

  centroid_Q_x =centroid_Q_x/point_count;
  centroid_Q_y =centroid_Q_y/point_count;
  centroid_Q_z =centroid_Q_z/point_count;
//P-macierz punktów points_mat_1 po ich transpozycji przez centroid_P
  //cvMat* P= cvCreateMat(point_count, 3, CV_64FC1);
  CvMat* P  = cvCreateMat (3, point_count, CV_64FC1);
  for (int i = 0; i<point_count; ++i)
  {
    CV_MAT_ELEM(*P, double, 0, i) =cvmGet(points_mat_1,0,i)-centroid_P_x;
    CV_MAT_ELEM(*P, double, 1, i) =cvmGet(points_mat_1,1,i)-centroid_P_y;
    CV_MAT_ELEM(*P, double, 2, i) =cvmGet(points_mat_1,2,i)-centroid_P_z;
  }

//Q-macierz punktów points_mat_2 po ich transpozycji przez centroid_Q
  CvMat* Q = cvCreateMat (point_count, 3, CV_64FC1);

  for (int i = 0; i<point_count; ++i)
  {
    CV_MAT_ELEM(*Q, double, i, 0) =cvmGet(points_mat_2,i,0)-centroid_Q_x;
    CV_MAT_ELEM(*Q, double, i, 1) =cvmGet(points_mat_2,i,1)-centroid_Q_y;
    CV_MAT_ELEM(*Q, double, i, 2) =cvmGet(points_mat_2,i,2)-centroid_Q_z;
  }

  CV_MAT_ELEM(*Centroid_P, double, 0, 0) = centroid_P_x;
  CV_MAT_ELEM(*Centroid_P, double, 1, 0) = centroid_P_y;
  CV_MAT_ELEM(*Centroid_P, double, 2, 0) = centroid_P_z;

  CV_MAT_ELEM(*Centroid_Q, double, 0, 0) = centroid_Q_x;
  CV_MAT_ELEM(*Centroid_Q, double, 1, 0) = centroid_Q_y;
  CV_MAT_ELEM(*Centroid_Q, double, 2, 0) = centroid_Q_z;

  //std::cout<<"\nwiersze P: "<< P->rows<<" kolumny P: "<< P->cols<< std::endl;
  //std::cout<<"\nwiersze Q: "<< Q->rows<<" kolumny Q: "<< Q->cols<< std::endl;

  //wyswietl_macierz_transponowana(P);

  CvMat* H = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* U_T = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* S = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* V = cvCreateMat( 3, 3, CV_64FC1);
  //CvMat* R1 = cvCreateMat( 3, 3, CV_64FC1);
  //Mat r;
  //Mat H = Mat::zeros(3, 3, CV_64FC1);
  
  cvMatMul(P,Q,H);
  cvSVD(H,S,U_T,V,CV_SVD_U_T); //H=U*WV^T
  //std::cout<<"macierze P i Q:\n";
  //wyswietl_macierz(P); std::cout<<"\n\n\n";
  //wyswietl_macierz(Q);
  cvMatMul(V,U_T,Rotation);
  //wyswietl_macierz (U_T);

  // W nowym opencv cvMatMul to macro dla GEMM, CvMat jest przestarzałe
  // i trzymane tylko jako spadek po poprzednich wersjach. Lepiej używać Mat
  //cvGEMM(V, U_T, 1, NULL, 0, R1, 0);
  //aa= &V;
  //bb= &U_T;
  //wyswietl_macierz(R);
  //wyswietl_macierz(R1);
  //wyswietl_macierz(H);
  //Mat aa = cvarrToMat(V);
  //Mat bb = cvarrToMat(U_T);
  //r=aa*bb;
  //std::cout<<"macierz r:\n"<< r <<std::endl;
  //std::cout<<"wyznacznik R: "<<cvDet(Rotation)<<std::endl;
  //wyswietl_macierz(R);
  if(cvDet(Rotation)<0)
  {
    //std::cout<<" Kabsch w druga strone\n";
    //CV_MAT_ELEM(*Rotation, double, 0, 2) = -cvmGet(Rotation,0,2);
    //CV_MAT_ELEM(*Rotation, double, 1, 2) = -cvmGet(Rotation,1,2);
    //CV_MAT_ELEM(*Rotation, double, 2, 2) = -cvmGet(Rotation,2,2);

    //std::cout<<" Rotacja przed i po\n";
    //wyswietl_macierz (Rotation);
    CV_MAT_ELEM(*V, double, 0, 2) = -cvmGet(V,0,2);
    CV_MAT_ELEM(*V, double, 1, 2) = -cvmGet(V,1,2);
    CV_MAT_ELEM(*V, double, 2, 2) = -cvmGet(V,2,2);

    cvMatMul(V,U_T,Rotation);
    //wyswietl_macierz (Rotation);
  }


  //wyswietl_macierz(R);
  
  //cvMatMul(-1,Rotation, Temp);
  cvGEMM (Rotation, Centroid_P, -1, Centroid_Q, 1, Translation, 0);

 /* std::cout<<"\n Mnozenie ";
  cvMatMul(Rotation, Centroid_P,Temp1);  
  wyswietl_macierz(Temp1);
  cvGEMM(Rotation, Centroid_P,1,NULL,0,Temp2,0);
  wyswietl_macierz(Temp2);

  wyswietl_macierz(Centroid_P);
  wyswietl_macierz(Rotation);
  wyswietl_macierz(Centroid_Q);
*/
  cvReleaseMat(&Centroid_P);
  cvReleaseMat(&Centroid_Q);
  cvReleaseMat(&H);
  cvReleaseMat(&P);
  cvReleaseMat(&Q);
  cvReleaseMat(&U_T);
  cvReleaseMat(&S);
  cvReleaseMat(&V);
}

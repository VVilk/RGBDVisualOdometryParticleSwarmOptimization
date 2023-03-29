#include "../odometria/odometrie.cpp"

//max maksymalna liczba
void ransac_v2(CvMat* points_mat_transposed_1, CvMat* points_mat_2, int point_count, char* maska, double ejection_distance_tereshold, double fidelity, int max_ransac_iterations, int* liczba_inlierow, char* ransac_fail)
{
  CvMat* Rotation_test = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* Translation_test = cvCreateMat( 3, 1, CV_64FC1);
  CvMat* punkty_ransac_transpozycja_1 = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* punkty_ransac_2 = cvCreateMat( 3, 3, CV_64FC1);
  CvMat* col_mat_1 = cvCreateMat( 3, 1, CV_64FC1);
  CvMat* point_1_after = cvCreateMat(3 ,1, CV_64FC1);
  double odl = 0;
  int inliers_count = 0;
  int maxinliersbylo =0;
  int min_inliers=(int)((point_count*fidelity)+1);
  int ransac_iterations = 0;

  //losuje 3 punkty do kabsch'a
  int punkt_1, punkt_2, punkt_3;

  srand( time(NULL));

  while ((inliers_count < min_inliers) && (ransac_iterations != max_ransac_iterations))
  {
      losuj_3(point_count-1, &punkt_1, &punkt_2, &punkt_3);

    //Przekazuje punkt 1 do kabscha
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 0, 0) =cvmGet(points_mat_transposed_1, 0, punkt_1);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 1, 0) =cvmGet(points_mat_transposed_1, 1, punkt_1);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 2, 0) =cvmGet(points_mat_transposed_1, 2, punkt_1);

    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 0, 1) =cvmGet(points_mat_transposed_1, 0, punkt_2);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 1, 1) =cvmGet(points_mat_transposed_1, 1, punkt_2);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 2, 1) =cvmGet(points_mat_transposed_1, 2, punkt_2);

    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 0, 2) =cvmGet(points_mat_transposed_1, 0, punkt_3);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 1, 2) =cvmGet(points_mat_transposed_1, 1, punkt_3);
    CV_MAT_ELEM(* punkty_ransac_transpozycja_1, double, 2, 2) =cvmGet(points_mat_transposed_1, 2, punkt_3);

    //Przekazuje punkty 2 do kabscha
    CV_MAT_ELEM(* punkty_ransac_2, double, 0, 0) =cvmGet(points_mat_2, punkt_1, 0);
    CV_MAT_ELEM(* punkty_ransac_2, double, 0, 1) =cvmGet(points_mat_2, punkt_1, 1);
    CV_MAT_ELEM(* punkty_ransac_2, double, 0, 2) =cvmGet(points_mat_2, punkt_1, 2);

    CV_MAT_ELEM(* punkty_ransac_2, double, 1, 0) =cvmGet(points_mat_2, punkt_2, 0);
    CV_MAT_ELEM(* punkty_ransac_2, double, 1, 1) =cvmGet(points_mat_2, punkt_2, 1);
    CV_MAT_ELEM(* punkty_ransac_2, double, 1, 2) =cvmGet(points_mat_2, punkt_2, 2);

    CV_MAT_ELEM(* punkty_ransac_2, double, 2, 0) =cvmGet(points_mat_2, punkt_3, 0);
    CV_MAT_ELEM(* punkty_ransac_2, double, 2, 1) =cvmGet(points_mat_2, punkt_3, 1);
    CV_MAT_ELEM(* punkty_ransac_2, double, 2, 2) =cvmGet(points_mat_2, punkt_3, 2);

  
    //kabsch(points_mat_transposed_1, points_mat_2, point_count, Rotation_test, Translation_test);
    kabsch( punkty_ransac_transpozycja_1, punkty_ransac_2, 3, Rotation_test, Translation_test);
    inliers_count = 0;
    for (int i=0; i < point_count; ++i)
    {
      CV_MAT_ELEM(* col_mat_1, double, 0, 0) =cvmGet(points_mat_transposed_1, 0, i);
      CV_MAT_ELEM(* col_mat_1, double, 1, 0) =cvmGet(points_mat_transposed_1, 1, i);
      CV_MAT_ELEM(* col_mat_1, double, 2, 0) =cvmGet(points_mat_transposed_1, 2, i);
     
      cvGEMM (Rotation_test, col_mat_1, 1, Translation_test, 1, point_1_after, 0);
      odl = sqrt (
        pow(cvmGet( point_1_after, 0, 0) - cvmGet (points_mat_2, i, 0 ),2) +
        pow(cvmGet( point_1_after, 1, 0) - cvmGet (points_mat_2, i, 1 ),2) +
        pow(cvmGet( point_1_after, 2, 0) - cvmGet (points_mat_2, i, 2 ),2)
                 );
      if(odl< ejection_distance_tereshold)
      {
       *(maska+i) =1;
       ++inliers_count;
      }
      else
      {
       *(maska+i) =0;
      }
    }
    ++ransac_iterations;
    if(inliers_count> maxinliersbylo)
    {
      maxinliersbylo= inliers_count;
    } 
    //std::cout<<"inlairs count to:"<< inliers_count<<" point count: "<< point_count<< "\n";
  }
  if((inliers_count < min_inliers) && (ransac_iterations == max_ransac_iterations))
  {
    *liczba_inlierow = inliers_count;
    *ransac_fail = 1;
  }
  else
  {
    *ransac_fail = 0;
    *liczba_inlierow = inliers_count;
    //kabsch(points_inliers_mat_transposed_1, points_inliers_mat_2, inliers_count, Rotation, Translation);
  }
  cvReleaseMat(&Rotation_test);
  cvReleaseMat(&Translation_test);
  cvReleaseMat(&punkty_ransac_2);
  cvReleaseMat(&punkty_ransac_transpozycja_1);
  cvReleaseMat(&col_mat_1);
  cvReleaseMat(&point_1_after);
  //cvReleaseMat(&points_inliers_mat_1);
  //cvReleaseMat(&points_inliers_mat_transposed_1);
  //cvReleaseMat(&points_inliers_mat_2);
}

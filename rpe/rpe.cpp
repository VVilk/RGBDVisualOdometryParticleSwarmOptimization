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

void associate_stare (std::vector < std::pair <double, std::string>  > rgb,  std::vector < std::pair <double, std::string>  > depth,  std::vector < std::pair <double, std::string>  >& rgb_matched,  std::vector < std::pair <double, std::string>  >& depth_matched)
{
  //depth_stamps_and_filenames[0].second
  int idx_stare = -1;
  std::cout<<"liczbka klatek: depth:  "<< depth.size() << std::endl;
  std::cout<<"liczbka klatek: rgb:  "<< rgb.size() << std::endl;
  for (unsigned int i = 0 ; i < depth.size() ; ++i) 
  {
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[i].first<<std::endl;
    int poczatek = 0;
    double roznica = abs (rgb[0].first - depth[i].first) ;
    int najlepszy =  0 ;
    int koniec = rgb.size();
    int srodek = -1;
    int idx = 0;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        if (abs ( rgb[srodek].first- depth[i].first ) < roznica)
        {
          roznica = abs (rgb[srodek].first - depth[i].first);
          najlepszy = srodek;
        }
        if (depth[i].first == rgb[srodek].first)
        {
          idx = srodek;
          //std::cout<<"goto \n";
          goto mam_idx;
        }
        if  (rgb[srodek].first > depth[i].first)
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( abs(rgb[najlepszy-1].first - depth[i].first) < abs(rgb[najlepszy].first-depth[i].first) )
      {
        najlepszy = najlepszy - 1;
        idx = najlepszy;
      }
      if ( abs(rgb[najlepszy+1].first - depth[i].first) < abs(rgb[najlepszy].first-depth[i].first) )
      {
        najlepszy = najlepszy + 1;
        idx = najlepszy;
      }
    }
    mam_idx:        // skok jak znajde juz idx
    if (idx != idx_stare)
    {
      rgb_matched.push_back(rgb[idx]);
      depth_matched.push_back(depth[i]);
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[idx].first<<std::endl;
    }
    else
    { 
      //std::cout<<"idx_stare = idx roznica: " << roznica << std::endl;
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[idx].first<<std::endl;
    }
    idx_stare= idx;
  }
    std::cout<<"powiazane pary klatek:  "<<depth_matched.size() << std::endl;
}

void associate_delta (std::vector<double> stampy, double delta, std::vector < std::pair < unsigned int, unsigned int> >& delta_stamps)
{
  //depth_stamps_and_filenames[0].second
  std::cout<<"liczbka stampow1: "<< stampy.size() << std::endl;
  for (unsigned int i = 0 ; i < stampy.size() ; ++i) 
  {
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[i].first<<std::endl;
    int poczatek = 0;
    double roznica = abs (stampy[0] - stampy[delta]) ;
    int najlepszy =  0 ;
    int koniec = stampy.size();
    int srodek = -1;
    unsigned int idx = 0;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        if (abs ( stampy[srodek]- stampy[i]-delta ) < roznica)
        {
          roznica = abs (stampy[srodek] - stampy[i]-delta);
          najlepszy = srodek;
        }
        if (stampy[i]+delta == stampy[srodek])
        {
          idx = srodek;
          //std::cout<<"goto \n";
          goto mam_idx;
        }
        if  (stampy[srodek] > stampy[i]+delta)
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( abs(stampy[najlepszy-1] - stampy[i]-delta) < abs(stampy[najlepszy]-stampy[i]-delta) )
      {
        najlepszy = najlepszy - 1;
        idx = najlepszy;
      }
      if ( abs(stampy[najlepszy+1] - stampy[i]-delta) < abs(stampy[najlepszy]-stampy[i]-delta) )
      {
        najlepszy = najlepszy + 1;
        idx = najlepszy;
      }
    }
    mam_idx:        // skok jak znajde juz idx
    if (idx!=stampy.size()-1)
    {
      delta_stamps.push_back(std::make_pair(i,idx));
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[idx].first<<std::endl;
    }
    else
    { 
      //std::cout<<"idx_stare = idx roznica: " << roznica << std::endl;
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[idx].first<<std::endl;
    }
  }
    std::cout<<"powiazane pary klatek:  "<<delta_stamps.size() << std::endl;
}

void powiaz_delta (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > est,  std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt, double delta, double m_roznica_czasu, std::vector < std::tuple < unsigned int, unsigned int, unsigned int, unsigned int> >& delta_stamps)
{
  //depth_stamps_and_filenames[0][0].second
  //std::cout<<"liczbka stampow1: "<< est.size() << std::endl;
  for (unsigned int i = 0 ; i < est.size() ; ++i) 
  {
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i].first<< "   "<< rgb[i].first<<std::endl;
    int poczatek = 0;
    double roznica = std::abs (est[0][0] - est[i][0]-delta);
    int najlepszy =  0 ;
    int koniec = est.size();
    int srodek = -1;
    unsigned int idx = 0;

      //    std::cout<<"roznica: "<< roznica <<"\n";
      while (poczatek < koniec)
      {
        //std::cout<<"poczatek " << poczatek << " srodek: "<< srodek << " koniec " << koniec << "est[srodek][0]" << est[srodek][0] << " est[i][0]-delta " << est[i][0]-delta << "abs "<< std::abs ( est[srodek][0]- est[i][0]-delta ) << "nie abs "<< est[srodek][0]- est[i][0]-delta<<"roznica "<<roznica << "\n";
        //std::cout<<"poczatek " << poczatek << " srodek: "<< srodek << " koniec " << koniec << "\n";
        srodek  = floor((koniec + poczatek)/2);
        if (std::abs ( est[srodek][0]- est[i][0]-delta ) < roznica)
        {
          roznica = std::abs (est[srodek][0] - est[i][0]-delta);
          //std::cout<<"roznica: "<< roznica <<"\n";
          //std::cout<<"cos lepszego \n";
          najlepszy = srodek;
        }
        if (est[i][0]+delta == est[srodek][0])
        {
          idx = srodek;
          //std::cout<<"goto mam_idx \n";
          goto mam_idx;
        }
        if  (est[srodek][0] > est[i][0]+delta)
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( std::abs(est[najlepszy-1][0] - est[i][0]-delta) < std::abs(est[najlepszy][0]-est[i][0]-delta) )
      {
        najlepszy = najlepszy - 1;
        idx = najlepszy;
      }
      if ( std::abs(est[najlepszy+1][0] - est[i][0]-delta) < std::abs(est[najlepszy][0]-est[i][0]-delta) )
      {
        najlepszy = najlepszy + 1;
        idx = najlepszy;
      }
    }
    if (idx == est.size()-1)
    {
      //std::cout <<" idx - estsize -1 \n";
      continue;
    }
    mam_idx:        // skok jak znajde juz idx
    poczatek = 0;
    roznica = std::abs (gt[0][0] - est[i][0]) ;
    //std::cout <<i << " roznica " <<roznica << "gt- est " <<std::abs ( est[0][0] - gt[i][0] ) << " gt "<<  gt[i][0]<< "est " << est[0][0] <<"\n";
    najlepszy =  0 ;
    koniec = gt.size();
    srodek = -1;
    unsigned int idx_gt_1 = 0;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        //std::cout <<i << " roznica " <<roznica << "gt- est " <<std::abs ( gt[srodek][0]- est[i][0] ) << " gt "<<  gt[srodek][0]<< "est " << est[i][0] <<"\n";
        if (std::abs ( gt[srodek][0]- est[i][0] ) < roznica)
        {
          roznica = std::abs (gt[srodek][0] - est[i][0]);
          najlepszy = srodek;
          //std::cout <<i << " najlepszy \n";
        }
        if (est[i][0] == gt[srodek][0])
        {
          idx_gt_1 = srodek;
          //std::cout<<"goto mam_idx_gt_1 " << idx_gt_1 << "\n" ;
          goto mam_idx_gt_1;
        }
        if  (gt[srodek][0] > est[i][0])
        {
          //std::cout << "koniec \n";
          koniec = srodek;
        }
        else
        {
          //std::cout << "poczatek \n";
          poczatek = srodek + 1;
        }
        idx_gt_1 = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( std::abs(gt[najlepszy-1][0] - est[i][0]) < std::abs(gt[najlepszy][0]-est[i][0]) )
      {
        najlepszy = najlepszy - 1;
        idx_gt_1 = najlepszy;
      }
      if ( std::abs(gt[najlepszy+1][0] - est[i][0]) < std::abs(gt[najlepszy][0]-est[i][0]) )
      {
        najlepszy = najlepszy + 1;
        idx_gt_1 = najlepszy;
      }
    }

    mam_idx_gt_1:        // skok jak znajde juz idx

/*

    roznica = std::abs (gt[0][0] - est[i][0]-delta) ; // a nie est[i][0] - delta
    najlepszy =  0 ;
    koniec = gt.size();
    srodek = -1;
    unsigned int idx_gt_2 = 0;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        if (std::abs ( gt[srodek][0]- est[i][0]-delta ) < roznica)
        {
          roznica = std::abs (gt[srodek][0] - est[i][0]-delta);
          najlepszy = srodek;
        }
        if (est[i][0] + delta == gt[srodek][0])
        {
          idx_gt_2 = srodek;
          //std::cout<<"goto mam_idx_2 \n";
          goto mam_idx_gt_2;
        }
        if  (gt[srodek][0] > est[i][0] + delta)
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx_gt_2 = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( std::abs(gt[najlepszy-1][0] - est[i][0]-delta) < std::abs(gt[najlepszy][0]-est[i][0]-delta) )
      {
        najlepszy = najlepszy - 1;
        idx_gt_2 = najlepszy;
      }
      if ( std::abs(gt[najlepszy+1][0] - est[i][0]-delta) < std::abs(gt[najlepszy][0]-est[i][0]-delta) )
      {
        najlepszy = najlepszy + 1;
        idx_gt_2 = najlepszy;
      }
    }


*/



    poczatek = 0;
    roznica = std::abs (gt[0][0] - est[idx][0]) ; // a nie est[i][0] - delta
    najlepszy =  0 ;
    koniec = gt.size();
    srodek = -1;
    unsigned int idx_gt_2 = 0;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        if (std::abs ( gt[srodek][0]- est[idx][0] ) < roznica)
        {
          roznica = std::abs (gt[srodek][0] - est[idx][0]);
          najlepszy = srodek;
        }
        if (est[idx][0] == gt[srodek][0])
        {
          idx_gt_2 = srodek;
          //std::cout<<"goto mam_idx_2 \n";
          goto mam_idx_gt_2;
        }
        if  (gt[srodek][0] > est[idx][0])
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx_gt_2 = najlepszy;
    }
    if (najlepszy > 1)
    {
      if ( std::abs(gt[najlepszy-1][0] - est[idx][0]) < std::abs(gt[najlepszy][0]-est[idx][0]) )
      {
        najlepszy = najlepszy - 1;
        idx_gt_2 = najlepszy;
      }
      if ( std::abs(gt[najlepszy+1][0] - est[idx][0]) < std::abs(gt[najlepszy][0]-est[idx][0]) )
      {
        najlepszy = najlepszy + 1;
        idx_gt_2 = najlepszy;
      }
    }
    mam_idx_gt_2:        // skok jak znajde juz idx
    if ((std::abs(gt[idx_gt_1][0] - est[i][0]) <= m_roznica_czasu) && (std::abs(gt[idx_gt_2][0] - est[idx][0]) <= m_roznica_czasu))
    {
      delta_stamps.push_back(std::make_tuple(i,idx,idx_gt_1,idx_gt_2));
      //std::cout<<i<< " " << idx << " " << idx_gt_1 << " " << idx_gt_2 <<"\n";
    }
    //if ((std::abs(gt[idx_gt_1][0] - est[i][0]) > m_roznica_czasu) || (std::abs(gt[idx_gt_2][0] - est[idx][0]) > m_roznica_czasu))
   // {
   //   std::cout<<" I " << i << " " << idx << "\n"; // << std::abs(gt[idx_gt_1][0] - est[i][0]) << " " << std::abs(gt[idx_gt_2][0] - est[idx][0]) << "\n";
   // }
    else
    {
      //delta_stamps.push_back(std::make_tuple(i,idx,idx_gt_1,idx_gt_2));
      //std::cout<<i<< " " << idx << "\n";
      //std::cout<<" I " << i << " " << idx << " " << idx_gt_1 << " " << idx_gt_2<<"Maxymalna roznica czasu: " << m_roznica_czasu << " " <<  std::abs(gt[idx_gt_1][0] - est[i][0]) << " " << std::abs(gt[idx_gt_2][0] - est[idx][0]) << "\n";
    }
    ////std::cout<< i << " "<< idx << " " << idx_gt_1 << " " << idx_gt_2 << " "<< est[i][0]<< " "<< est[idx][0] << " " << gt[idx_gt_1][0] << " " << gt[idx_gt_2][0] <<"\n"; 
    //std::cout<<std::fixed<< std::setprecision(6)<<depth[i][0].first<< "   "<< rgb[idx][0].first<<std::endl;
  }
    //std::cout<<"powiazane pary klatek:  "<<delta_stamps.size() << "max r czasu: "<<  m_roznica_czasu<< std::endl;
}

void associate (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > est,  std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt,  std::vector<double> & est_matched, std::vector<double> & gt_matched, int pierwszy = 0, int ostatni = -1)
{
  if(ostatni == -1)
  {
    ostatni = est.size();
  }
  //depth_stamps_and_filenames[0].second
  int idx_stare = -1;
  int idx2_stare = -1;
  std::cout<<"liczbka klatek: oszacowanej:  "<< est.size() << std::endl;
  std::cout<<"liczbka klatek: gt:  "<< gt.size() << std::endl;
  for (unsigned int i = pierwszy ; i < ostatni ; ++i) 
  {
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i]<< "   "<< rgb[i]<<std::endl;
    //std::cout<<"est[i](0) := " << est[i](0) << "gt[i](0) := " << gt[i](0) << "\n";
    int poczatek = 0;
    int koniec = gt.size();
    int srodek = -1;
    double roznica = std::abs (est[i](0) - gt[0](0)) ;
    int najlepszy =  pierwszy ;
    int idx = pierwszy;
    int idx2=-1;
    double roznica_tmp;

      while (poczatek < koniec)
      {
        srodek  = floor((koniec + poczatek)/2);
        roznica_tmp = std::abs(est[i](0)- gt[srodek](0));
        //std::cout<<"roznica: " << roznica << " bez abs: "<< est[i](0) - gt[srodek](0) << " roznica tmp: "<< roznica_tmp << "costam: " << abs(3.141519) << " \n";
        if (roznica_tmp < roznica)
        {
          roznica = roznica_tmp;
          najlepszy = srodek;
        }
        if (gt[srodek](0) == est[i](0))
        {
          idx = i;
          idx2 = srodek;
          //std::cout<<"goto \n";
          goto mam_idx;
        }
        if  (gt[srodek](0) > est[i](0))
        {
          koniec = srodek;
        }
        else
        {
          poczatek = srodek + 1;
        }
        idx2 = najlepszy;
    }
    //std::cout<<" idx2: "<< idx2 << "\n";
    if (idx2 == idx2_stare)
    {
      //std::cout<<"continue \n";
      continue;
    }
    poczatek = pierwszy;
    koniec = ostatni;
    najlepszy = i;
    //std::cout<<" poczatek: "<< poczatek << " koniec: "<< koniec <<"\n";
    while (poczatek < koniec)
    {
      srodek  = floor((koniec + poczatek)/2);
      roznica_tmp = std::abs(est[srodek](0)- gt[idx2](0));
      if (roznica_tmp < roznica)
      {
        roznica = roznica_tmp;
        najlepszy = srodek;
      }
      if (gt[idx2](0) == est[srodek](0))
      {
        idx = srodek;
        //std::cout<<"goto \n";
        goto mam_idx;
      }
      if  (est[srodek](0) > gt[idx2](0))
      {
        koniec = srodek;
      }
      else
      {
        poczatek = srodek + 1;
      }
      idx = najlepszy;
    }
    mam_idx:        // skok jak znajde juz idx
    if ((idx != idx_stare) && (idx2 != idx2_stare))
    {
      est_matched.push_back(idx);
      gt_matched.push_back(idx2);
      //std::cout<<"estmatched [i](0) := " << est[idx](0) << " matchedgt[i](0) := " << gt[idx2](0) << "\n";

      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i]<< "   "<< rgb[idx]<<std::endl;
    }
    else
    { 
      //std::cout<<"idx_stare = idx roznica: " << roznica << std::endl;
      //std::cout<<std::fixed<< std::setprecision(6)<<depth[i]<< "   "<< rgb[idx]<<std::endl;
    }
    idx_stare= idx;
    idx2_stare= idx2;
  }
    std::cout<<"powiazane pary klatek:  "<< gt_matched.size() << std::endl;
}

double max_roznica_czasu (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  > gt)
{
  size_t size = gt.size();
  std::vector<double> roznice(size);
  //std::vector<double> roznice;
  for(size_t i = 0; i < size-1; ++i)
  {
    roznice[i] = gt[i+1][0]-gt[i][0];
    //std::cout<< "roznice "<< roznice[i] << "\n";
  }
  if(size % 2 == 0) //czyli nieparzysta ilosć elementów
  {
    size_t n = size / 2;
    nth_element(roznice.begin(), roznice.begin()+n, roznice.end());
    //std::cout<<" MEDIANA  1 : "<< roznice[n] << "\n";
    return 2*roznice[n];
  }
  else
  {
    double a,b;
    size_t n = floor(size / 2);
    //std::cout.precision(12);
    //a = roznice[0];
    //std::cout<<" A: " << a << "\n";
    //a = roznice[n];
    //std::cout<<" A: " << a << "\n";
    nth_element(roznice.begin(), roznice.begin()+n, roznice.end());
    a = roznice[n];
    b = *std::min_element(roznice.begin()+n+1, roznice.end());
    //std::cout<<" A: " << a <<" B: " << b << "\n";
    //std::cout<<" MEDIANA 2 : "<< (a+b)/ 2.0 << "\n";
    return (a+b);// nie dziele przez 2 bo i tak potem bym mnozyl
  }
}

void readFile_timestamp (std::string file , std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >& trans_kwat)
{
  char buffer [4096];
  char c;
  //double t1;
  //std::string name;
  //double tx , ty, tz, qx, qy, qz, qw;
  Eigen::Matrix<double,8,1>  wektor;

  std::ifstream iff (file.c_str());
  if(!iff)
  {
    std::cout << "Can't read " << file << std::endl;
    exit(1);
  }
  // ignore three header lines
  //iff.getline(buffer, sizeof(buffer));
  //iff.getline(buffer, sizeof(buffer));
  //iff.getline(buffer, sizeof(buffer));

  int gdzie_jestem;
  gdzie_jestem = iff.tellg();
  iff.get(c);
  iff.seekg (gdzie_jestem);
  std::cout.precision(27);
  // each line consists of the timestamp and the filename of the depth image
  while (!iff.eof())
  {
    iff>>std::ws;
    gdzie_jestem = iff.tellg();
    if(iff.get(c)) 
    {
      //std::cout<<" ok"; 
      iff.seekg (gdzie_jestem);
      //while (iff.get(c))                  // loop getting single characters
     // {
     //   std::cout << c <<" ";
     // }
      //goto m1;

      if(c != '#')
      {
        //long double time; std::string name; //w rpe z monachium korzystaja z floata/ float64 wiec double powinno starczyc
        //iff >> time >> name;
        iff >> wektor[0] >> wektor[1] >> wektor[2] >> wektor[3] >> wektor[4] >> wektor[5] >> wektor[6] >> wektor[7];
        //
        //
        //
        //tx , ty, tz, qx, qy, qz, qw;
        //time = roundl(time *1000000)/1000000;
        //time = floorl((time+0.5) *1000000)/1000000;
        //std::cout<<"czas: " << time << "     nazwa: "<< name << "\n";
        //v1[0] = t1;
        //v1[1] = 0.0;
        //std::cout<<std::fixed<< std::setprecision(17)<<time<<" ";
        trans_kwat.push_back( wektor );
      }
      else
      {
        iff.getline(buffer, sizeof(buffer));
        //std::cout << "pomijam linijke \n";
      }
    }
    else
    {
      break;
    }
  }
}

// nie wiem czemu ale nie dziala dla Matrix<double,4,4>
void transform44p1_stare (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >  & trans_kwat, std::vector < std::pair <double, Eigen::Matrix4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Matrix4d> >  >& T44p1)
{
  Eigen::Matrix<double,4,4>  Mat;
  for(std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >::iterator it= trans_kwat.begin(); it != trans_kwat.end(); ++it)
  {
    //std::cout<< (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << " " << (*it)[7] << "\n";

    double qx2 = (*it)[4]*(*it)[4];
    double qy2 = (*it)[5]*(*it)[5];
    double qz2 = (*it)[6]*(*it)[6];
    double qw2 = (*it)[7]*(*it)[7];
    double norma_kwaternionu = qx2+qy2+qz2+qw2;
    if( norma_kwaternionu <  std::numeric_limits<double>::epsilon())
    {
       std::cout<<"EPSILON DBL \n";
       Mat<< 1.0,  0.0,  0.0,  (*it)[1],
             0.0,  1.0,  0.0,  (*it)[2],
             0.0,  0.0,  1.0,  (*it)[3],
             0.0,  0.0,  0.0,   1.0;
    }
    else
    {
      double qx = (*it)[4]*sqrt(2.0/norma_kwaternionu); 
      double qy = (*it)[5]*sqrt(2.0/norma_kwaternionu); 
      double qz = (*it)[6]*sqrt(2.0/norma_kwaternionu); 
      double qw = (*it)[7]*sqrt(2.0/norma_kwaternionu); 
      //wszystko ponizej *2
      double qx2 = qx*qx;
      double qy2 = qy*qy;
      double qz2 = qz*qz;
      //double qw2 = qw*qw;
      double qxy = qx*qy;
      double qxz = qx*qz;
      double qxw = qx*qw;
      double qyz = qy*qz;
      double qyw = qy*qw;
      double qzw = qz*qw;
      //norma_kwaternionu;
      Mat<< 1-qy2-qz2,   qxy-qzw,   qxz+qyw, (*it)[1],
              qxy+qzw, 1-qx2-qz2,   qyz+qxw, (*it)[2],
              qxz-qyw,   qyz-qxw, 1-qx2-qy2, (*it)[3],
              0.0,       0.0,       0.0,     1.0;
    }
    T44p1.push_back( std::make_pair ((*it)[0], Mat) );

  }
}

void transform44p1 (std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >  & trans_kwat, std::vector < std::pair <double, Eigen::Matrix4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Matrix4d> >  >& T44p1, std::vector<double> vec_matched)
{
  Eigen::Matrix<double,4,4>  Mat;
  for(std::vector<double>::iterator it= vec_matched.begin(); it != vec_matched.end(); ++it)
  {
    //std::cout<< (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << " " << (*it)[7] << "\n";

    double qx2 = trans_kwat[*it](4)*trans_kwat[*it](4);
    double qy2 = trans_kwat[*it](5)*trans_kwat[*it](5);
    double qz2 = trans_kwat[*it](6)*trans_kwat[*it](6);
    double qw2 = trans_kwat[*it](7)*trans_kwat[*it](7);
    double norma_kwaternionu = qx2+qy2+qz2+qw2;
    //std::cout<<"Norma kwaternionu: "<< norma_kwaternionu <<"\n";
    if( norma_kwaternionu <  std::numeric_limits<double>::epsilon())
    {
       std::cout<<"EPSILON DBL \n";
       Mat<< 1.0,  0.0,  0.0,  trans_kwat[*it](1),
             0.0,  1.0,  0.0,  trans_kwat[*it](2),
             0.0,  0.0,  1.0,  trans_kwat[*it](3),
             0.0,  0.0,  0.0,   1.0;
    }
    else
    {
      double qx = trans_kwat[*it](4)*sqrt(2.0/norma_kwaternionu); 
      double qy = trans_kwat[*it](5)*sqrt(2.0/norma_kwaternionu); 
      double qz = trans_kwat[*it](6)*sqrt(2.0/norma_kwaternionu); 
      double qw = trans_kwat[*it](7)*sqrt(2.0/norma_kwaternionu); 
      //wszystko ponizej *2
      qx2 = qx*qx;
      qy2 = qy*qy;
      qz2 = qz*qz;
      qw2 = qw*qw;
      double qxy = qx*qy;
      double qxz = qx*qz;
      double qxw = qx*qw;
      double qyz = qy*qz;
      double qyw = qy*qw;
      double qzw = qz*qw;
      //norma_kwaternionu;
      Mat<< 1.0-qy2-qz2,     qxy-qzw,     qxz+qyw, trans_kwat[*it](1),
                qxy+qzw, 1.0-qx2-qz2,     qyz-qxw, trans_kwat[*it](2),
                qxz-qyw,     qyz+qxw, 1.0-qx2-qy2, trans_kwat[*it](3),
                0.0,         0.0,         0.0,     1.0;
    }
    T44p1.push_back( std::make_pair (trans_kwat[*it](0), Mat) );

  }
}

void transform44p1_1macierz (Eigen::Matrix<double,8,1> & trans_kwat, Eigen::Matrix4d& T44p1)
{
  Eigen::Matrix<double,4,4>  Mat;
  double qx2 = trans_kwat(4)*trans_kwat(4);
  double qy2 = trans_kwat(5)*trans_kwat(5);
  double qz2 = trans_kwat(6)*trans_kwat(6);
  double qw2 = trans_kwat(7)*trans_kwat(7);
  double norma_kwaternionu = qx2+qy2+qz2+qw2;
  //std::cout<<"Norma kwaternionu: "<< norma_kwaternionu <<"\n";
  if( norma_kwaternionu <  std::numeric_limits<double>::epsilon())
  {
     std::cout<<"EPSILON DBL \n";
     T44p1<< 1.0,  0.0,  0.0,  trans_kwat(1),
             0.0,  1.0,  0.0,  trans_kwat(2),
             0.0,  0.0,  1.0,  trans_kwat(3),
             0.0,  0.0,  0.0,   1.0;
  }
  else
  {
    double qx = trans_kwat(4)*sqrt(2.0/norma_kwaternionu); 
    double qy = trans_kwat(5)*sqrt(2.0/norma_kwaternionu); 
    double qz = trans_kwat(6)*sqrt(2.0/norma_kwaternionu); 
    double qw = trans_kwat(7)*sqrt(2.0/norma_kwaternionu); 
    //wszystko ponizej *2
    qx2 = qx*qx;
    qy2 = qy*qy;
    qz2 = qz*qz;
    qw2 = qw*qw;
    double qxy = qx*qy;
    double qxz = qx*qz;
    double qxw = qx*qw;
    double qyz = qy*qz;
    double qyw = qy*qw;
    double qzw = qz*qw;
    //norma_kwaternionu;
    T44p1<< 1.0-qy2-qz2,     qxy-qzw,     qxz+qyw, trans_kwat(1),
                qxy+qzw, 1.0-qx2-qz2,     qyz-qxw, trans_kwat(2),
                qxz-qyw,     qyz+qxw, 1.0-qx2-qy2, trans_kwat(3),
                0.0,         0.0,         0.0,     1.0;
  }
}





void ominus(Eigen::Matrix<double,4,4>& A, Eigen::Matrix<double,4,4>& B, Eigen::Matrix<double,4,4>& Wynik)
{
  //std::cout<< "A" << A << "\n";
  Wynik=A.inverse()*B;
  //std::cout<< "A invetse " << A.inverse() << "\n";
}

void readme_rpe()
{
	printf("Jestes noobem.... tobie po prostu nie da sie pomoc\n");
}


double rpe(std::string gt_txt, std::string szac_txt)
{
  /*
  if(argc > 1)
  {
    if(strcmp(argv[1], "help") == 0 ||strcmp(argv[1], "HELP") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "-H") == 0 || strcmp(argv[1], "h") == 0 || strcmp(argv[1], "H") == 0 )
    {
      readme_rpe();
      return -1;
    }
  }
  std::string gt_txt;
  std::string szac_txt;

  if( argc == 3 )
  {
    gt_txt = argv[1];
    szac_txt = argv[2];    
  }
  else
  {
    printf("za malo argumentow wejsciowych sprobuj: \n");
    readme_rpe();
    return -1;
  }
  args_count_ok:
  */

  //printf("gt_txt path: %s   ", gt_txt.c_str());
  //std::cout<<"szac_txt path: "<< szac_txt<< " \n";

  std::vector< std::pair<double, std::string> > rgb_stamps_and_filenames_tmp;
  std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >  M_szac, M_gt;

  //std::vector < std::pair <double, Eigen::Matrix4d>, Eigen::aligned_allocator<std::pair<double,Eigen::Matrix4d> >  > Oszacowana_T44p1, GT_T44p1; //macierz 4x4 + 1 (za timestampa w parze)

  readFile_timestamp (gt_txt, M_gt);
  readFile_timestamp (szac_txt, M_szac);

  std::vector<double> szac_matched;
  std::vector<std::pair<unsigned int, unsigned int> > delta_szac;
  std::vector<std::tuple<unsigned int, unsigned int, unsigned int, unsigned int> > szac_delta_gt;
  std::vector<double> gt_matched;

  //associate (M_szac, M_gt, szac_matched, gt_matched);
  //associate_delta(szac_matched, 1, delta_szac);
  double m_roznica_czasu = max_roznica_czasu(M_gt);
  //std::cout <<"max_roznica_czasu "<< m_roznica_czasu << "\n"; 
  powiaz_delta(M_szac, M_gt, 1, m_roznica_czasu, szac_delta_gt);

  //transform44p1 (M_szac, Oszacowana_T44p1, szac_matched);
  //transform44p1 (M_gt, GT_T44p1, gt_matched);

  Eigen::Matrix<double,4,4> Szac_0, Szac_1, Gt_0, Gt_1;
  Eigen::Matrix<double,4,4> Wzgledne_szac, Wzgledne_gt, Wynik;


  double miara1 = 0, kwadraty_do_rmse = 0;
  double m1 = 0;
  //unsigned int offset = 30;
  //std::cout<<" sszac_delta_gt size= " << szac_delta_gt.size() << "\n";
  for(unsigned int i = 0; i < szac_delta_gt.size(); ++i)
  {
    //std::cout<<"i: "<< i << " gt Matched: "<< gt_matched[i] << "szac: " << szac_matched[i] << "\n";
    //std::cout<< "szacowanie " << Oszacowana_T44p1[i].first << "\n" << Oszacowana_T44p1[i].second<<"\n";
    //std::cout<< "gt: " <<GT_T44p1[i].first  <<"\n" << GT_T44p1[i].second<<"\n";
    transform44p1_1macierz(M_szac[std::get<0>(szac_delta_gt[i])], Szac_0);
    transform44p1_1macierz(M_szac[std::get<1>(szac_delta_gt[i])], Szac_1);
    transform44p1_1macierz(M_gt[std::get<2>(szac_delta_gt[i])], Gt_0);
    transform44p1_1macierz(M_gt[std::get<3>(szac_delta_gt[i])], Gt_1);
    ominus(Szac_1, Szac_0, Wzgledne_szac);
    ominus(Gt_1, Gt_0, Wzgledne_gt);
    //std::cout<< "Wzgledne szacowanie\n" << Wzgledne_szac<<"\n";
    //std::cout<< "Wzgledne gt\n" << Wzgledne_gt<<"\n";
    ominus(Wzgledne_szac, Wzgledne_gt, Wynik);
    m1 = std::sqrt(Wynik(0,3)*Wynik(0,3) + Wynik(1,3)*Wynik(1,3) + Wynik(2,3)*Wynik(2,3));
    kwadraty_do_rmse += Wynik(0,3)*Wynik(0,3) + Wynik(1,3)*Wynik(1,3) + Wynik(2,3)*Wynik(2,3);
    miara1 += std::sqrt(Wynik(0,3)*Wynik(0,3) + Wynik(1,3)*Wynik(1,3) + Wynik(2,3)*Wynik(2,3));
    //std::cout<<" pozycja szac: "<< Oszacowana_T44p1[i].second(0,3) <<" " << Oszacowana_T44p1[i].second(1,3) <<" " << Oszacowana_T44p1[i].second(2,3) <<"\n ";
    //std::cout<<" cala macierz: "<< Oszacowana_T44p1[i].second;

  //std::cout<< i <<"  m1 " << m1 << "Miara1  "<< miara1 << "\n";
  //std::cout<<" \n "<< i << " " << miara1 << "\n" << Wynik<<" Wynik\n";
  }

  double rmse, Mean;
  Mean = std::abs(miara1)/szac_delta_gt.size();
  rmse = std::sqrt(kwadraty_do_rmse/(szac_delta_gt.size()));
  //std::cout<< "m1 co by bylo uzyte(kompilator narzeka ....): " << m1 << "Miara1: " << miara1 << " Mean " << Mean << " rmse " << rmse <<  "\n";
  std::cout<< "rmse: " << rmse <<  "\n";

  /*
  std::cout<<"----------------------------------\n";
  for(std::vector < Eigen::Matrix<double,8,1>, Eigen::aligned_allocator<Eigen::Matrix<double,8,1> >  >::iterator it= M_v1.begin(); it != M_v1.end(); ++it)
  {
    std::cout<< (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << " " << (*it)[3] << " " << (*it)[4] << " " << (*it)[5] << " " << (*it)[6] << " " << (*it)[7] << "\n";
  }
  */
  
  return rmse;

}

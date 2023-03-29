#include <stdio.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <sys/time.h>
#include <sys/stat.h> // dla czy da sie otworzyc plik

#include <cfloat> // DBL_MAX
#include <cmath> // std::nextafter
#include <random>


//#include "opencv2/core.hpp"
//#include "opencv2/features2d.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/xfeatures2d.hpp"
//#include <opencv2/calib3d/calib3d.hpp>
//#include "opencv/cv.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

//#include "odometria/odometrie.cpp"   includuja to ransaci
#include "odometria/cala_petla.cpp"
//#include "rpe/hello.h"
#include "rpe/rpe.h"


#include <stdio_ext.h> // dla fpurge by czyscic strumien z klawiatury 
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>

#include <unistd.h> // dla sleep
#include <thread>
#include <pthread.h> // bo gcc spelnia standard c11, czyli watki z c11, poprzez biblioteke pthread. Moze byc uzyta inna ale na linuxie i tak z niej korzystamy na 99%
#include <sstream>

//pozyczone z https://www.gnu.org/software/libc/manual/html_node/Calculating-Elapsed-Time.html
int timeval_subtract (struct timeval *result, struct timeval *x, struct timeval *y)
{
  /* Perform the carry for the later subtraction by updating y. */
  if (x->tv_usec < y->tv_usec) {
    int nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
    y->tv_usec -= 1000000 * nsec;
    y->tv_sec += nsec;
  }
  if (x->tv_usec - y->tv_usec > 1000000) {
    int nsec = (x->tv_usec - y->tv_usec) / 1000000;
    y->tv_usec += 1000000 * nsec;
    y->tv_sec -= nsec;
  }

  /* Compute the time remaining to wait.
     tv_usec is certainly positive. */
  result->tv_sec = x->tv_sec - y->tv_sec;
  result->tv_usec = x->tv_usec - y->tv_usec;

  /* Return 1 if result is negative. */
  return x->tv_sec < y->tv_sec;
}


int getkey() {
    
/*	int ch;
    while(ch = getchar() != EOF)
    {
	    printf("%s", getchar());
      if(ch = getchar() == EOF) break;
    }*/
    //__fpurge();
    int character;
    struct termios orig_term_attr;

    struct termios new_term_attr;

    /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);
    /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
    character = fgetc(stdin);
    /* restore the original terminal attributes */
    tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

    return character;
}


void clean_stdin(void)
{
    struct termios orig_term_attr;
    struct termios new_term_attr;
   /* set the terminal to raw mode */
    tcgetattr(fileno(stdin), &orig_term_attr);
    memcpy(&new_term_attr, &orig_term_attr, sizeof(struct termios));
    new_term_attr.c_lflag &= ~(ECHO|ICANON);
    new_term_attr.c_cc[VTIME] = 0;
    new_term_attr.c_cc[VMIN] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &new_term_attr);
   /* read a character from the stdin stream without blocking */
    /*   returns EOF (-1) if no character is available */
     while (fgetc(stdin) != EOF)
     {
       if (fgetc(stdin) == EOF) break;
     }
       /* restore the original terminal attributes */
  tcsetattr(fileno(stdin), TCSANOW, &orig_term_attr);

}



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
/*
//fr1
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

//dataset 1 kin 1 putkk
/*double fx = 532.77250;
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

void readme();
//void wyswietl_macierz_transponowana(CvMat* Macierz);


void readFile (std::string folder, std::string rgb_txt , std::vector < std::string  >& klatki)
{
  FILE *file_txt;
  file_txt=fopen(rgb_txt.c_str(),"r");
  int znak_plik;
  std::string bufor_klatka;
  bufor_klatka.reserve(32);

  if (file_txt != NULL)
  {
    int ignoruj_komentarz = 0;
    int ignoruj_time_stamp = 1;
    while((znak_plik=getc(file_txt)) != EOF )
    {
      if (znak_plik == '#')
      {
        ignoruj_komentarz=1;
        continue;
      }
      if (ignoruj_komentarz == 1)
      {
        if (znak_plik == '\n')
        {
          ignoruj_komentarz=0;
        }
      }
      else
      {
        if(ignoruj_time_stamp == 1)
        {
         if(znak_plik == ' ')
         {
           ignoruj_time_stamp = 0;
         }
        }
        else
        {
          if(znak_plik == '\n')
          {
            ignoruj_time_stamp = 1;
           // aktualna_klatka_rgb = folder + "/" + bufor_klatka_rgb;
            klatki.push_back(folder + "/" + bufor_klatka);
            bufor_klatka.clear();
           }
           else
           {
             bufor_klatka += znak_plik;
           }
        }
      }
    }
  }
  fclose(file_txt);
}

/*void readFile_timestamp (std::string folder, std::string file , std::vector < std::pair <double, std::string>  >& klatki)
{

  char buffer [4096];
  std::ifstream iff (file.c_str());
  if(!iff)
  {
    std::cout << "Can't read " << file << std::endl;
    exit(1);
  }
  // ignore three header lines
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));

  // each line consists of the timestamp and the filename of the depth image
  while (!iff.eof())
  {
    double time; std::string name;
    iff >> time >> name;
    //std::cout<<std::fixed<< std::setprecision(17)<<time<<" ";
    klatki.push_back( std::make_pair (time, name));
    //std::cout<<time<< " " << name<< std::endl;
  }

}*/

void readFile_timestamp (std::string folder, std::string file , std::vector < std::pair <double, std::string>  >& klatki)
{

  char buffer [4096];
  std::ifstream iff (file.c_str());
  if(!iff)
  {
    std::cout << "Can't read " << file << std::endl;
    exit(1);
  }
  // ignore three header lines
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));
  iff.getline(buffer, sizeof(buffer));
  double time; std::string name;

  // each line consists of the timestamp and the filename of the depth image
  while (iff>>time>>name)
  {
    //std::cout<<std::fixed<< std::setprecision(17)<<time<<" ";
    klatki.push_back( std::make_pair (time, name));
    //std::cout<<time<< " " << name<< std::endl;
  }

}
/*
 *Znalezione w necie.... zmieniam trochę funkcję
 *
 INPUT FILE
 Miller Andrew 65789.87 5
Green Sheila 75892.56 9
Sethi Amit 74900.50 6.1

ifstream inFile;
ofstream outFile;
string laastName;
string firstName;
double salary;
double percent;
double new Salary;
double increase;

inFile.open("Ch3_Ex6Data.txt");
outFile.open("Ch3_Ex6Output.dat");

while(!inFile.eof()) {

    inFile >> lastName;
    inFile >> firstName;
    inFile >> salary;
    inFile >> percent;

    percent /= 100;
    increase = salary * percent;
    newSalary = increase + salary;

    outFile << firstName << " " << lastName << " ";
    outFile << setprecision(2) << fixed << newSalary << endl;

}

inFile.close();
outFile.close();

return 0
}

Output File

Andrew Miller 69079.36
Sheila Green 82722.89
Amit Sethi 79469.43
Amit Sethi 74946.19

My question is why is the last line getting output twice and why is it different than the first one? I don't understand why the loop continues on. Is the end of file marker not hitting? I was able to hard code it by putting in an index variable and putting a second condition into the while loop by saying && less than index but i feel as if i shouldn't have to do that whatsoever.


Zaakceptowana odpowiedź:
The problem is that eof does not do what you think it does.

Imagine you are walking on a floor, tile after tile, picking up what's on the floor, putting it in your pockets and show up (print) your pockets content.

When you put you feet on the last tile the floor is not yet "ended" and your nose is still safe. You have not (yet) smashed the wall. You fill-up the pockets and print them.

eof,then, tells you when your nose is broken, not when the tile is the last.

So, you are on the last tile, check your nose, find it ok, and go one step forward. Your nose is now bleeding, nothing is there to peek up to put in our pockets, and your pockets still contain ... what they had before.

You print the content of your pocket (once again) and than check your nose. It's broken: you exit.

The idiomatic way to solve that problem is this one:

while(inFile >> lastName
             >> firstName
             >> salary
             >> percent)
{
   //all your computation here
}

I think you should understand by yourself why.

 
 
 */

void associate (std::vector < std::pair <double, std::string>  > rgb,  std::vector < std::pair <double, std::string>  > depth,  std::vector < std::pair <double, std::string>  >& rgb_matched,  std::vector < std::pair <double, std::string>  >& depth_matched)
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



// w c wlono char (*maska)[]
double daj_miare_ate_rmse(std::string* folder, std::string* result_folder, std::string* trajektoria)
{
  FILE* file;
  double interesujaca_mnie_miara = 100000;
  char path[1035];
  std::string linia;
  std::stringstream calosc;
  std::stringstream ss;

  std::string name;
  double miara;
  char jednostki;
  std::string pairs; //bo dla 1 linijki char nie dziala ale jej nie potrzebuje

  std::string komenda = "/home/alek/ate.py --verbose " + *folder +"/groundtruth.txt " + *result_folder +"trajectory_" + *trajektoria + ".txt";
  std::cout<<komenda<<std::endl;
  //char kkomenda [] = "/home/ja/ate.py --verbose ~/Pulpit/rgbd_dataset_freiburg1_desk/groundtruth.txt /home/ja/studia/pso/test/trajectory.txt --plot //home/ja/studia/c_2_python/ate_test.eps"; 

  file = popen(komenda.c_str(),"r");
  while (fgets(path, sizeof(path)-1, file) != NULL) 
  {
    calosc << path;
  }
  int i=0;
  while(std::getline(calosc,linia))
  {
    ss << linia;
      if(i == 0)
      {
        ss >> name >> miara >> pairs;
        std::cout << std::endl << name << " "<< miara << " " << pairs << std::endl;
	ss.str(std::string());
	ss.clear();
	++i;
      }
      else
      {
        ss >> name >> miara >> jednostki;
        //std::cout << "nazwa: " << name << " miara: " << miara << " jednostki: " << jednostki << std::endl;
        std::cout << name << " " << miara << " " << jednostki << std::endl;
        if( i == 1)
        {
          interesujaca_mnie_miara = miara;
        }
      ++i;
      }
  }
  pclose(file);
  return interesujaca_mnie_miara;
}


//int plik_txt_istnieje(const char * filename)
//{
//    /* try to open file to read */
//    struct stat st;
//    int result = stat (filename, &st);
//    return result == 0;
//}

int plik_txt_istnieje(const char * filename)
{
    /* try to open file to read */
    FILE *file;
    if (file = fopen(filename, "r"))
    {
      fclose(file);
      return 1;
    }
  return 0;
}

struct polozenie
{
  double max_polozenie_error_threshold1;
  double max_polozenie_pewnosc_1;
  double max_polozenie_error_threshold2;
  double max_polozenie_pewnosc_2;
  float  max_polozenie_detector_thresh;
  float  max_polozenie_akaze_kcontrast_percentile;

  double min_polozenie_error_threshold1;
  double min_polozenie_pewnosc_1;
  double min_polozenie_error_threshold2;
  double min_polozenie_pewnosc_2;
  float  min_polozenie_detector_thresh;
  float  min_polozenie_akaze_kcontrast_percentile;
};

struct predkosc
{
  double max_predkosc_error_threshold1;
  double max_predkosc_pewnosc_1;
  double max_predkosc_error_threshold2;
  double max_predkosc_pewnosc_2;
  float  max_predkosc_detector_thresh;
  float  max_predkosc_akaze_kcontrast_percentile;

  double min_predkosc_error_threshold1;
  double min_predkosc_pewnosc_1;
  double min_predkosc_error_threshold2;
  double min_predkosc_pewnosc_2;
  float  min_predkosc_detector_thresh;
  float  min_predkosc_akaze_kcontrast_percentile;
};

struct parametry
{
  int    status; // 0- niepoliczone, 1- licze, 2-policzone i zapisane
  int    flann_param;
  double error_threshold1;
  double pewnosc_1;
  double error_threshold2;
  double pewnosc_2;
  float  detector_thresh;
  float  akaze_kcontrast_percentile;
  double miara; // 0- niepoliczone, 1- licze, 2-policzone i zapisane

  double naj_error_threshold1;
  double naj_pewnosc_1;
  double naj_error_threshold2;
  double naj_pewnosc_2;
  float  naj_detector_thresh;
  float  naj_akaze_kcontrast_percentile;
  double naj_miara; // 0- niepoliczone, 1- licze, 2-policzone i zapisane

  double predkosc_error_threshold1;
  double predkosc_pewnosc_1;
  double predkosc_error_threshold2;
  double predkosc_pewnosc_2;
  float  predkosc_detector_thresh;
  float  predkosc_akaze_kcontrast_percentile;

};

struct parametry_naj
{
  int    flann_param;
  double error_threshold1;
  double pewnosc_1;
  double error_threshold2;
  double pewnosc_2;
  float  detector_thresh;
  float  akaze_kcontrast_percentile;
  double miara; // 0- niepoliczone, 1- licze, 2-policzone i zapisane
};

void readsave_pso_file (std::string file, int* koniec, int* liczba_czastek_pso, int* iteracje, int* iteracje_max ,int* nr, parametry (*parametry_pso), parametry_naj (*parametry_pso_naj))
{
  char buffer [4096];
  std::ifstream iff (file.c_str());
  if(!iff)
  {
    std::cout << "Can't read " << file << std::endl;
    exit(1);
  }
  // ignoruj 1 linie - naglowek
  iff.getline(buffer, sizeof(buffer));
  //iff.getline(buffer, sizeof(buffer));
  //iff.getline(buffer, sizeof(buffer));
  int i = 0;
  // each lin
  if (!iff.eof())
  {
  iff >> *koniec >> *liczba_czastek_pso >> *iteracje >> *iteracje_max >> *nr;
  std::cout<< "nagowek" << *koniec << " " <<  *liczba_czastek_pso << " " << *iteracje << " " << *iteracje_max<< " " << *nr << std::endl;
  iff >>  parametry_pso_naj->error_threshold1 >> parametry_pso_naj->pewnosc_1 >> parametry_pso_naj->error_threshold2 >> parametry_pso_naj->pewnosc_2 >> parametry_pso_naj->detector_thresh >> parametry_pso_naj->akaze_kcontrast_percentile  >> parametry_pso_naj->flann_param >> parametry_pso_naj->miara;
  std::cout <<  parametry_pso_naj->error_threshold1 << parametry_pso_naj->pewnosc_1 << parametry_pso_naj->error_threshold2 << parametry_pso_naj->pewnosc_2 << parametry_pso_naj->detector_thresh << parametry_pso_naj->akaze_kcontrast_percentile  << parametry_pso_naj->flann_param << parametry_pso_naj->miara << std::endl;
  }

  // each line consists of the timestamp and the filename of the depth image
  while ((!iff.eof()) && (i < *liczba_czastek_pso))
  {
 
    iff >> i >> std::fixed >> std::setprecision(6) >> parametry_pso->error_threshold1 >> parametry_pso->pewnosc_1 >> parametry_pso->error_threshold2 >> parametry_pso->pewnosc_2 >> parametry_pso->detector_thresh >> parametry_pso->akaze_kcontrast_percentile >> parametry_pso->flann_param >> parametry_pso->naj_error_threshold1 >> parametry_pso->naj_pewnosc_1 >> parametry_pso->naj_error_threshold2 >> parametry_pso->naj_pewnosc_2 >> parametry_pso->naj_detector_thresh >> parametry_pso->naj_akaze_kcontrast_percentile  >> parametry_pso->naj_miara >>  parametry_pso->predkosc_error_threshold1 >> parametry_pso->predkosc_pewnosc_1 >> parametry_pso->predkosc_error_threshold2 >> parametry_pso->predkosc_pewnosc_2 >> parametry_pso->predkosc_detector_thresh >> parametry_pso->predkosc_akaze_kcontrast_percentile  >>parametry_pso->status >> parametry_pso->miara;
    //iff >> i >> parametry_pso->error_threshold1 >> parametry_pso->pewnosc_1 >> (*parametry_pso).error_threshold2 >> (*parametry_pso).pewnosc_2 >> (*parametry_pso).detector_thresh >> (*parametry_pso).flann_param >> (*parametry_pso).status >> (*parametry_pso).miara;
  //std::cout <<  parametry_pso->error_threshold1 << " " << parametry_pso->pewnosc_1 << " " << parametry_pso->error_threshold2 << " " << parametry_pso->pewnosc_2 << " " << parametry_pso->detector_thresh << " " << parametry_pso->flann_param << " " << parametry_pso->miara<< "\n";
    std::cout << i << " " << parametry_pso->error_threshold1 << " " << parametry_pso->pewnosc_1 << " " << parametry_pso->error_threshold2 << " " << parametry_pso->pewnosc_2 << " " << parametry_pso->detector_thresh << " " << parametry_pso->akaze_kcontrast_percentile << " " << parametry_pso->flann_param << " " << parametry_pso->naj_error_threshold1 << " " << parametry_pso->naj_pewnosc_1 << " " << parametry_pso->naj_error_threshold2 << " " <<  parametry_pso->naj_pewnosc_2 << " " << parametry_pso->naj_detector_thresh << " " << parametry_pso->naj_akaze_kcontrast_percentile<< " " << parametry_pso->naj_miara << " " <<  parametry_pso->predkosc_error_threshold1 << " " << parametry_pso->predkosc_pewnosc_1 << " " << parametry_pso->predkosc_error_threshold2 << " " << parametry_pso->predkosc_pewnosc_2 << " " << parametry_pso->predkosc_detector_thresh << " " << parametry_pso->predkosc_akaze_kcontrast_percentile << " " << parametry_pso->status << " " << parametry_pso->miara << std::endl;
    //std::cout<< i <<" " <<parametry_pso->error_threshold1 <<" " << parametry_pso->pewnosc_1 <<" " << (*parametry_pso).error_threshold2 <<" " << (*parametry_pso).pewnosc_2 << " " << (*parametry_pso).detector_thresh <<" "<< (*parametry_pso).flann_param <<" "<< (*parametry_pso).status <<" "<< (*parametry_pso).miara<<"\n";
    ++i;
    ++parametry_pso;
  }
}

void aktualizuj_parametry (parametry (*parametry_pso), parametry_naj (*parametry_pso_naj), polozenie (*p_max), predkosc (*v_max), std::mt19937 *gen)
{
  //double c1 = std::uniform_real_distribution<double> dis(start, std::nextafter(stop, DBL_MAX));

  if(parametry_pso->miara < parametry_pso->naj_miara)
  {
    parametry_pso->naj_error_threshold1 = parametry_pso->error_threshold1;
    parametry_pso->naj_pewnosc_1 = parametry_pso->pewnosc_1;
    parametry_pso->naj_error_threshold2 = parametry_pso->error_threshold2;
    parametry_pso->naj_pewnosc_2 = parametry_pso->pewnosc_2;
    parametry_pso->naj_detector_thresh = parametry_pso->detector_thresh;
    //parametry_pso->naj_akaze_kcontrast_percentile = parametry_pso->akaze_kcontrast_percentile;
    parametry_pso->naj_miara = parametry_pso->miara;
  }


  std::uniform_real_distribution<double> dis(0, std::nextafter(1, DBL_MAX));
  std::uniform_real_distribution<float> dis_det(0, std::nextafter(1, FLT_MAX));
  /*for (int i=0; i<100; ++i)
  {
    printf(" %f", dis(*gen));
  }*/
  parametry_pso->predkosc_error_threshold1 = parametry_pso->predkosc_error_threshold1 + 2*dis(*gen)*(parametry_pso->naj_error_threshold1 - parametry_pso->error_threshold1)  +2*dis(*gen)*(parametry_pso_naj->error_threshold1 -parametry_pso->error_threshold1);
  parametry_pso->predkosc_pewnosc_1 = parametry_pso->predkosc_pewnosc_1 + 2*dis(*gen)*(parametry_pso->naj_pewnosc_1 - parametry_pso->pewnosc_1)  +2*dis(*gen)*(parametry_pso_naj->pewnosc_1 -parametry_pso->pewnosc_1);
  parametry_pso->predkosc_error_threshold2 = parametry_pso->predkosc_error_threshold2 + 2*dis(*gen)*(parametry_pso->naj_error_threshold2 - parametry_pso->error_threshold2)  +2*dis(*gen)*(parametry_pso_naj->error_threshold2 -parametry_pso->error_threshold2);
  parametry_pso->predkosc_pewnosc_2 = parametry_pso->predkosc_pewnosc_2 + 2*dis(*gen)*(parametry_pso->naj_pewnosc_2 - parametry_pso->pewnosc_2)  +2*dis(*gen)*(parametry_pso_naj->pewnosc_2 -parametry_pso->pewnosc_2);
  //std::cout<<" predkosc detector: " <<  parametry_pso->predkosc_detector_thresh << " " << parametry_pso->naj_detector_thresh << " " <<parametry_pso_naj->detector_thresh << std::endl;
  parametry_pso->predkosc_detector_thresh = parametry_pso->predkosc_detector_thresh + 2*dis_det(*gen)*(parametry_pso->naj_detector_thresh - parametry_pso->detector_thresh)  +2*dis_det(*gen)*(parametry_pso_naj->detector_thresh -parametry_pso->detector_thresh);
  //parametry_pso->predkosc_akaze_kcontrast_percentile = parametry_pso->predkosc_akaze_kcontrast_percentile + 2*dis_det(*gen)*(parametry_pso->naj_akaze_kcontrast_percentile - parametry_pso->akaze_kcontrast_percentile)  +2*dis_det(*gen)*(parametry_pso_naj->akaze_kcontrast_percentile -parametry_pso->akaze_kcontrast_percentile);

  //std::cout << parametry_pso->predkosc_error_threshold1 << std::endl;

  if(parametry_pso->predkosc_error_threshold1  >  v_max->max_predkosc_error_threshold1)
  {
    parametry_pso->predkosc_error_threshold1  = v_max->max_predkosc_error_threshold1;
  //std::cout << parametry_pso->predkosc_error_threshold1 << "+++++\n";
  }
  if(parametry_pso->predkosc_error_threshold1  <  v_max->min_predkosc_error_threshold1)
  {
    parametry_pso->predkosc_error_threshold1  = v_max->min_predkosc_error_threshold1;
  //std::cout << parametry_pso->predkosc_error_threshold1 << "-----\n";
  }
  if(parametry_pso->predkosc_pewnosc_1  >  v_max->max_predkosc_pewnosc_1)
  {
    parametry_pso->predkosc_pewnosc_1  = v_max->max_predkosc_pewnosc_1;
  }
  if(parametry_pso->predkosc_pewnosc_1  <  v_max->min_predkosc_pewnosc_1)
  {
    parametry_pso->predkosc_pewnosc_1  = v_max->min_predkosc_pewnosc_1;
  }
  if(parametry_pso->predkosc_error_threshold2  >  v_max->max_predkosc_error_threshold2)
  {
    parametry_pso->predkosc_error_threshold2  = v_max->max_predkosc_error_threshold2;
  }
  if(parametry_pso->predkosc_error_threshold2  <  v_max->min_predkosc_error_threshold2)
  {
    parametry_pso->predkosc_error_threshold2  = v_max->min_predkosc_error_threshold2;
  }
  if(parametry_pso->predkosc_pewnosc_2  >  v_max->max_predkosc_pewnosc_2)
  {
    parametry_pso->predkosc_pewnosc_2  = v_max->max_predkosc_pewnosc_2;
  }
  if(parametry_pso->predkosc_pewnosc_2  <  v_max->min_predkosc_pewnosc_2)
  {
    parametry_pso->predkosc_pewnosc_2  = v_max->min_predkosc_pewnosc_2;
  }
  if(parametry_pso->predkosc_detector_thresh  >  v_max->max_predkosc_detector_thresh)
  {
    parametry_pso->predkosc_detector_thresh  = v_max->max_predkosc_detector_thresh;
  }
  if(parametry_pso->predkosc_detector_thresh  <  v_max->min_predkosc_detector_thresh)
  {
    parametry_pso->predkosc_detector_thresh  = v_max->min_predkosc_detector_thresh;
  }
  /*
  if(parametry_pso->predkosc_akaze_kcontrast_percentile  >  v_max->max_predkosc_akaze_kcontrast_percentile)
  {
    parametry_pso->predkosc_akaze_kcontrast_percentile = v_max->max_predkosc_akaze_kcontrast_percentile;
  }
  if(parametry_pso->predkosc_akaze_kcontrast_percentile <  v_max->min_predkosc_akaze_kcontrast_percentile)
  {
    parametry_pso->predkosc_akaze_kcontrast_percentile  = v_max->min_predkosc_akaze_kcontrast_percentile;
  }
  */

  // narazie zostawiam bo chce widziec czy zostaje to w miejscu
  parametry_pso->error_threshold1 = parametry_pso->error_threshold1 + parametry_pso->predkosc_error_threshold1;
  parametry_pso->pewnosc_1 = parametry_pso->pewnosc_1 + parametry_pso->predkosc_pewnosc_1;
  parametry_pso->error_threshold2 = parametry_pso->error_threshold2 + parametry_pso->predkosc_error_threshold2;
  parametry_pso->pewnosc_2 = parametry_pso->pewnosc_2 + parametry_pso->predkosc_pewnosc_2;
  parametry_pso->detector_thresh = parametry_pso->detector_thresh + parametry_pso->predkosc_detector_thresh;
  //parametry_pso->akaze_kcontrast_percentile = parametry_pso->akaze_kcontrast_percentile + parametry_pso->predkosc_akaze_kcontrast_percentile;

  if(parametry_pso->error_threshold1  >  p_max->max_polozenie_error_threshold1)
  {
    parametry_pso->error_threshold1  = p_max->max_polozenie_error_threshold1;
  }
  if(parametry_pso->error_threshold1  <  p_max->min_polozenie_error_threshold1)
  {
    parametry_pso->error_threshold1  = p_max->min_polozenie_error_threshold1;
  }
  if(parametry_pso->pewnosc_1  >  p_max->max_polozenie_pewnosc_1)
  {
    parametry_pso->pewnosc_1  = p_max->max_polozenie_pewnosc_1;
  }
  if(parametry_pso->pewnosc_1  <  p_max->min_polozenie_pewnosc_1)
  {
    parametry_pso->pewnosc_1  = p_max->min_polozenie_pewnosc_1;
  }
  if(parametry_pso->error_threshold2  >  p_max->max_polozenie_error_threshold2)
  {
    parametry_pso->error_threshold2  = p_max->max_polozenie_error_threshold2;
  }
  if(parametry_pso->error_threshold2  <  p_max->min_polozenie_error_threshold2)
  {
    parametry_pso->error_threshold2  = p_max->min_polozenie_error_threshold2;
  }
  if(parametry_pso->pewnosc_2  >  p_max->max_polozenie_pewnosc_2)
  {
    parametry_pso->pewnosc_2  = p_max->max_polozenie_pewnosc_2;
  }
  if(parametry_pso->pewnosc_2  <  p_max->min_polozenie_pewnosc_2)
  {
    parametry_pso->pewnosc_2  = p_max->min_polozenie_pewnosc_2;
  }
  if(parametry_pso->detector_thresh  >  p_max->max_polozenie_detector_thresh)
  {
    parametry_pso->detector_thresh  = p_max->max_polozenie_detector_thresh;
  }
  if(parametry_pso->detector_thresh  <  p_max->min_polozenie_detector_thresh)
  {
    parametry_pso->detector_thresh  = p_max->min_polozenie_detector_thresh;
  }
  /*
  if(parametry_pso->akaze_kcontrast_percentile >  p_max->max_polozenie_akaze_kcontrast_percentile)
  {
    parametry_pso->akaze_kcontrast_percentile = p_max->max_polozenie_akaze_kcontrast_percentile;
  }
  if(parametry_pso->akaze_kcontrast_percentile <  p_max->min_polozenie_akaze_kcontrast_percentile)
  {
    parametry_pso->akaze_kcontrast_percentile  = p_max->min_polozenie_akaze_kcontrast_percentile;
  }
  */

  parametry_pso->status = 0;

}

/** @function main */
int main( int argc, char** argv )
{
  if(strcmp(argv[1], "help") == 0 ||strcmp(argv[1], "HELP") == 0 || strcmp(argv[1], "-h") == 0 || strcmp(argv[1], "-H") == 0 || strcmp(argv[1], "h") == 0 || strcmp(argv[1], "H") == 0 )
   {
     readme();
     return -1;
   }
  std::string result_folder; 

  if( argc == 3 )
  {
    result_folder = argv[2];
    result_folder += "/";
    goto args_count_ok;
  } 
  if(argc == 2)
  {
    result_folder = "";
    goto args_count_ok;
  }


  if( argc != 2 )
   {
     readme();
     return -1;
   }

  args_count_ok:

  struct timeval ttime_start, ttime_end, ttime_diff;
  gettimeofday(&ttime_start, NULL);

  time_t t_poczatkowy = time(NULL);
  struct tm tm_start = *localtime(&t_poczatkowy);
  std::ofstream plik_laczny_czas_optymalizacji;
  plik_laczny_czas_optymalizacji.open((result_folder + "czasy.txt").c_str(), std::ofstream::out | std::ofstream::app);
  plik_laczny_czas_optymalizacji<< "Start: rok, dzien, miesiac, godzina, minuta, sekunda: " << std::endl << tm_start.tm_year + 1900 << " " << tm_start.tm_mon + 1 << " " << tm_start.tm_mday << " " << tm_start.tm_hour <<" " << tm_start.tm_min <<" " << tm_start.tm_sec << std::endl<<std::flush;
  plik_laczny_czas_optymalizacji.close();

  std::string folder = argv[1];
  std::string gt_txt = folder + "/groundtruth.txt";
  //inicjalizacja czytania folderow 
  std::random_device rd;
  std::mt19937 gen(rd()); // ustawiam generator liczb pseudolosowych

  //timeval timer;
  //double time_1, time_2, time_3, time_4, time_5, time_6, time_7,
  //        time_8, time_9, time_10, time_11, time_12, time_13, time_14;
  //double time_cumulative_1 = 0, time_cumulative_2 = 0, time_cumulative_3 = 0, time_cumulative_4 = 0,
  //      time_cumulative_5 = 0, time_cumulative_6 = 0, time_cumulative_7 = 0, time_cumulative_8 = 0,
  //       time_cumulative_9 = 0, time_cumulative_10 = 0;
  //gettimeofday(&timer, NULL);
  //time_1= timer.tv_sec+(timer.tv_usec/1000000.0);
  //FILE *file_rgb_txt;
  //FILE *file_depth_txt;
  //int znak_plik;

  struct predkosc vmax;

  vmax.max_predkosc_error_threshold1 = 0.005;
  vmax.min_predkosc_error_threshold1 = -0.005;
  vmax.max_predkosc_pewnosc_1 = 0.05;
  vmax.min_predkosc_pewnosc_1 = -0.05;
  vmax.max_predkosc_error_threshold2 = 0.005;
  vmax.min_predkosc_error_threshold2 = -0.005;
  vmax.max_predkosc_pewnosc_2 = 0.05;
  vmax.min_predkosc_pewnosc_2 = -0.05;
  vmax.max_predkosc_detector_thresh = 5e-3;
  vmax.min_predkosc_detector_thresh = -5e-3;
  vmax.max_predkosc_akaze_kcontrast_percentile = 0; //0.05;
  vmax.min_predkosc_akaze_kcontrast_percentile = 0; //-0.05;

  std::uniform_real_distribution<double> dis_predkosc_error_threshold1(vmax.min_predkosc_error_threshold1, std::nextafter(vmax.max_predkosc_error_threshold1, DBL_MAX));
  std::uniform_real_distribution<double> dis_predkosc_pewnosc_1(vmax.min_predkosc_pewnosc_1, std::nextafter(vmax.max_predkosc_pewnosc_1, DBL_MAX));
  std::uniform_real_distribution<double> dis_predkosc_error_threshold2(vmax.min_predkosc_error_threshold2, std::nextafter(vmax.max_predkosc_error_threshold2, DBL_MAX));
  std::uniform_real_distribution<double> dis_predkosc_pewnosc_2(vmax.min_predkosc_pewnosc_2, std::nextafter(vmax.max_predkosc_pewnosc_2, DBL_MAX));
  std::uniform_real_distribution<float>  dis_predkosc_detector_thresh(vmax.min_predkosc_detector_thresh, std::nextafter(vmax.max_predkosc_detector_thresh, FLT_MAX));
  std::uniform_real_distribution<float>  dis_predkosc_akaze_kcontrast_percentile(vmax.min_predkosc_akaze_kcontrast_percentile, std::nextafter(vmax.max_predkosc_akaze_kcontrast_percentile, FLT_MAX));


  struct polozenie pmax;
  pmax.max_polozenie_error_threshold1 = 0.09;
  pmax.min_polozenie_error_threshold1 = 0.001;
  pmax.max_polozenie_pewnosc_1 = 0.95;
  pmax.min_polozenie_pewnosc_1 = 0.80;
  pmax.max_polozenie_error_threshold2 = 0.09;
  pmax.min_polozenie_error_threshold2 = 0.0005;
  pmax.max_polozenie_pewnosc_2 = 0.95;
  pmax.min_polozenie_pewnosc_2 = 0.80;
  pmax.max_polozenie_detector_thresh = 2e-3;
  pmax.min_polozenie_detector_thresh = 1e-5;
  pmax.max_polozenie_akaze_kcontrast_percentile = 0.7; //0.90;
  pmax.min_polozenie_akaze_kcontrast_percentile = 0.7; //0.50;


  std::uniform_real_distribution<double> dis_polozenie_error_threshold1(pmax.min_polozenie_error_threshold1, std::nextafter(pmax.max_polozenie_error_threshold1, DBL_MAX));
  std::uniform_real_distribution<double> dis_polozenie_pewnosc_1(pmax.min_polozenie_pewnosc_1, std::nextafter(pmax.max_polozenie_pewnosc_1, DBL_MAX));
  std::uniform_real_distribution<double> dis_polozenie_error_threshold2(pmax.min_polozenie_error_threshold2, std::nextafter(pmax.max_polozenie_error_threshold2, DBL_MAX));
  std::uniform_real_distribution<double> dis_polozenie_pewnosc_2(pmax.min_polozenie_pewnosc_2, std::nextafter(pmax.max_polozenie_pewnosc_2, DBL_MAX));
  std::uniform_real_distribution<float>  dis_polozenie_detector_thresh(pmax.min_polozenie_detector_thresh, std::nextafter(pmax.max_polozenie_detector_thresh, FLT_MAX));
  std::uniform_real_distribution<float>  dis_polozenie_akaze_kcontrast_percentile(pmax.min_polozenie_akaze_kcontrast_percentile, std::nextafter(pmax.max_polozenie_akaze_kcontrast_percentile, FLT_MAX));


  int liczba_czastek_pso = 40;
  struct parametry parametry_pso[40];
  struct parametry_naj parametry_pso_naj;
  int nr = 0;
  int iteracje = 0;
  int iteracje_max = 9;

  std::string rgb_txt;
  std::string depth_txt;

  rgb_txt = folder + "/rgb.txt";
  depth_txt = folder + "/depth.txt";

  std::vector< std::pair<double, std::string> > depth_stamps_and_filenames; 
  std::vector< std::pair<double, std::string> > depth_stamps_and_filenames_tmp;
  std::vector< std::pair<double, std::string> > rgb_stamps_and_filenames;
  std::vector< std::pair<double, std::string> > rgb_stamps_and_filenames_tmp;

  readFile_timestamp (folder, depth_txt, depth_stamps_and_filenames_tmp);
  readFile_timestamp (folder, rgb_txt, rgb_stamps_and_filenames_tmp);
  associate(rgb_stamps_and_filenames_tmp, depth_stamps_and_filenames_tmp, rgb_stamps_and_filenames, depth_stamps_and_filenames);


  for(unsigned int numer_klatki=0 ; numer_klatki < (rgb_stamps_and_filenames).size(); ++numer_klatki)
  {
    std::cout<<folder + "/" + (rgb_stamps_and_filenames)[numer_klatki].second;
    std::cout<<" "<<folder + "/" + (depth_stamps_and_filenames)[numer_klatki].second<<std::endl;
  }

  std::thread w[12];
  int w_nr_pso[12];
  for(int i = 0; i < 12; ++i)
  {
    w_nr_pso[i] = -1;
  }

  int key;
  char zapisuj_na_rozkaz = 0;
  int koniec=0;

  std::string pso_parametry_trajektoria = result_folder + "pso_parametry_trajektoria.txt";
  std::string pso_parametry = result_folder + "pso_parametry.txt";
  std::string pso_log_parametry = result_folder + "pso_log_parametry.txt";
  std::string w_baza = "p_";
  std::string w_nazwa;
  /*
  std::string w_nazwa[12];
  for(int i = 0; i < 12; ++i)
  {
    w_nazwa[i] = "w"+std::to_string(i+1);

  }
  */

  //std::ofstream plik_pso_parametry( (pso_parametry).c_str());
  //std::ofstream plik_log_pso_parametry( (pso_log_parametry).c_str());                     // Co za bzdura, bo gcc (tworcom) nie podoba sie goto przed inicjalizacja zmiennych (ktore i tak koncza zywot przed docelowym miejscem skoku..... czyli koncem proggramu .......)
  //std::ofstream plik_pso_parametry_trajektoria( (pso_parametry_trajektoria).c_str());

  std::ofstream plik_pso_parametry;
  std::ofstream plik_log_pso_parametry;
  std::ofstream plik_pso_parametry_trajektoria;
  FILE *file;
  std::string komenda;

  //
  // Wczytujemy?
  if (plik_txt_istnieje((pso_parametry).c_str()))
  {
    readsave_pso_file (pso_parametry, &koniec, &liczba_czastek_pso, &iteracje, &iteracje_max, &nr, &(parametry_pso[0]), &parametry_pso_naj); //wczytuje dane
    if (koniec == 1) //eksperymrnt skonczony
    {
      printf("Nie nadpisuje plikow- stworz nowy folder dla nowego eksperymentu!!! ten skonczony \n Lub napraw pliki txt (bo tak z nich wynika ze koniec) \n");
      goto eksperyment_skonczony;
    }
    else
    {
       printf("wczytano dane i to nie koniec liczenia \n");
       ++nr; // bo zapisalem ostatnio liczona to teraz kolejna
       if(nr == (liczba_czastek_pso)) // bo moze jednak to byl ostatni nr w danej iteracji tu nie jest liczba_czastek_pso - 1 bo dodaje ++nr wiec sie bedzie zgadzac
       {
         printf("reset nr\n");
         for(int i = 0 ; i < liczba_czastek_pso; ++i)
         {
           aktualizuj_parametry(&(parametry_pso[i]), &parametry_pso_naj, &pmax, &vmax, &gen);
         }
       ++iteracje; 
       nr = 0;
       }
       for (int i = 0 ; i < liczba_czastek_pso; ++i)
       {
         std::cout<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].akaze_kcontrast_percentile <<" " << parametry_pso[i].flann_param << " " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
       }
    plik_pso_parametry.open((pso_parametry).c_str(), std::ofstream::out | std::ofstream::app);
    plik_log_pso_parametry.open((pso_log_parametry).c_str(), std::ofstream::out | std::ofstream::app);
    plik_pso_parametry_trajektoria.open((pso_parametry_trajektoria).c_str(), std::ofstream::out | std::ofstream::app);
    }
  }
  else
  {
    printf("inicjalizacja PSO\n");
    // inicjalizacja

    for(int i=0; i< liczba_czastek_pso; ++i)
    {
      parametry_pso[i].error_threshold1 = dis_polozenie_error_threshold1(gen);
      parametry_pso[i].pewnosc_1 = dis_polozenie_pewnosc_1(gen);
      parametry_pso[i].error_threshold2 = dis_polozenie_error_threshold2(gen);
      parametry_pso[i].pewnosc_2 = dis_polozenie_pewnosc_2(gen);
      parametry_pso[i].detector_thresh = dis_polozenie_detector_thresh(gen);
      parametry_pso[i].akaze_kcontrast_percentile = 0.7;//dis_polozenie_akaze_kcontrast_percentile(gen);

      parametry_pso[i].naj_error_threshold1 = parametry_pso[i].error_threshold1;
      parametry_pso[i].naj_pewnosc_1 = parametry_pso[i].pewnosc_1;
      parametry_pso[i].naj_error_threshold2 = parametry_pso[i].error_threshold2;
      parametry_pso[i].naj_pewnosc_2 = parametry_pso[i].pewnosc_2;
      parametry_pso[i].naj_detector_thresh = parametry_pso[i].detector_thresh;
      parametry_pso[i].naj_akaze_kcontrast_percentile = parametry_pso[i].akaze_kcontrast_percentile;

      parametry_pso[i].predkosc_error_threshold1 = dis_predkosc_error_threshold1(gen);
      parametry_pso[i].predkosc_pewnosc_1 = dis_predkosc_pewnosc_1(gen);
      parametry_pso[i].predkosc_error_threshold2 = dis_predkosc_error_threshold2(gen);
      parametry_pso[i].predkosc_pewnosc_2 = dis_predkosc_pewnosc_2(gen);
      parametry_pso[i].predkosc_detector_thresh = dis_predkosc_detector_thresh(gen);
      parametry_pso[i].predkosc_akaze_kcontrast_percentile = 0; //dis_predkosc_akaze_kcontrast_percentile(gen);

      parametry_pso[i].flann_param = 32;
      parametry_pso[i].status = 0;
      parametry_pso[i].miara = 10000;
      parametry_pso[i].naj_miara = parametry_pso[i].miara;
    }
    
     //if ((iteracje == 0 ) && !(plik_txt_istnieje((pso_parametry).c_str())))
    // {
     // wazne zeby cos byla a nie ze za najlepszy wynik wczyta obojetnie co. Najgorzej jak bardzo dobra wartosc i bedzie dazyc do bzdur bo nic tego nie przebije 
     parametry_pso_naj.miara = parametry_pso[0].miara;
     std::cout<< "parametry pso naj miara: "<<parametry_pso_naj.miara << std::endl;
     parametry_pso_naj.error_threshold1 = parametry_pso[0].error_threshold1;
     parametry_pso_naj.pewnosc_1 = parametry_pso[0].pewnosc_1;
     parametry_pso_naj.error_threshold2 = parametry_pso[0].error_threshold2;
     parametry_pso_naj.pewnosc_2 = parametry_pso[0].pewnosc_2;
     parametry_pso_naj.detector_thresh = parametry_pso[0].detector_thresh;
     parametry_pso_naj.akaze_kcontrast_percentile= parametry_pso[0].akaze_kcontrast_percentile;
     parametry_pso_naj.flann_param = parametry_pso[0].flann_param;
   //}
  
    plik_pso_parametry.open((pso_parametry).c_str(), std::ofstream::out);
    plik_log_pso_parametry.open((pso_log_parametry).c_str(), std::ofstream::out);
    plik_pso_parametry_trajektoria.open((pso_parametry_trajektoria).c_str(), std::ofstream::out);
  }


  while ((iteracje < iteracje_max) && (zapisuj_na_rozkaz == 0) )
  {
    for(int i=0; i< liczba_czastek_pso; ++i)
    {
      //plik_pso_parametry<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
      plik_log_pso_parametry<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].akaze_kcontrast_percentile  << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].naj_error_threshold1 << " " << parametry_pso[i].naj_pewnosc_1 << " " << parametry_pso[i].naj_error_threshold2 << " " << parametry_pso[i].naj_pewnosc_2 << " " << parametry_pso[i].naj_detector_thresh<< " " << parametry_pso[i].naj_akaze_kcontrast_percentile  << " " << parametry_pso[i].naj_miara << " "<< parametry_pso[i].predkosc_error_threshold1<< " "<< parametry_pso[i].predkosc_pewnosc_1 << " " << parametry_pso[i].predkosc_error_threshold2 << " " << parametry_pso[i].predkosc_pewnosc_2 <<" " << parametry_pso[i].predkosc_detector_thresh << " " << parametry_pso[i].predkosc_akaze_kcontrast_percentile  <<" " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
      //plik_log_pso_parametry<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].status <<" " << parametry_pso[i].miara << std::endl;
    }
    printf("licze czastki w lailu dla %d  w najmniejszej petli for pso\n", iteracje);


    //double miara_wyliczona = 10000;
    //miara_wyliczona = daj_miare_ate_rmse(&folder, &result_folder, &w1_nazwa);
    //std::cout<<" \n\n\n\n Costam policzylem :     "<< miara_wyliczona <<" \n\n\n\n";
  
    plik_pso_parametry<< "#czy zostalo policzone 0 nie 1 tak # liczba czastek pso # liczba iteracji # nr ostatniej obliczone jczastki\n";
    plik_pso_parametry<< koniec <<" " << liczba_czastek_pso << " " << iteracje<< " " << iteracje_max << " " << nr<<"\n";
  

    for(int i = 0; i < 12; ++i)
    {
      while((parametry_pso[nr].status == 2) || (parametry_pso[nr].status == 3) ) //3 oznacza liczenia na innym komputerze
      {
        ++nr;
        printf("++nr przwijanie w wczytywaniu \n");
      }
      if (nr < liczba_czastek_pso)
      {
        w_nr_pso[i] = nr;
        w_nazwa = w_baza + std::to_string(w_nr_pso[i]);
        std::cout<<"w nazwa "<< w_nazwa << std::endl;
        w[i] = std::thread(cala_petla, &folder, &result_folder, w_nazwa, &rgb_stamps_and_filenames, &depth_stamps_and_filenames, parametry_pso[w_nr_pso[i]].error_threshold1, parametry_pso[w_nr_pso[i]].pewnosc_1,  parametry_pso[w_nr_pso[i]].error_threshold2, parametry_pso[w_nr_pso[i]].pewnosc_2, parametry_pso[w_nr_pso[i]].detector_thresh, parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile,parametry_pso[w_nr_pso[i]].flann_param, &parametry_pso[w_nr_pso[i]].status);
        plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2 << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile<< " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold1 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].naj_detector_thresh<< " " << parametry_pso[w_nr_pso[i]].naj_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].naj_miara << " "<< parametry_pso[w_nr_pso[i]].predkosc_error_threshold1<< " "<< parametry_pso[w_nr_pso[i]].predkosc_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].predkosc_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].predkosc_pewnosc_2 <<" " << parametry_pso[w_nr_pso[i]].predkosc_detector_thresh << " " << parametry_pso[w_nr_pso[i]].predkosc_akaze_kcontrast_percentile <<" " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
        ++nr;
      }
    //plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2   << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << "1" << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
   // poniewaz zaczynam i naj nie jest jeszcze liczone  przyjmuje za naj jest dla 
    }
    --nr; //bo dodaje +1 na koncu a potem watek dodaje juz sobie sam i w pierwszej petli while mam przeskok o 2 

    key = getkey();
    if(key == ' ') //zapisuj 
    {
      zapisuj_na_rozkaz = 1;
    }
  
    while((nr <  (liczba_czastek_pso - 1)) && (zapisuj_na_rozkaz == 0) )
    {
      for(int i = 0; i < 12; ++i)
      {
         if(parametry_pso[w_nr_pso[i]].status == 2)
        {
          w[i].join();
          w_nazwa = w_baza + std::to_string(w_nr_pso[i]);
          //parametry_pso[w_nr_pso[i]].miara = daj_miare_ate_rmse(&folder, &result_folder, &w_nazwa);
          parametry_pso[w_nr_pso[i]].miara = rpe(gt_txt, result_folder +"trajectory_" + w_nazwa + ".txt");
          plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2 << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold1 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].naj_detector_thresh<< " " << parametry_pso[w_nr_pso[i]].naj_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].naj_miara << " "<< parametry_pso[w_nr_pso[i]].predkosc_error_threshold1<< " "<< parametry_pso[w_nr_pso[i]].predkosc_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].predkosc_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].predkosc_pewnosc_2 <<" " << parametry_pso[w_nr_pso[i]].predkosc_detector_thresh << " " << parametry_pso[w_nr_pso[i]].predkosc_akaze_kcontrast_percentile <<" " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
          //plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2   << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
          ++nr;
          w_nr_pso[i] = nr;
          while((parametry_pso[w_nr_pso[i]].status == 3) && (nr <  liczba_czastek_pso ))
          {
            ++nr; // bo liczone na innym koputerze
            w_nr_pso[i] = nr;
          }
          if(nr == liczba_czastek_pso)
          {
            --nr;
          }
          else
          {
            w_nazwa = w_baza + std::to_string(w_nr_pso[i]);
            w[i] = std::thread(cala_petla, &folder, &result_folder, w_nazwa, &rgb_stamps_and_filenames, &depth_stamps_and_filenames, parametry_pso[w_nr_pso[i]].error_threshold1, parametry_pso[w_nr_pso[i]].pewnosc_1,  parametry_pso[w_nr_pso[i]].error_threshold2, parametry_pso[w_nr_pso[i]].pewnosc_2, parametry_pso[w_nr_pso[i]].detector_thresh, parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile, parametry_pso[w_nr_pso[i]].flann_param, &parametry_pso[w_nr_pso[i]].status);
            // ustawiam parametry_pso[w_nr_pso[i]].status) na 1 bo watek zostaje odpalony ale zanim zacznie chodzić już jest sprawdzany staus
            plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2 << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold1 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].naj_detector_thresh << " " << parametry_pso[w_nr_pso[i]].naj_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].naj_miara << " "<< parametry_pso[w_nr_pso[i]].predkosc_error_threshold1<< " "<< parametry_pso[w_nr_pso[i]].predkosc_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].predkosc_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].predkosc_pewnosc_2 <<" " << parametry_pso[w_nr_pso[i]].predkosc_detector_thresh << " " << parametry_pso[w_nr_pso[i]].predkosc_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
            //plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2   << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << "1" << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
          }
        }
      }
  
      key = getkey();
      if(key == ' ') //zapisuj 
      {
        std::cout<<" \n\n\n\n Wduszono spacje     \n\n\n\n\n\n\n\n\n\n\n \n\n\n\n";
	zapisuj_na_rozkaz = 1; 
        //nr = liczba_czastek_pso; //while nie policzy juz i czekam na koniec liczenia tego co mam
      }
    sleep(3);
    }
  
  
    // I tak nie ma znaczenia w ktorej kolejnosci skoncza bo czekam na wszystkie
    for(int i = 0; i < 12; ++i)
    {
      if(w_nr_pso[i] > -1)
      {
        w[i].join();
        w_nazwa = w_baza + std::to_string(w_nr_pso[i]);
      parametry_pso[w_nr_pso[i]].miara = daj_miare_ate_rmse(&folder, &result_folder, &w_nazwa);
      parametry_pso[w_nr_pso[i]].miara = rpe(gt_txt, result_folder +"trajectory_" + w_nazwa + ".txt");
    //plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2   << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
        plik_log_pso_parametry<< w_nr_pso[i] << " " << std::fixed<< std::setprecision(6) << parametry_pso[w_nr_pso[i]].error_threshold1 << " " << parametry_pso[w_nr_pso[i]].pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].error_threshold2 << " " << parametry_pso[w_nr_pso[i]].pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].detector_thresh << " " << parametry_pso[w_nr_pso[i]].akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].flann_param << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold1 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].naj_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].naj_pewnosc_2 << " " << parametry_pso[w_nr_pso[i]].naj_detector_thresh << " " << parametry_pso[w_nr_pso[i]].naj_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].naj_miara << " "<< parametry_pso[w_nr_pso[i]].predkosc_error_threshold1<< " "<< parametry_pso[w_nr_pso[i]].predkosc_pewnosc_1 << " " << parametry_pso[w_nr_pso[i]].predkosc_error_threshold2 << " " << parametry_pso[w_nr_pso[i]].predkosc_pewnosc_2 <<" " << parametry_pso[w_nr_pso[i]].predkosc_detector_thresh << " " << parametry_pso[w_nr_pso[i]].predkosc_akaze_kcontrast_percentile << " " << parametry_pso[w_nr_pso[i]].status << " " << parametry_pso[w_nr_pso[i]].miara << std::endl;
      }
    }
  
    for(int i=0; i< liczba_czastek_pso; ++i)
    {
      if(parametry_pso[i].miara < parametry_pso_naj.miara)
      {
	std::cout << "parametry_pso[i].miara " << parametry_pso[i].miara << " < parametry_pso_naj.miara" << parametry_pso_naj.miara << std::endl;
        parametry_pso_naj.miara = parametry_pso[i].miara;
	parametry_pso_naj.error_threshold1 = parametry_pso[i].error_threshold1;
	parametry_pso_naj.pewnosc_1 = parametry_pso[i].pewnosc_1; 
	parametry_pso_naj.error_threshold2 = parametry_pso[i].error_threshold2; 
	parametry_pso_naj.pewnosc_2 = parametry_pso[i].pewnosc_2; 
	parametry_pso_naj.detector_thresh = parametry_pso[i].detector_thresh; 
        parametry_pso_naj.akaze_kcontrast_percentile = parametry_pso[i].akaze_kcontrast_percentile;
        parametry_pso_naj.flann_param = parametry_pso[i].flann_param;


        w_nazwa = w_baza + std::to_string(i);
        komenda = "mv " + result_folder+"trajectory_"+ w_nazwa+".txt " +result_folder + "trajektoria_naj.txt";
        std::cout<<komenda<<std::endl;
        file = popen(komenda.c_str(),"r");
        pclose(file);
        komenda = "mv " + result_folder+"statystyki_kumulatywne_"+ w_nazwa+".txt " +result_folder + "statystyki_kumulatywne_naj.txt";
        std::cout<<komenda<<std::endl;
        file = popen(komenda.c_str(),"r");
        pclose(file);
        komenda = "mv " + result_folder+"statystyki_"+ w_nazwa+".txt " +result_folder + "statystyki_naj.txt";
        std::cout<<komenda<<std::endl;
        file = popen(komenda.c_str(),"r");
        pclose(file);
      }
    }
    
    if(iteracje == (iteracje_max-1))
    {
      koniec = 1;
    }

    plik_pso_parametry.close();
    plik_pso_parametry.open((pso_parametry).c_str());
    plik_pso_parametry<< "#czy zostalo policzone 0 nie 1 tak # liczba czastek pso # liczba iteracji # nr ostatniej obliczone jczastki\n";
    plik_pso_parametry<< koniec <<" " << liczba_czastek_pso << " " << iteracje << " " << iteracje_max<< " " << nr<<"\n";
    plik_pso_parametry<< std::fixed<< std::setprecision(6) << parametry_pso_naj.error_threshold1 << " " << parametry_pso_naj.pewnosc_1 << " " << parametry_pso_naj.error_threshold2 << " " << parametry_pso_naj.pewnosc_2 << " " << parametry_pso_naj.detector_thresh << " " << parametry_pso_naj.akaze_kcontrast_percentile<< " " << parametry_pso_naj.flann_param << " " << parametry_pso_naj.miara << std::endl;

    for(int i=0; i< liczba_czastek_pso; ++i)
    {
      plik_pso_parametry<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].akaze_kcontrast_percentile << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].naj_error_threshold1 << " " << parametry_pso[i].naj_pewnosc_1 << " " << parametry_pso[i].naj_error_threshold2 << " " << parametry_pso[i].naj_pewnosc_2 << " " << parametry_pso[i].naj_detector_thresh<< " " << parametry_pso[i].naj_akaze_kcontrast_percentile << " " << parametry_pso[i].naj_miara << " "<< parametry_pso[i].predkosc_error_threshold1<< " "<< parametry_pso[i].predkosc_pewnosc_1 << " " << parametry_pso[i].predkosc_error_threshold2 << " " << parametry_pso[i].predkosc_pewnosc_2 <<" " << parametry_pso[i].predkosc_detector_thresh << " " << parametry_pso[i].predkosc_akaze_kcontrast_percentile<< " " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
      plik_pso_parametry_trajektoria<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].akaze_kcontrast_percentile << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].naj_error_threshold1 << " " << parametry_pso[i].naj_pewnosc_1 << " " << parametry_pso[i].naj_error_threshold2 << " " << parametry_pso[i].naj_pewnosc_2 << " " << parametry_pso[i].naj_detector_thresh<< " " << parametry_pso[i].naj_akaze_kcontrast_percentile << " " << parametry_pso[i].naj_miara << " "<< parametry_pso[i].predkosc_error_threshold1<< " "<< parametry_pso[i].predkosc_pewnosc_1 << " " << parametry_pso[i].predkosc_error_threshold2 << " " << parametry_pso[i].predkosc_pewnosc_2 <<" " << parametry_pso[i].predkosc_detector_thresh << " " << parametry_pso[i].predkosc_akaze_kcontrast_percentile << " " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
      //plik_pso_parametry_trajektoria<< i << " " << std::fixed<< std::setprecision(6) << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].status << " " << parametry_pso[i].miara << std::endl;
    }
    plik_pso_parametry_trajektoria<< " --------------------------------------------------------------" << std::endl;
    if(nr == (liczba_czastek_pso - 1))
    {
      printf("reset nr\n");
      for(int i = 0 ; i < liczba_czastek_pso; ++i)
      {
        aktualizuj_parametry(&(parametry_pso[i]), &parametry_pso_naj, &pmax, &vmax, &gen);
      }
    nr = 0;
    }
    printf("Aktualizacja zakonczona sukcesem\n");
    //double miara_wyliczona = 10000;
    //miara_wyliczona = daj_miare_ate_rmse(&folder, &result_folder, &w1_nazwa);
    //parametry_pso[w_nr_pso[i]].miara = daj_miare_ate_rmse(&folder, &result_folder, &w_nazwa);
    //parametry_pso[w_nr_pso[i]].miara = rpe(gt_txt, result_folder +"trajectory_" + w_nazwa + ".txt");
    //std::cout<<" \n\n\n\n Costam policzylem :     "<< miara_wyliczona <<" \n\n\n\n";
  
  /*
    for(int i=0; i< liczba_czastek_pso; ++i)
    {
      plik_pso_parametry<< i << " " << parametry_pso[i].error_threshold1 << " " << parametry_pso[i].pewnosc_1 << " " << parametry_pso[i].error_threshold2 << " " << parametry_pso[i].pewnosc_2 << " " << parametry_pso[i].detector_thresh << " " << parametry_pso[i].flann_param << " " << parametry_pso[i].status << std::endl;
    }
  */
  ++iteracje; 
  } //koniec duzego hile od iteracji calego pso

    plik_pso_parametry.close();
    plik_pso_parametry_trajektoria.close();
    plik_log_pso_parametry.close();

  eksperyment_skonczony:

  plik_laczny_czas_optymalizacji.open((result_folder + "czasy.txt").c_str(), std::ofstream::out | std::ofstream::app);
  time_t t_koncowy = time(NULL);
  struct tm tm_koniec = *localtime(&t_koncowy);
  plik_laczny_czas_optymalizacji<< "Koniec rok, dzien, miesiac, godzina, minuta, sekunda: " << std::endl << tm_koniec.tm_year + 1900 << " " << tm_koniec.tm_mon + 1 << " " << tm_koniec.tm_mday << " " << tm_koniec.tm_hour <<" " << tm_koniec.tm_min <<" " << tm_koniec.tm_sec << std::endl<<std::flush;
  plik_laczny_czas_optymalizacji<<"Minelo sekund : " << (t_koncowy - t_poczatkowy) << std::endl<<std::flush;

  gettimeofday(&ttime_end, NULL);
  int a= timeval_subtract(&ttime_diff, &ttime_end, &ttime_start);
  plik_laczny_czas_optymalizacji<<"gettimeofday czy starczy bufora? sekundy: " << ttime_diff.tv_sec  << " Mikro: " << ttime_diff.tv_usec << std::endl<<std::flush;
  
  plik_laczny_czas_optymalizacji.close();

  waitKey(0);
  return 0;
}

 /** @function readme */
void readme()
{
  std::cout << " Usage: ./porownywarka <path to folder>" << std::endl;
  std::cout << "pcjonalnie można podać jako trzeci argument folder do zapisu danych" << std::endl;
  std::cout << " Usage: ./SURF_descriptor <path to data folder>    <path to results folder>" << std::endl;
  std::cout<<"przykladowe odpalanie: \n/home/ja/studia/porownanie_odometrii/build/main ~/Pulpit/trajectories/trajektoria_3 /home/ja/studia/porownanie_odometrii/test \n";
}

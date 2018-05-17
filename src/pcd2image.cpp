#include <iostream>
#include <string>
#include <iterator>
#include <vector>

// opencv
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

// pcl
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>

using namespace pcl;
using namespace pcl::io;
using namespace std;
using namespace cv;

#define length  30

pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
// pcl::io::loadPCDFile<pcl::PointXYZI>("/media/tusimple/SSD_2T_005/2018-05-03-11-42-59/1.pcd", *cloud_all);

void saveImage(double min_x, double max_x, double min_y, double max_y){

    string name = std::to_string((int)min_x) + std::to_string((int)min_y);

    int scale = 100;
    int dimension = length * scale;
    std::vector<std::vector<int> > vIntensityArray(dimension, vector<int>(dimension));
    std::vector<std::vector<int> > count(dimension, vector<int>(dimension));
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
    // pcl::io::loadPCDFile<pcl::PointXYZI>("/home/tusimple/Documents/p01008_n00799.txt.pcd", *cloud_all);
    for(int i = 0; i < cloud_all->size(); i++){
        if(cloud_all->points[i].x > min_x && cloud_all->points[i].x < max_x && 
           cloud_all->points[i].y > min_y && cloud_all->points[i].y < max_y){
            int col = (cloud_all->points[i].x - min_x) * scale;
            int row = (cloud_all->points[i].y - min_y) * scale;
            vIntensityArray[col][row] += (int)cloud_all->points[i].intensity;
            count[col][row] += 1;
        }
    }

    Mat outImage(dimension, dimension, CV_8UC1);
    // int nr = outImage.rows;  
    // int nc = outImage.cols; 
    for(int i = 0; i < dimension; ++i){
        for(int j = 0; j < dimension; ++j){
            if(count[i][j] <= 0){
                outImage.at<uchar>(i, j) = 0;
            }else{
                outImage.at<uchar>(i, j) = vIntensityArray[i][j] / count[i][j]; 
            }
        }
    }  

    string path = "/home/lgw/Documents/project/pcd2image/intensityImage/" + name + ".bmp";
    // cvNamedWindow("outImage_1", 1);
    // imwrite("/home/tusimple/Project/pcd2image/outImage/intensity/1.bmp", outImage);  
    imwrite(path, outImage);  
    // imshow("outImage", outImage);
    // waitKey(0);

    // path = "/home/lgw/Documents/project/pcd2image/intensityImage/" + name + ".bmp";
    // Mat outImage_2(dimension, dimension, CV_8UC1); // = outImage.clone();
    // threshold(outImage, outImage_2, 150, 255, CV_THRESH_BINARY);  
    // cvNamedWindow("outImage_2", 0);
    // imwrite(path, outImage_2); 
    // imshow("outImage", outImage_2);
    // waitKey(0);
}


int  main(int argc, char* argv[]) {

    if(argc < 2){
        cout << "Arguments wrong!" << endl << " Please enter PCD path." << endl;
    }

    string PCDpath = string(argv[1]);
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_all(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::loadPCDFile<pcl::PointXYZI>(PCDpath, *cloud_all);
    cout << "load PCD sucess!" << endl;
   
    double x_min_ = DBL_MAX;
    double y_min_ = DBL_MAX;
    double x_max_ = -DBL_MAX;
    double y_max_ = -DBL_MAX;

    for (int i = 0; i < cloud_all->size(); i++) {
        if (x_min_ > cloud_all->points[i].x) x_min_ = cloud_all->points[i].x;
        if (x_max_ < cloud_all->points[i].x) x_max_ = cloud_all->points[i].x;
        if (y_min_ > cloud_all->points[i].y) y_min_ = cloud_all->points[i].y;
        if (y_max_ < cloud_all->points[i].y) y_max_ = cloud_all->points[i].y;
    }
    std::cout << std::setprecision(12) << "x_max_:" << x_max_ << std::endl;
    std::cout << std::setprecision(12) << "x_min_:" << x_min_ << std::endl;
    std::cout << std::setprecision(12) << "y_max_:" << y_max_ << std::endl;
    std::cout << std::setprecision(12) << "y_min_:" << y_min_ << std::endl;

    double X = x_min_;
    double Y = y_min_;
    double step = 10;
    while(X < x_max_){
        while(Y < y_max_){
            saveImage(X, X + length, Y, Y + length);
            Y += step;
        }
        Y = y_min_;
        X += step;
    }  

    return 0;
}

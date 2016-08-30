#ifndef FACSLOCALFUNCTIONS_H
#define FACSLOCALFUNCTIONS_H

#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include "opencv2/core/core.hpp"

namespace facs
{

# define M_PI           3.14159265358979323846  /* pi */

    using namespace std;
    using namespace cv;

    struct paramList_facs
    {
        int patchSize;
        int stride;
        int featSize;
        int noPatches;
        int noTrees;
        Mat patchCoords;
    };

    struct randomTree_facs
    {
        vector<double> cutVar, cutValue, rightChild, leafVal;
    };

    //vector<string> get_all_files_names_within_folder(string folder);

    Mat RV_readCSVMat_facs(string csvFrameName, int lin, int col);
    void writeMatToFile_facs(cv::Mat& m, const char* filename);
    void RV_readParamList_facs(string foldername, paramList_facs *p);
    vector<randomTree_facs> RV_readAllTrees_facs(string forestDir, paramList_facs params);
    vector<double> RV_computeHist_facs(vector<double> vec, int noBins);

    vector<double> RV_testForest_facs(Mat localData, vector<randomTree_facs> &localForest, paramList_facs params, int numberOfClasses);

    //Rect RV_getBoundingBox(const Mat &im3D, paramList_facs p);
    double RV_computeNorm_facs(vector<double> vec1, vector<double> vec2);
    double RV_computeMean_facs(vector<double> vec);

    int RV_imshow_facs(string windowName, Mat img);
    void RV_readLPQParams_facs(string path, vector<Mat> &filters, Mat &V);
    vector<Mat> RV_lpqImage_facs(Mat img, vector<Mat> filters, Mat V, vector<paramList_facs> params);
    vector<double> RV_lpqPatch_facs(Mat img, vector<Mat> filters, Mat V);
    Mat RV_conv2pair_facs(Mat in, Mat k0, Mat k1);
    vector<double> RV_computeHist_2_facs(vector<double> vec, int noBins, double *sum);
    int RV_coutMat_facs(Mat in);

    //Mat RV_lbp(const Mat &src);

    //vector<double> RV_extractLPQ_light(Mat img);

}

#endif

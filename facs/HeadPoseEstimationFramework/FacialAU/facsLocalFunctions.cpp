#include <string>
#include <algorithm>
#include <iostream>
#include <vector>
#include <stdint.h>
#include <opencv2/core/core.hpp>
#include "facsLocalFunctions.h"
#include <fstream>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <map>
#include <boost/filesystem.hpp>
#include <boost/iterator/counting_iterator.hpp>

#define M_PI           3.14159265358979323846

namespace facs
{

using namespace std;
using namespace cv;

Mat RV_readCSVMat_facs(string csvFrameName, int lin, int col)
{
    Mat output(lin, col, CV_64FC1);
    ifstream inFile(csvFrameName); //.c_str()); //csvFrameName
    bool existFlag = boost::filesystem::exists(csvFrameName);
    if(!existFlag)
    {
        cout << "Missing file: " << csvFrameName << endl;
        throw std::exception();
    }
    string line;
    int linenum = 0;
    while (getline(inFile, line))
    {
        linenum++;
        //cout << "\nLine #" << linenum << ":" << endl;
        istringstream linestream(line);
        string item;
        int itemnum = 0;
        while (getline(linestream, item, ','))
        {
            itemnum++;
            //cout << "Item #" << itemnum << ": " << item << endl;
            double temp = (double)atof(item.c_str());
            output.at<double>(linenum - 1, itemnum - 1) = (double)temp;
        }
    }
    return output;
}

void writeMatToFile_facs(cv::Mat& m, const char* filename)
{
    ofstream fout(filename);

    if(!fout)
    {
        cout<<"File Not Opened"<<endl;  return;
    }

    for(int i=0; i<m.rows; i++)
    {
        for(int j=0; j<m.cols; j++)
        {
            fout<<m.at<double>(i,j)<<"\t";
        }
        fout<<endl;
    }

    fout.close();
}

void RV_readParamList_facs(string foldername, paramList_facs *p)
{
    cout << "Reading params from: " << foldername << "...";
    string configStr = foldername + "config.txt";
    ifstream in(configStr.c_str());
    string dummy;

    if (in.is_open()) {

        in >> dummy;
        in >> p->patchSize;

        in >> dummy;
        in >> p->stride;

        in >> dummy;
        in >> p->featSize;

        in >> dummy;
        in >> p->noPatches;

        in >> dummy;
        in >> p->noTrees;

    }
    else {
        cerr << "File not found " << foldername + "config.txt" << endl;
        exit(-1);
    }
    in.close();

    Mat paramsPatchCoords = RV_readCSVMat_facs(foldername + "patchCoords.csv", p->noPatches, 2);
    p->patchCoords = paramsPatchCoords;

    cout << "done!" << endl;
    /*cout << endl << "------------------------------------" << endl;
    cout << "Parameter List:" << endl << endl;
    cout << "patchSize:     " << p->patchSize << endl;
    cout << "stride:        " << p->stride << endl;
    cout << "featSize:      " << p->featSize << endl;
    cout << "noPatches:     " << p->noPatches << endl;*/
}

vector<randomTree_facs> RV_readAllTrees_facs(string treesDir, paramList_facs params)
{
    vector<randomTree_facs> allTrees(params.noTrees);
    cout << "Reading forests from: " << treesDir << "...";

    for (int j = 0; j < params.noTrees; j++)
    {
        Mat dummySize = RV_readCSVMat_facs(treesDir + "tree_" + to_string(j + 1) + "_size.csv", 1, 1);
        Mat dummyMat = RV_readCSVMat_facs(treesDir + "tree_" + to_string(j + 1) + ".csv", 4, (int)dummySize.at<double>(0, 0));
        allTrees[j].cutVar.reserve(dummyMat.cols);
        allTrees[j].cutValue.reserve(dummyMat.cols);
        allTrees[j].rightChild.reserve(dummyMat.cols);
        allTrees[j].leafVal.reserve(dummyMat.cols);
        dummyMat.row(0).copyTo(allTrees[j].cutVar);
        dummyMat.row(1).copyTo(allTrees[j].cutValue);
        dummyMat.row(2).copyTo(allTrees[j].rightChild);
        dummyMat.row(3).copyTo(allTrees[j].leafVal);
        //cout << "patch: " << i << " tree: " << j << endl;
    }

    cout << "done!" << endl;
    cout << endl << "------------------------------------" << endl << endl;
    return allTrees;
}

vector<double> RV_testForest_facs(Mat localData, vector<randomTree_facs> &localForest, paramList_facs params, int numberOfClasses)
{
    vector<double> votes(params.noTrees);
    votes.assign(params.noTrees, 0);
    for (int k = 0; k < params.noTrees; k++)
    {
        randomTree_facs tree = localForest[k];
        int currNode = 0;
        while (tree.cutVar[currNode] > 0)
        {
            if (localData.at<double>(tree.cutVar[currNode] - 1) > tree.cutValue[currNode])
            {
                currNode = tree.rightChild[currNode];
            }
            else
            {
                currNode = tree.rightChild[currNode] - 1;
            }
        }
        votes[k] = tree.leafVal[currNode];
    }
    vector<double> probs = RV_computeHist_facs(votes, numberOfClasses);
    return probs;
}

vector<double> RV_computeHist_facs(vector<double> vec, int noBins)
{
    // supposedly vec has exactly noBins unique values
    const double eps = 1.0 / vec.size();
    vector<double> out(noBins);
    out.assign(noBins, 0);
    for (int i = 0; i < (int)vec.size(); i++)
    {
        out[(int)vec[i] - 1] += eps;
    }
    return out;
}

double RV_computeNorm_facs(vector<double> vec1, vector<double> vec2)
{
    double result = 0;
    int dim = vec1.size();
    vector<double> diff(dim);
    double sumSq = 0;
    for (int i = 0; i < dim; i++)
    {
        diff[i] = vec1[i] - vec2[i];
        sumSq += pow(diff[i], 2);
    }
    result = sqrt(sumSq);
    return result;
}

double RV_computeMean_facs(vector<double> vec)
{
    double sum = 0;
    for (int k = 0; k < (int)vec.size(); k++)
    {
        sum += vec[k];
    }
    return sum / vec.size();
}

int RV_imshow_facs(const string windowName, cv::Mat img)
{
    double minVal, maxVal;
    Mat draw(img.rows, img.cols, CV_8UC1);
    minMaxLoc(img, &minVal, &maxVal);
    img.convertTo(draw, CV_8UC1, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
    cv::imshow(windowName, draw);
    return 1;
}

void RV_readLPQParams_facs(string path, vector<Mat> &filters, Mat &V)
{
    //vector<Mat> filters(5);
    filters[0] = RV_readCSVMat_facs( path + "w0.csv", 1, 3);
    filters[1] = RV_readCSVMat_facs( path + "w1_real.csv", 1, 3);
    filters[2] = RV_readCSVMat_facs( path + "w1_imag.csv", 1, 3);
    filters[3] = RV_readCSVMat_facs( path + "w2_real.csv", 1, 3);
    filters[4] = RV_readCSVMat_facs( path + "w2_imag.csv", 1, 3);

    //Mat V;
    V = RV_readCSVMat_facs( path + "V.csv", 8, 8);
}

vector<Mat> RV_lpqImage_facs(Mat img, vector<Mat> filters, Mat V, vector<paramList_facs> params)
{
    int noAU = params.size();
    //vector<vector<double> > featData(noAU);
    vector<Mat> featData_mat(noAU);
    // for every action unit index
    for(int i= 0; i< noAU; i++)
    {
        vector<double> localFeat;
        for(int j= 0; j< params[i].noPatches; j++)
        {
            Mat localImg;
            vector<double> coords = {params[i].patchCoords.at<double>(j, 0)- 1, params[i].patchCoords.at<double>(j, 1)- 1};
            img(Rect(coords[0], coords[1], params[i].patchSize, params[i].patchSize)).copyTo(localImg);
            //clock_t startTime = clock();
            vector<double> hist = RV_lpqPatch_facs(localImg, filters, V);
            //cout << "LPQ_patch took: " << double( clock() - startTime ) / (double)CLOCKS_PER_SEC<< " seconds." << endl;
            localFeat.insert(localFeat.end(), hist.begin(), hist.end()); //();
        }
        //featData[i].reserve(localFeat.size());
        //copy(localFeat.begin(), localFeat.end(), back_inserter(featData[i]));
        featData_mat[i] = Mat(1, localFeat.size(), CV_64FC1, localFeat.data()).clone();
    }
    return featData_mat;
}

vector<double> RV_lpqPatch_facs(Mat img, vector<Mat> filters, Mat V)
{
    int freqNum = 8;
    int freqRows = img.rows- 2;
    int freqCols = img.cols- 2;

    vector<Mat> dst;
    for(int i= 0; i< freqNum; i++)
    {
        dst.push_back(Mat(freqRows, freqCols, CV_64FC1, Scalar::all(0)));
    }

    /// Apply filters
    dst[0] = RV_conv2pair_facs(img, filters[0], filters[1]);
    dst[1] = RV_conv2pair_facs(img, filters[0], filters[2]);
    dst[2] = RV_conv2pair_facs(img, filters[1], filters[0]);
    dst[3] = RV_conv2pair_facs(img, filters[2], filters[0]);

    dst[4] = RV_conv2pair_facs(img, filters[1], filters[1])- RV_conv2pair_facs(img, filters[2], filters[2]);
    dst[5] = RV_conv2pair_facs(img, filters[1], filters[2])+ RV_conv2pair_facs(img, filters[2], filters[1]);
    dst[6] = RV_conv2pair_facs(img, filters[1], filters[3])- RV_conv2pair_facs(img, filters[2], filters[4]);
    dst[7] = RV_conv2pair_facs(img, filters[1], filters[4])+ RV_conv2pair_facs(img, filters[2], filters[3]);

    // perform decorrelation
    // If decorrelation is used, compute covariance matrix and corresponding whitening transform
    Mat dstR(freqRows* freqCols, freqNum, CV_64FC1);

    for(int i= 0; i< dst.size(); i++)
    {
        Mat dummy = Mat(dst[i].t()).reshape(0, freqRows* freqCols);
        //cout << dummy.channels() << " " << dstR.channels() << " " << dst[i].channels() << endl;
        dummy.copyTo(dstR(Rect(i, 0, dummy.cols, dummy.rows)));
    }

    Mat freqResp = (V.t()*dstR.t()).t();

    // Perform quantization and compute LPQ codewords
    Mat LPQdesc = Mat::zeros(1, freqResp.rows, CV_64FC1);
    for(int i= 0; i< freqResp.rows; i++)
    {
        for(int j= 0; j< freqResp.cols; j++)
        {
            if(freqResp.at<double>(i, j) > 0)
            {
                LPQdesc.at<double>(0, i)+= pow(2, j);
            }
        }
    }

    std::vector<double> LPQdesc_vect;
    for (int i = 0; i < LPQdesc.rows; ++i) {
        LPQdesc_vect.insert(LPQdesc_vect.end(), LPQdesc.ptr<double>(i), LPQdesc.ptr<double>(i)+ LPQdesc.cols);
    }

    double sum_of_elems;
    vector<double> hist = RV_computeHist_2_facs(LPQdesc_vect, 256, &sum_of_elems);
    transform(hist.begin(), hist.end(), hist.begin(), bind1st(std::multiplies<double>(), 1.0/sum_of_elems));

    return hist;
}

vector<double> RV_computeHist_2_facs(vector<double> vec, int noBins, double* sum)
    {
        // supposedly vec has only values between 0 and noBins- 1 (different from RV_computeHist!!!)
        // also, here there's no normalization
        *sum = 0.0;
        const double eps = 1.0; // / vec.size();
        vector<double> out(noBins);
        out.assign(noBins, 0);
        for (int i = 0; i < (int)vec.size(); i++)
        {
            out[(int)vec[i]] += eps;
            *sum+= eps;
        }
        return out;
    }

Mat RV_conv2pair_facs(Mat in, Mat k0, Mat k1)
{
    // convolve Mat in with kernels k0 and k1, respectively (both k0 and k1 are row vectors)
    // and return the convolution result (without padding)

    int ddepth = -1;
    Point anchor = Point(-1, -1);
    double delta = 0.0;

    Mat dst_dummy_1, dst_dummy_2;
    Mat w0; flip(k0, w0, -1);
    Mat w1; flip(k1, w1, -1);
    Mat out(in.rows- 2, in.cols- 2, CV_64FC1);

    filter2D(in, dst_dummy_1, ddepth, w0.t(), anchor, delta, BORDER_DEFAULT);
    filter2D(dst_dummy_1, dst_dummy_2, ddepth, w1, anchor, delta, BORDER_DEFAULT);
    dst_dummy_2(Rect(1, 1, dst_dummy_2.cols-2, dst_dummy_2.rows- 2)).copyTo(out);

    return out;
}

int RV_coutMat_facs(Mat in)
{
    int rows = in.rows;
    int cols = in.cols;
    for(int i= 0; i< rows; i++)
    {
        for(int j= 0; j< cols- 1; j++)
        {
            cout<< in.at<double>(i, j)<< " ";
        }
        cout<< in.at<double>(i, cols- 1)<< endl;
    }
    return 1;
}
}

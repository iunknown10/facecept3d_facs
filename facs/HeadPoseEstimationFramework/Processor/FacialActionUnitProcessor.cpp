//
// Created by radu on 25/05/16.
//

#include "FacialActionUnitProcessor.h"

#include <DataObject/CloudXYZRGBA.h>
#include <DataObject/LandmarksObject.h>

#include <Helpers/HpeHelpers.h>

using namespace cv;

namespace hpe
{
    void FacialActionUnitProcessor::Process(IDataStorage::Ptr dataStorage)
    {
        if (m_facsDataReady)
        {
            m_facialActionUnitReady(m_facsResult);
            m_facsDataReady = false;
        }

        if (m_facsWorkerBusy == false)
        {
            CloudXYZRGBA::Ptr cloudObject = dataStorage->GetAndCast<CloudXYZRGBA>("OriginalCloud");
            if (cloudObject.get() == nullptr)
            {
                cloudObject = dataStorage->GetAndCastNotNull<CloudXYZRGBA>("FilteredOriginalCloud", "FacialActionUnitProcessor::Process - cloud is null");
            }
            LandmarksObject<pcl::PointXYZRGBA>::Ptr landmarksObject = dataStorage->GetAndCastNotNull<LandmarksObject<pcl::PointXYZRGBA>>("Landmarks", "FacialActionUnitProcessor::Process - landmarks are null");

            Common<PointT>::Landmarks l;
            for (int i = 0; i < landmarksObject->landmarks.size(); i += 1)
            {
                PointT pt;
                pt.getVector3fMap() = landmarksObject->landmarks[i].point.getVector3fMap();
                l.push_back(Common<PointT>::Landmark(landmarksObject->landmarks[i].index, pt));
            }

            m_facsLandmarks = l;
            pcl::copyPointCloud(*(cloudObject->cloud), m_facsCloud);

            m_facsWorkerBusy = true;
        }

    }

    std::shared_ptr<CylinderSampler> FacialActionUnitProcessor::CreateCylindricalSampler()
    {
        /*
        PhiRange=0.05,0.95
        ZRange=1.3,0.9
        TopRows=45
        BottomRows=105
        SampleColumns=120
        */
        CylinderSampler::Range phiRange;
        phiRange.first = 0.05;
        phiRange.second = 0.95;

        CylinderSampler::Range zRange;
        zRange.first = 0.9;
        zRange.second = 1.3;

        int topRows = 45;
        int bottomRows = 105;
        int sampleColumns = 120;

        std::vector<int> leftEyeIndices;
        leftEyeIndices.push_back(0);
        std::vector<int> rightEyeIndices;
        rightEyeIndices.push_back(1);

        std::shared_ptr<CylinderSampler> sampler(new CylinderSampler(phiRange, zRange, topRows, bottomRows, sampleColumns));
        sampler->SetVisualize(false);
        sampler->SetSupportOptimizedSampling(true);
        sampler->SetEyeIndices(leftEyeIndices, rightEyeIndices);

        return sampler;
    }

    FacialActionUnitProcessor::FacialActionUnitProcessor()
    {
        m_auList = {1, 6, 12};
        m_numberOfActionUnits = 3;
        Init("FACSData/");
    }

    FacialActionUnitProcessor::FacialActionUnitProcessor(std::string dataFolder, std::vector<int> auList)
    {
        m_auList = auList;
        m_numberOfActionUnits = m_auList.size();
        Init(dataFolder);
    }

    //called in constructors
    void FacialActionUnitProcessor::Init(std::string dataFolder)
    {
        m_facsWorkerBusy = false;
        m_facsDataReady = false;
        std::vector<facs::paramList_facs> params_aux(m_numberOfActionUnits);
        std::vector<std::vector<facs::randomTree_facs> > forest_aux(m_numberOfActionUnits);


        for(int i= 0; i< m_numberOfActionUnits; i++)
        {
            std::ostringstream dirIdx;
            dirIdx << m_auList[i];
            std::string paramsDir = dataFolder+ "params/au_"+ dirIdx.str()+ "/";
            facs::RV_readParamList_facs(paramsDir, &params_aux[i]);
            std::string treesDir = dataFolder+ "trees/au_"+ dirIdx.str()+ "/";
            forest_aux[i] = facs::RV_readAllTrees_facs(treesDir, params_aux[i]);
        }

        m_facsParameters = params_aux;
        m_facsForests = forest_aux;
        std::string lpqFolder = dataFolder+ "lpq/";
        std::vector<cv::Mat> filters_aux(5);
        facs::RV_readLPQParams_facs(lpqFolder, filters_aux, m_V);
        m_LPQFilters = filters_aux;
    }


    //called before grabber starts to send frames
    void FacialActionUnitProcessor::Init()
    {
        std::shared_ptr<hpe::CylinderSampler> sampler = CreateCylindricalSampler();
        m_featureCalculator = std::shared_ptr<hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>>(new hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>(sampler, hpe::FeatureType::RGB));
        m_featureCalculator->SetFeatureSize(cv::Size(120, 150));

        boost::thread t(boost::bind(&FacialActionUnitProcessor::ThreadRoutine, this));
    }

    void FacialActionUnitProcessor::ThreadRoutine()
    {
        while (true)
        {
            while (m_facsWorkerBusy == false)
            {
                boost::this_thread::sleep(boost::posix_time::milliseconds(10));
            }

            auto l = m_facsLandmarks;

            pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

            Eigen::Vector3f zAxis;
            zAxis << 0, 0, 1;
            Eigen::Vector3f normal = l[3].point.getVector3fMap() - l[2].point.getVector3fMap();

            Eigen::Quaternionf rotation;
            rotation.setFromTwoVectors(normal, zAxis);
            Eigen::Matrix4f transformRotation = Eigen::Matrix4f::Identity();
            transformRotation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            pcl::transformPointCloud(m_facsCloud, *cloud, transformRotation);

            l = TransformLandmarks<PointT>(l, transformRotation);

            Eigen::Vector3f xAxis;
            xAxis << 1, 0, 0;
            Eigen::Vector3f eyesLine = l[1].point.getVector3fMap() - l[0].point.getVector3fMap();

            rotation.setFromTwoVectors(eyesLine, xAxis);
            transformRotation = Eigen::Matrix4f::Identity();
            transformRotation.block<3, 3>(0, 0) = rotation.toRotationMatrix();
            pcl::transformPointCloud(*cloud, *cloud, transformRotation);

            l = TransformLandmarks<PointT>(l, transformRotation);

            m_featureCalculator->SetLandmarks(l);

            cv::Mat features = m_featureCalculator->GetFeatures(cloud);
            cv::Mat unrolled_aux = features.reshape(0, 120);
            cv::Mat unrolled = unrolled_aux.t();
            //unrolled.convertTo(unrolled, CV_64FC1);

            //cv::Mat frame = fer::RV_preprocessDepthFrame(unrolled);
            cv::Mat frame;
            cv::cvtColor(unrolled, frame, CV_BGR2GRAY);
            //facs::RV_imshow("Unrolled_converted", frame);
            cv::imshow("Unrolled", frame);
            cv::waitKey(20);

            /*cv::Mat bgr[3];
            cv::split(unrolled, bgr);
            fer::RV_writeCSVMat("test_R", bgr[2]);
            fer::RV_writeCSVMat("test_G", bgr[1]);
            fer::RV_writeCSVMat("test_B", bgr[0]);*/

            std::vector<cv::Mat> featData = facs::RV_lpqImage_facs(frame, m_LPQFilters, m_V, m_facsParameters);

            std::vector<std::vector<double> > pred(featData.size(), std::vector<double>(2));
            std::vector<double> finalProbs(featData.size());

            for(int i= 0; i< featData.size(); i++)
            {
                std::vector<facs::randomTree_facs> trees = m_facsForests[i];
                pred[i] = facs::RV_testForest_facs(featData[i], trees, m_facsParameters[i], 2);
                finalProbs[i] = pred[i][1];
            }

            m_facsResult = finalProbs;
            m_facsDataReady = true;
            m_facsWorkerBusy = false;
        }
    }

}

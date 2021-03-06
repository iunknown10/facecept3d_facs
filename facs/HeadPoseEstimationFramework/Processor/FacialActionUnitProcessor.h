//
// Created by radu on 25/05/16.
//

#pragma once

#ifndef FACIALACTIONUNITPROCESSOR_H
#define FACIALACTIONUNITPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Features/Sampler/CylinderSampler.h>
#include <Features/Calculator/CylinderOptimizedFeatureCalculator.h>
#include <FacialAU/facsLocalFunctions.h>
#include <pcl/point_types.h>
#include <boost/signals2/signal.hpp>
#include <boost/circular_buffer.hpp>

namespace hpe
{
    class FacialActionUnitProcessor: public IProcessor
    {
    public:
        typedef boost::signals2::signal<void (std::vector<double>)> FacialActionUnitReadySignal;

        FacialActionUnitProcessor();
        FacialActionUnitProcessor(std::string dataFolder, std::vector<int> auList);
        void Process(IDataStorage::Ptr dataStorage);
        void Init();
        void SubscribeFacialActionUnitReadySignal(const FacialActionUnitReadySignal::slot_type &slot)
        {
            m_facialActionUnitReady.connect(slot);
        }

        void SetSaveImageFlag(bool flag)
        {
            m_facsSaveImgFlag = flag;
        }

        void SetSaveImageRoot(std::string root)
        {
            m_facsSaveImgRoot = root;
        }

        void SetSaveImageLocalFolder(std::string localFolder)
        {
            m_facsSaveImgLocalFolder = localFolder;
        }

    private:
        typedef pcl::PointXYZRGBNormal PointT;
        typedef pcl::PointXYZRGBNormal NormalT;

        void Init(std::string);
        std::shared_ptr<CylinderSampler> CreateCylindricalSampler();
        void ThreadRoutine();

        std::vector<facs::paramList_facs> m_facsParameters;
        std::vector<std::vector<facs::randomTree_facs>> m_facsForests;
        std::shared_ptr<hpe::CylinderOptimizedFeatureCalculator<PointT, NormalT>> m_featureCalculator;

        bool m_facsWorkerBusy;
        bool m_facsDataReady;
        std::vector<double> m_facsResult;
        std::vector<boost::circular_buffer<double>> m_facsBuffer;
        Common<PointT>::Landmarks m_facsLandmarks;
        pcl::PointCloud<PointT> m_facsCloud;
        int m_numberOfActionUnits;
        std::vector<int> m_auList;
        std::vector<cv::Mat> m_LPQFilters;
        cv::Mat m_V;

//        cv::Mat m_faceImage;
        double m_facsCounter;
        bool m_facsSaveImgFlag;
        std::string m_facsSaveImgRoot;
        std::string m_facsSaveImgLocalFolder;

        FacialActionUnitReadySignal m_facialActionUnitReady;
    };
}

#endif
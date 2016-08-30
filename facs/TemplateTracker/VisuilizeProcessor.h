#pragma once

#ifndef VISUALIZEPROCESSOR_H
#define VISUALIZEPROCESSOR_H

#include <Processor/IProcessor.h>
#include <Processor/TrackingProcessor.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ctime>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <chrono>

class VisuilizeProcessor : public hpe::IProcessor
{
    public:
        VisuilizeProcessor(std::string cloudKey = "Cloud");
        VisuilizeProcessor(bool saveScreenshots);
        ~VisuilizeProcessor(void);

        void Process(hpe::IDataStorage::Ptr dataStorage);

        void HandleFER(std::vector<double> ferdata);
        void HandleFACS(std::vector<double> facsdata);

        void SetTrackingProcessor(std::shared_ptr<hpe::TrackingProcessor> processor)
        {
            m_trackingProcessor = processor;
        }

        void SetLoggingFlag(bool loggingFlag)
        {
            m_loggingFlag = loggingFlag;
        }

    private:
        pcl::visualization::PCLVisualizer m_visualizer;
        bool m_first;
        bool m_saveScreenshots;
        bool m_haveFerData;
        bool m_haveFacsData;
        bool m_loggingFlag;
        bool m_initLogging;
        std::ofstream m_logFileStream;
        std::chrono::time_point<std::chrono::high_resolution_clock> m_logStartTime;
        std::vector<double> m_ferData;
        std::vector<double> m_facsData;
        std::string m_cloudKey;
        std::string m_landmarksKey;
        std::vector<std::string> m_expressionLabels;
        std::vector<std::string> m_actionUnitLabels;

        std::shared_ptr<hpe::TrackingProcessor> m_trackingProcessor;

        static void KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender);

        Eigen::Vector3f VectorToEulerAngles(Eigen::Vector3f v);

        void SaveCamera(pcl::visualization::Camera &camera, std::string file);
        void LoadCamera(pcl::visualization::Camera &camera, std::string file);

        void InitExpressionLabels();
        void InitActionUnitLabels();

        void InitLogging();
};

#endif
#include "stdafx.h"
#include "VisuilizeProcessor.h"

#include <DataObject/LandmarksObject.h>
#include <DataObject/CloudXYZRGBA.h>
#include <iomanip>
#include <boost/date_time/posix_time/ptime.hpp>
#include <chrono>
//#include <boost/date_time/posix_time/posix_time_io.hpp>
//#include <sys/time.h>
//
//#include <boost/format.hpp>
//#include <boost/filesystem.hpp>
//
//#include <math.h>
//#include <fstream>
//#include <sstream>
//#include <iomanip>

using namespace hpe;

VisuilizeProcessor::VisuilizeProcessor(std::string cloudKey)
    : m_cloudKey(cloudKey), m_landmarksKey("Landmarks"), m_visualizer("VisualizerProcessor"),
      m_first(true), m_saveScreenshots(false), m_haveFerData(false), m_haveFacsData(false), m_loggingFlag_au(false),
      m_loggingFlag_hp(false), m_initLogging(false), m_plotCloudFlag(true)
{
    InitExpressionLabels();
    InitActionUnitLabels();
    time_t now = time(0);
    m_logStartTime = std::chrono::high_resolution_clock::now();
    m_visCounter = -1;
    //m_logST = localtime(&now);
}

VisuilizeProcessor::VisuilizeProcessor(bool saveScreenshots)
    : m_cloudKey("Cloud"), m_landmarksKey("Landmarks"), m_visualizer("VisualizerProcessor"),
      m_first(true), m_saveScreenshots(saveScreenshots), m_haveFerData(false), m_haveFacsData(false), m_loggingFlag_au(false),
      m_loggingFlag_hp(false), m_initLogging(false)
{
    InitExpressionLabels();
    InitActionUnitLabels();
    time_t now = time(0);
    m_logStartTime = std::chrono::high_resolution_clock::now();
    //m_logST = localtime(&now);
}

VisuilizeProcessor::~VisuilizeProcessor(void)
{
}

void VisuilizeProcessor::Process(hpe::IDataStorage::Ptr dataStorage)
{
    CloudXYZRGBA::Ptr cloud = dataStorage->GetAndCast<CloudXYZRGBA>(m_cloudKey);
    LandmarksObject<pcl::PointXYZRGBA>::Ptr landmarks = dataStorage->GetAndCast<LandmarksObject<pcl::PointXYZRGBA>>(m_landmarksKey);
    if (cloud.get() == nullptr || landmarks.get() == nullptr)
    {
        return;
    }

    Eigen::Vector3f angles = VectorToEulerAngles(landmarks->landmarks[2].point.getVector3fMap() - landmarks->landmarks[3].point.getVector3fMap());

    if (m_plotCloudFlag) {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr eyes(new pcl::PointCloud<pcl::PointXYZRGBA>);
        eyes->push_back(landmarks->landmarks[0].point);
        eyes->push_back(landmarks->landmarks[1].point);


        pcl::ModelCoefficients cylinderCoefficients;
        cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.x);
        cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.y);
        cylinderCoefficients.values.push_back(landmarks->landmarks[2].point.z);
        cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.x - landmarks->landmarks[2].point.x) / 10);
        cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.y - landmarks->landmarks[2].point.y) / 10);
        cylinderCoefficients.values.push_back((landmarks->landmarks[3].point.z - landmarks->landmarks[2].point.z) / 10);
        cylinderCoefficients.values.push_back(0.002);

        std::string anglesText = (boost::format("Head pose in degrees:\n  Yaw   %5.2f\n  Tilt  %5.2f\n  Roll  %5.2f") %
                                  angles(0) % angles(1) % angles(2)).str();

        if (m_first) {
            m_first = false;

            m_visualizer.registerKeyboardCallback(&VisuilizeProcessor::KeyboardEventCallback, this);

            m_visualizer.addCylinder(cylinderCoefficients);
            m_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, ((float) 0x73) / 255,
                                                     ((float) 0x3C) / 255, "cylinder");
            m_visualizer.addPointCloud<pcl::PointXYZRGBA>(cloud->cloud, m_cloudKey);
            m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_cloudKey);
            m_visualizer.addPointCloud<pcl::PointXYZRGBA>(eyes, m_landmarksKey);
            m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0,
                                                          m_landmarksKey);
            m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5,
                                                          m_landmarksKey);
            m_visualizer.addText(anglesText, 25, 175, 17, 1, 1, 1, "anglesText");
            //m_visualizer.addText("Facial expression:", 25, 100, 17, 1, 1, 1, "anglesTextCaption");
            m_visualizer.setBackgroundColor(0, 0, 0);
            if (m_saveScreenshots) {
                if (boost::filesystem::exists("camera.cam")) {
                    pcl::visualization::Camera camera;
                    LoadCamera(camera, "camera.cam");
                    m_visualizer.setCameraParameters(camera);
                } else {
                    while (m_visualizer.wasStopped() == false) {
                        m_visualizer.spinOnce(100);
                    }
                    std::vector<pcl::visualization::Camera> cameras;
                    m_visualizer.getCameras(cameras);
                    pcl::visualization::Camera camera = cameras[0];
                    SaveCamera(camera, "camera.cam");
                }
            }
        } else {
            m_visualizer.removeShape("cylinder");
            m_visualizer.addCylinder(cylinderCoefficients);
            m_visualizer.removeShape("anglesText");
            m_visualizer.addText(anglesText, 25, 175, 17, 1, 1, 1, "anglesText");
            m_visualizer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, ((float) 0x73) / 255,
                                                     ((float) 0x3C) / 255, "cylinder");
            m_visualizer.removePointCloud(m_cloudKey);
            m_visualizer.addPointCloud<pcl::PointXYZRGBA>(cloud->cloud, m_cloudKey);
            m_visualizer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_cloudKey);
//        m_visualizer.updatePointCloud<pcl::PointXYZRGBA>(cloud->cloud, m_cloudKey);
            m_visualizer.updatePointCloud(eyes, m_landmarksKey);
        }

        m_visualizer.spinOnce(1);
    }

    if (m_haveFerData)
    {
        int x = 20;
        int y = 75;
        int delta = 16;

        auto recognizedExpression = std::max_element(m_ferData.begin(), m_ferData.end());
        int recognizedExpressionIndex = std::distance(m_ferData.begin(), recognizedExpression);

        for (int i = 0; i < m_ferData.size(); i++)
        {
            double colorScale = i == recognizedExpressionIndex ? 2 : 1;
            std::string message = (boost::format("  %1%: %2%") % m_expressionLabels[i] % m_ferData[i]).str();
            std::string textObjectLabel = (boost::format("ferText%1%") % i).str();
            m_visualizer.removeShape(textObjectLabel);
            m_visualizer.addText(message, x, y, 17, colorScale * 0.5, colorScale * 0.5, colorScale * 0.5, textObjectLabel);
            y += delta;
        }
    }

    if (m_haveFacsData)
    {
        int x = 20;
        int y = 35;
        int delta = 16;

        for (int i = 0; i < m_facsData.size()- 1; i++)
        {
            double colorScale = 2; //i == recognizedExpressionIndex ? 2 : 1;
            std::string message = (boost::format("  %1% %2%") % m_actionUnitLabels[m_facsData.size()- i- 2] % (boost::format("%.2f") % m_facsData[m_facsData.size()- i- 2]).str()).str();
            std::string textObjectLabel = (boost::format("facsText%1%") % i).str();
            m_visualizer.removeShape(textObjectLabel);
            m_visualizer.addText(message, x, y, 17, colorScale * 0.5, colorScale * 0.5, colorScale * 0.5, textObjectLabel);
            y += delta;
        }

        if (m_loggingFlag_au)
        {
            if (m_initLogging == false)
            {
                InitLogging();
            }

            if (!m_logFileStream_au.is_open())
            {
                cerr << "File not open: " << endl;
            }
            else
            {
                long facsCounter = (long)m_facsData[m_facsData.size()- 1];
                if (facsCounter > m_visCounter) {

                    m_visCounter++;
                    auto elapsed_time = std::chrono::high_resolution_clock::now();

                    std::chrono::duration<double> diff = (elapsed_time - m_logStartTime);
                    m_logFileStream_au << (boost::format("%.3f") % diff.count()) << ";FACS;" <<
                                    (boost::format("%.2f") % m_facsData[0]).str() << ";" <<
                                    (boost::format("%.2f") % m_facsData[1]).str() << ";" <<
                                    (boost::format("%.2f") % m_facsData[2]).str() << ";" <<
                                    (boost::format("%.2f") % m_facsData[3]).str() << ";" <<
                                    (boost::format("%.2f") % m_facsData[4]).str() << ";" <<
                                    (boost::format("%.2f") % m_facsData[5]).str() << ";" <<
                                    "IMG_" << setfill('0') << setw(6) << facsCounter << ".JPG" << endl;
                }
            }
        }
    }

    if (m_loggingFlag_hp)
    {
        if (m_initLogging == false)
        {
            InitLogging();
        }

        if (!m_logFileStream_hp.is_open())
        {
            cerr << "File not open: " << endl;
        }
        else
        {
            auto elapsed_time = std::chrono::high_resolution_clock::now();

            std::chrono::duration<double> diff = (elapsed_time - m_logStartTime);
            m_logFileStream_hp << (boost::format("%.3f") % diff.count()) << ";HEADPOSE;" <<
                               (boost::format("%.2f") % angles(0)).str() << ";" <<
                               (boost::format("%.2f") % angles(1)).str() << ";" << endl;
        }
    }

    if (m_saveScreenshots)
    {
        PCDGrabber::CloudFileInfo::Ptr fileInfo = dataStorage->GetAndCast<PCDGrabber::CloudFileInfo>("FileInfo");
        if (fileInfo.get() != nullptr)
        {
            std::string screenshotPath = boost::replace_all_copy(fileInfo->Filename, ".pcd", ".png");
            m_visualizer.saveScreenshot(screenshotPath);
        }
    }
}

Eigen::Vector3f VisuilizeProcessor::VectorToEulerAngles(Eigen::Vector3f v)
{
    Eigen::Vector3f normed = v / v.norm();
    float x = normed(0);
    float y = normed(1);
    float z = normed(2);

    Eigen::Vector3f result;
    result(0) = std::atan2(z, x) - M_PI / 2; // yaw
    result(1) = -std::atan2(y, z); // tilt
    result(2) = std::atan(y / x); // roll

    return result * 180 / M_PI;

}

void VisuilizeProcessor::SaveCamera(pcl::visualization::Camera &camera, std::string file)
{
    std::ofstream str(file);

    str << camera.clip[0] << std::endl;
    str << camera.clip[1] << std::endl;
    str << camera.focal[0] << std::endl;
    str << camera.focal[1] << std::endl;
    str << camera.focal[2] << std::endl;
    str << camera.fovy << std::endl;
    str << camera.pos[0] << std::endl;
    str << camera.pos[1] << std::endl;
    str << camera.pos[2] << std::endl;
    str << camera.view[0] << std::endl;
    str << camera.view[1] << std::endl;
    str << camera.view[2] << std::endl;
    str << camera.window_pos[0] << std::endl;
    str << camera.window_pos[1] << std::endl;
    str << camera.window_size[0] << std::endl;
    str << camera.window_size[1] << std::endl;
}

void VisuilizeProcessor::LoadCamera(pcl::visualization::Camera &camera, std::string file)
{
    std::ifstream str(file);

    str >> camera.clip[0];
    str >> camera.clip[1];
    str >> camera.focal[0];
    str >> camera.focal[1];
    str >> camera.focal[2];
    str >> camera.fovy;
    str >> camera.pos[0];
    str >> camera.pos[1];
    str >> camera.pos[2];
    str >> camera.view[0];
    str >> camera.view[1];
    str >> camera.view[2];
    str >> camera.window_pos[0];
    str >> camera.window_pos[1];
    str >> camera.window_size[0];
    str >> camera.window_size[1];
}

void VisuilizeProcessor::HandleFER(std::vector<double> ferdata)
{
    m_haveFerData = true;
    m_ferData = ferdata;
}

void VisuilizeProcessor::HandleFACS(std::vector<double> facsdata)
{
    m_haveFacsData = true;
    m_facsData = facsdata;
}

void VisuilizeProcessor::InitExpressionLabels()
{
    m_expressionLabels.clear();
    m_expressionLabels.push_back("Neutral");
    m_expressionLabels.push_back("Happy");
    m_expressionLabels.push_back("Surprise");
}

void VisuilizeProcessor::InitActionUnitLabels()
{
    m_actionUnitLabels.clear();
    m_actionUnitLabels.push_back("Inner Brow Raiser (AU01): ");
    m_actionUnitLabels.push_back("Brow lowerer       (AU04): ");
    m_actionUnitLabels.push_back("Cheek Raiser       (AU06): ");
    m_actionUnitLabels.push_back("Nose Wrinkler      (AU09): ");
    m_actionUnitLabels.push_back("Lip Corner Puller  (AU12): ");
    m_actionUnitLabels.push_back("Eyes Closed        (AU43): ");
}

void VisuilizeProcessor::KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender)
{
    if (event.getKeySym() == "space")
    {
        VisuilizeProcessor *_this = static_cast<VisuilizeProcessor *>(sender);
        if (_this->m_trackingProcessor.get() != nullptr)
        {
            _this->m_trackingProcessor->ResetTracker();
        }
    }
}


void VisuilizeProcessor::InitLogging()
{
    /*std::ostringstream logFileName_aux;
    std::string logFileName = logFileName_aux.str();
    logFileName_aux << "./" << m_logST->tm_year+ 1900 <<
            (boost::format("%02d") % (m_logST->tm_mon + 1)) <<
            (boost::format("%02d") % (m_logST->tm_mday)) << "_" <<
            m_logST->tm_hour << m_logST->tm_min << m_logST->tm_sec << ".log";*/

    using namespace boost::posix_time;
    std::locale loc(std::cout.getloc(), new time_facet("%Y%m%d_%H%M%S"));
    std::stringstream wss;
    wss.imbue(loc);
    ptime now = second_clock::local_time();
    wss << now;

    if (m_loggingFlag_au) {
        std::string wss_string_au = wss.str() + "_au.log";
        const char* p_au = wss_string_au.c_str();

        m_logFileStream_au.open(p_au, ios::binary);

        if (!m_logFileStream_au.is_open()) {
            cerr << "Unable to open: " << wss_string_au << "_au.log" << endl;
        } else {
            std::locale loc_aux(std::cout.getloc(), new boost::posix_time::time_facet("%Y %m %d, %H:%M:%S"));
            std::stringstream wss_aux;
            wss_aux.imbue(loc_aux);
            wss_aux << now;
            std::string wss_aux_string = wss_aux.str();
            m_logFileStream_au << "DATE; " << wss_aux_string << endl;
            m_logFileStream_au << "timestamp;FACS;AU1;AU4;AU6;AU9;AU12;AU43;[img_name;]" << endl;
        }
    }

    if (m_loggingFlag_hp) {
        std::string wss_string_hp = wss.str() + "_hp.log";
        const char* p_hp = wss_string_hp.c_str();

        m_logFileStream_hp.open(p_hp, ios::binary);

        if (!m_logFileStream_hp.is_open()) {
            cerr << "Unable to open: " << wss_string_hp << "_hp.log" << endl;
        } else {
            std::locale loc_aux(std::cout.getloc(), new boost::posix_time::time_facet("%Y %m %d, %H:%M:%S"));
            std::stringstream wss_aux;
            wss_aux.imbue(loc_aux);
            wss_aux << now;
            std::string wss_aux_string = wss_aux.str();
            m_logFileStream_hp << "DATE; " << wss_aux_string << endl;
            m_logFileStream_hp << "timestamp;HEADPOSE;YAW;TILT;" << endl;
        }
    }

    m_initLogging = true;
}

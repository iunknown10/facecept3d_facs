#include "ShowCloudProcessor.h"

#include <DataObject/CloudXYZRGBA.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>

namespace hpe
{

    ShowCloudProcessor::ShowCloudProcessor(void)
        : m_key("Cloud"), m_visualizer("Cloud"), m_first(true)
    {
    }

    ShowCloudProcessor::ShowCloudProcessor(std::string key)
        : m_key(key), m_visualizer(key), m_first(true)
    {
    }

    ShowCloudProcessor::~ShowCloudProcessor(void)
    {
    }

    void ShowCloudProcessor::Process(IDataStorage::Ptr storage)
    {
        IDataObject::Ptr obj = storage->Get(m_key);
        CloudXYZRGBA::Ptr cloudObject = std::dynamic_pointer_cast<CloudXYZRGBA>(obj);
        //CloudXYZRGBA::Ptr cloudObject(new CloudXYZRGBA(true));
        //std::string path = "/home/radu/work/cpp/FaceCept3D/FaceCept3D/TemplateTracker/build-TemplateTracker-GCC4_8-Default/template.pcd";
        //pcl::io::loadPCDFile(path, *(cloudObject->cloud));
        if (cloudObject.get() == nullptr)
        {
            return;
        }

        if (m_first)
        {
            m_visualizer.registerKeyboardCallback(&ShowCloudProcessor::KeyboardEventCallback, this);
            m_visualizer.addPointCloud<pcl::PointXYZRGBA>(cloudObject->cloud, m_key);
            //m_visualizer.addCoordinateSystem(1.0);
            m_visualizer.initCameraParameters ();
            //m_visualizer.setCameraPosition(0.528442, 0.648384, -1.67579, 0, 1, 0);
            //m_visualizer.updateCamera();
            m_first = false;
        }
        else
        {
            m_visualizer.updatePointCloud(cloudObject->cloud, m_key);
        }

        m_visualizer.spinOnce();
        //boost::this_thread::sleep(boost::posix_time::seconds(1));
    }

    void ShowCloudProcessor::KeyboardEventCallback(const pcl::visualization::KeyboardEvent &event, void *sender)
    {
        ShowCloudProcessor *_this = static_cast<ShowCloudProcessor *>(sender);
        _this->KeyPressed(event);
    }

    void ShowCloudProcessor::KeyPressed(const pcl::visualization::KeyboardEvent &event)
    {
    }

}

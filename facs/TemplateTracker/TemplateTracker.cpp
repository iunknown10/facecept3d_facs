// TemplateTracker.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"

#include "ShowTwoCloudsProcessor.h"
#include "VoxelizeProcessor.h"
#include "UIProcessor.h"
#include "VisuilizeProcessor.h"

#include <UI/PointPicker.h>
#include <Common.h>
#include <Landmarks.h>
#include <Grabber/ProviderGrabber.h>

#include <Filter/Filters.h>
#include <Filter/FunctorFilter.h>

#include <Processor/ConverterProcessor.h>
#include <Processor/DetectorProcessor.h>
#include <Processor/FilterProcessor.h>
#include "Converter/KinectDataConverter.h"

#include <boost/bind.hpp>
#include <Processor/FacialActionUnitProcessor.h>
#include <boost/date_time/posix_time/ptime.hpp>

using namespace boost::posix_time;
using namespace hpe;
namespace fs = boost::filesystem;

int main()
{
    HPESettings settings;

    OpenNIGrabber grabber;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr templateCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile(settings.GetString("Template"), *templateCloud);

    std::shared_ptr<UIProcessor> uiProcessor(new UIProcessor);
    uiProcessor->SetGrabber(&grabber);

    std::shared_ptr<DepthPreprocessingProcessor> preprocessor(new DepthPreprocessingProcessor(1, 3, 9, 5, 1.1)); // erodeSize, closingSize, gaussianSize, gaussianSigma, distanceTh

//    std::shared_ptr<DetectorProcessor> headposeDetector(new DetectorProcessor(settings.GetString("DetectorData")));

    std::shared_ptr<hpe::TrackingProcessor> templateTracker(new hpe::TrackingProcessor(templateCloud));
    Common<pcl::PointXYZRGBA>::Landmarks landmarks = LoadLandmarks<pcl::PointXYZRGBA>(settings.GetString("Landmarks"));
    templateTracker->SetTemplateLandmarks(landmarks);
    templateTracker->SetSaveLandmarks(settings.GetValue<bool>("SaveTrackingLandmarksFlag"));
    

    std::string cloudToShowKey = "FilteredOriginalCloud";
//    std::string cloudToShowKey = "Cloud";
    std::shared_ptr<VisuilizeProcessor> visualizer(new VisuilizeProcessor(cloudToShowKey));
    visualizer->SetTrackingProcessor(templateTracker);
    visualizer->SetLoggingFlag_AU(settings.GetValue<bool>("LoggingFlag_AU"));
    visualizer->SetLoggingFlag_HP(settings.GetValue<bool>("LoggingFlag_HP"));
    visualizer->SetPlotCloudFlag(settings.GetValue<bool>("PlotCloudFlag"));

    std::shared_ptr<hpe::FacialExpressionProcessor> ferProcessor(new hpe::FacialExpressionProcessor(settings.GetString("FERData")));
    ferProcessor->SubscribeFacialExpressionReadySignal(boost::bind(&VisuilizeProcessor::HandleFER, visualizer.get(), _1));

//    std::shared_ptr<hpe::FacialActionUnitProcessor> facsProcessor(new hpe::FacialActionUnitProcessor(settings.GetString("FACSData"), {1, 4, 6, 9, 12, 43}));
//    facsProcessor->SubscribeFacialActionUnitReadySignal(boost::bind(&VisuilizeProcessor::HandleFACS, visualizer.get(), _1));
//    bool saveImagesFlag = settings.GetValue<bool>("SaveFaceImageFlag");
//    if (saveImagesFlag) {
//        facsProcessor->SetSaveImageFlag(saveImagesFlag);
//
//        std::string imgRoot = settings.GetString("SaveFaceImageFolder");
//
//        facsProcessor->SetSaveImageRoot(imgRoot);
//
//        std::locale loc(std::cout.getloc(), new time_facet("%Y%m%d_%H%M%S"));
//        std::stringstream wss;
//        wss.imbue(loc);
//        ptime now = second_clock::local_time();
//        wss << imgRoot << now << "/";
//
//        const std::string localPath = wss.str();
//
//        if (fs::exists(localPath) == false) {
//            fs::create_directories(localPath);
//            cout << "Successfully created: " << localPath << endl;
//            facsProcessor->SetSaveImageLocalFolder(localPath);
//        }
//    }

    std::shared_ptr<FunctorFilter<pcl::PointXYZRGBA>> functorFilter(new FunctorFilter<pcl::PointXYZRGBA>(
    [&settings](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)->pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  {
        return PassThrough<pcl::PointXYZRGBA>(cloud, 0., settings.GetValue<float>("DistanceToShow"), "z");
    }));

    std::shared_ptr<FilterProcessor> filterProcessor(new FilterProcessor(functorFilter, "Cloud", "FilteredOriginalCloud"));

    std::shared_ptr<ConverterProcessor> converterProcessor(new hpe::ConverterProcessor());
    //converterproc->SetCloudKey("OriginalCloud");
    ConverterProcessor::ConverterPtr converter(new KinectDataConverter);
    converterProcessor->SetConverter(converter);

    //grabber.AddProcessor(uiProcessor);
    grabber.AddProcessor(preprocessor); // filters depth data

    grabber.AddProcessor(converterProcessor); // combines (filtered) depth and rgb into a point cloud representation saved in "Cloud"

    grabber.AddProcessor(filterProcessor); // filters the point cloud representation (in this case a parametrized (by "DistanceToShow") "PassThrough" filter is being used)
    // and returns the result in "FilteredOriginalCloud"

    grabber.AddProcessor(IProcessor::Ptr(new VoxelizeProcessor(0.01))); // applies yet another filter to the "Cloud" and returns the result in "Cloud"
    //grabber.AddProcessor(headposeDetector);

//    grabber.AddProcessor(headposeDetector);
    grabber.AddProcessor(templateTracker);
//    grabber.AddProcessor(facsProcessor);
    grabber.AddProcessor(ferProcessor);
    grabber.AddProcessor(visualizer);

    grabber.Start();

    return 0;
}


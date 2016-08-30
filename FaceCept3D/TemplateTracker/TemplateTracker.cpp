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


using namespace hpe;

int main()
{
    HPESettings settings;

    OpenNIGrabber grabber;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr templateCloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::io::loadPCDFile(settings.GetString("Template"), *templateCloud);

    std::shared_ptr<UIProcessor> uiProcessor(new UIProcessor);
    uiProcessor->SetGrabber(&grabber);

    std::shared_ptr<DepthPreprocessingProcessor> preprocessor(new DepthPreprocessingProcessor(1, 3, 13, 7, 1.2));

    std::shared_ptr<hpe::TrackingProcessor> templateTracker(new hpe::TrackingProcessor(templateCloud));
    Common<pcl::PointXYZRGBA>::Landmarks landmarks = LoadLandmarks<pcl::PointXYZRGBA>(settings.GetString("Landmarks"));
    templateTracker->SetTemplateLandmarks(landmarks);
    templateTracker->SetSaveLandmakrs(false);
    

    std::string cloudToShowKey = "FilteredOriginalCloud";
//    std::string cloudToShowKey = "Cloud";
    std::shared_ptr<VisuilizeProcessor> visualizer(new VisuilizeProcessor(cloudToShowKey));
    visualizer->SetTrackingProcessor(templateTracker);

    //std::shared_ptr<hpe::FacialExpressionProcessor> ferProcessor(new hpe::FacialExpressionProcessor(settings.GetString("FERData")));
    //ferProcessor->SubscribeFacialExpressionReadySignal(boost::bind(&VisuilizeProcessor::HandleFER, visualizer.get(), _1));


    std::shared_ptr<FunctorFilter<pcl::PointXYZRGBA>> functorFilter(new FunctorFilter<pcl::PointXYZRGBA>(
    [&settings](pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)->pcl::PointCloud<pcl::PointXYZRGBA>::Ptr  {
        return PassThrough<pcl::PointXYZRGBA>(cloud, 0., settings.GetValue<float>("DistanceToShow"), "z");
    }));

    std::shared_ptr<FilterProcessor> filterProcessor(new FilterProcessor(functorFilter, "Cloud", "FilteredOriginalCloud"));

    std::shared_ptr<ConverterProcessor> converterProcessor(new hpe::ConverterProcessor());
    //converterproc->SetCloudKey("OriginalCloud");
    ConverterProcessor::ConverterPtr converter(new KinectDataConverter);
    converterProcessor->SetConverter(converter);

    //grabber.AddProcessor(IProcessor::Ptr(new ConverterProcessor(ConverterProcessor::ConverterPtr(new KinectDataConverter))));

    //grabber.AddProcessor(uiProcessor);
    grabber.AddProcessor(preprocessor);

    grabber.AddProcessor(converterProcessor);

    grabber.AddProcessor(filterProcessor);
    grabber.AddProcessor(IProcessor::Ptr(new VoxelizeProcessor(0.005)));
    grabber.AddProcessor(templateTracker);
    //grabber.AddProcessor(ferProcessor);
    grabber.AddProcessor(visualizer);

    grabber.Start();

    return 0;
}


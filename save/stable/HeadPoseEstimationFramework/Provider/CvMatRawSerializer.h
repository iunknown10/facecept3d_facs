#ifndef CVMATRAWWRITER_H
#define CVMATRAWWRITER_H

#include <fstream>

#include <opencv2/opencv.hpp>

namespace hpe
{
    class CvMatRawSerializer
    {
        public:
            void Save(std::string file, cv::Mat &mat);
            void Load(std::string file, cv::Mat &mat);

            void SaveCSVFloat(std::string file, cv::Mat &mat);

        private:
            void WriteHeader(std::ostream &stream, cv::Mat &mat);
            void WriteData(std::ostream &stream, cv::Mat &mat);
            void ReadHeader(std::istream &stream, cv::Mat &mat);
            void ReadData(std::istream &stream, cv::Mat &mat);
    };
}

#endif

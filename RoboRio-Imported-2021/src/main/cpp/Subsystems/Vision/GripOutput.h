#pragma once
#include "vision/VisionPipeline.h"

#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {

/**
* A representation of the different types of blurs that can be used.
*
*/
enum BlurType {
	BOX, GAUSSIAN, MEDIAN, BILATERAL
};
/**
* GripOutput class.
* 
* An OpenCV pipeline generated by GRIP.
*/
class GripOutput : public frc::VisionPipeline {
	private:
		cv::Mat resizeImageOutput;
		cv::Mat blurOutput;
		cv::Mat hslThresholdOutput;
		void resizeImage(cv::Mat &, double , double , int , cv::Mat &);
		void blur(cv::Mat &, BlurType &, double , cv::Mat &);
		void hslThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);

	public:
		GripOutput();
		void Process(cv::Mat& source0) override;
		cv::Mat* GetResizeImageOutput();
		cv::Mat* GetBlurOutput();
		cv::Mat* GetHslThresholdOutput();
};


} // end namespace grip



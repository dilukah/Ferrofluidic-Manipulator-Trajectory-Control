#ifndef VISION_H
#define VISION_H
#undef usingOpenCVDisplay
#define usingCamera

//#undef usingCamera

#define BinaryDebugDisplay
#undef BinaryDebugDisplay

#include "opencv2/imgproc.hpp"
// basic visp include
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpIoTools.h>
#include <visp3/core/vpMath.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/core/vpImage.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3\core\vpImageConvert.h>
#include <visp3\blob\vpDot.h>

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpVideoWriter.h>

//Includes for testing performance of Detection and tracking algorithms
#include <visp3/gui/vpDisplayX.h>
#include <visp3/imgproc/vpImgproc.h>
#include <visp3/core/vpImageTools.h>
#include <visp3/blob/vpDot2.h>


// various tracking algorithm
#include <visp3/tt/vpTemplateTrackerSSD.h>
#include <visp3/tt/vpTemplateTrackerSSDESM.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerSSDForwardCompositional.h>
#include <visp3/tt/vpTemplateTrackerSSDInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerZNCCForwardAdditional.h>
#include <visp3/tt/vpTemplateTrackerZNCCInverseCompositional.h>
#include <visp3/tt/vpTemplateTrackerWarpAffine.h>
#include <visp3/tt/vpTemplateTrackerWarpHomography.h>
#include <visp3/tt/vpTemplateTrackerWarpHomographySL3.h>
#include <visp3/tt/vpTemplateTrackerWarpRT.h>
#include <visp3/tt/vpTemplateTrackerWarpSRT.h>
#include <visp3/tt/vpTemplateTrackerWarpTranslation.h>
#include <visp3/tt_mi/vpTemplateTrackerMIESM.h>
#include <visp3/tt_mi/vpTemplateTrackerMIForwardAdditional.h>
#include <visp3/tt_mi/vpTemplateTrackerMIForwardCompositional.h>
#include <visp3/tt_mi/vpTemplateTrackerMIInverseCompositional.h>




// camera include
#include <visp3/sensor/vpFlyCaptureGrabber.h>


#include <visp3/io/vpVideoWriter.h>

// other includes
#include <string>

typedef enum {
	WARP_AFFINE,
	WARP_HOMOGRAPHY,
	WARP_HOMOGRAPHY_SL3,
	WARP_SRT,
	WARP_TRANSLATION,
	WARP_RT,
	WARP_LAST
} WarpType;

typedef enum {
	TRACKER_SSD_ESM,
	TRACKER_SSD_FORWARD_ADDITIONAL,
	TRACKER_SSD_FORWARD_COMPOSITIONAL,
	TRACKER_SSD_INVERSE_COMPOSITIONAL, // The most efficient
	TRACKER_ZNCC_FORWARD_ADDITIONEL,
	TRACKER_ZNCC_INVERSE_COMPOSITIONAL,
	TRACKER_MI_ESM,
	TRACKER_MI_FORWARD_ADDITIONAL,
	TRACKER_MI_FORWARD_COMPOSITIONAL,
	TRACKER_MI_INVERSE_COMPOSITIONAL, // The most efficient
	TRACKER_LAST
} TrackerType;

class Vision
{
public:

	// Constructor
	Vision();
	Vision(bool imgType, bool halfDisplay, double recording_fps);

	// Initialize function
	void Initialize(int width, int height);
	void InitializeBinary(int threshold);
	void InitializeBinary2();
//	void InitializeVideo(const std::string fileName);


	// Acquisition and display function
	void AcquireImage();
	void ConvertToBinary(int threshold);

	void DisplayImage();
	void DisplayBinary();
	void DisplayBinary2();

	void Flush();
	void FlushBinary();
	void FlushBinary2();

	// Tracking function
	int InitializeBlobTracking();

	void TrackTemplate();
	int TrackBlob();

	int InitializeBlobTrackingViaIP(vpImagePoint &ip);

	void DisplayTemplateTracker();
	void DisplayBlobTracker();
	void DisplayBlobTrackerBinary();

	void GetTemplateTrackerCoG(vpImagePoint &ip_Track);
	void GetBlobTrackerCoG(vpImagePoint &ip_Track);

	void RecordImagePoint(vpImagePoint ip_Track);

	// Homography function
	void FillHomographyVector(std::vector<double> &x_Vector, std::vector<double> &y_Vector, vpImagePoint ip_Track);

	// Utility function
	void DisplayPointList(std::vector<double> uList, std::vector<double> vList, vpColor color);
	void DisplayPointList(vpMatrix iMatrix, vpColor color);

	void DisplayPoint(double u, double v);
	void DisplayPoint(vpImagePoint imagePoint);
	void DisplayPointBinary(double u, double v);
	void DisplayPointBinary(vpImagePoint imagePoint);

	void DisplayArrow(vpImagePoint srcImagePoint, vpImagePoint destImagePoint);
	void DisplayArrow(vpImagePoint srcImagePoint, vpImagePoint destImagePoint, vpColor color);
	void DisplayArrow(vpColVector srcVector, vpColVector destVector, vpColor color);

	

	void DisplayText(std::string txt);
	void DisplayText(std::string txt, int x, int y, vpColor color);

	void ShowClickedPosition();
	void ShowClickedPositionBinary();

	// Recording function
	void StartRecordingVideo();
	void StartRecordingVideoBinary();

	void AddFrameToVideo();
	void AddFrameToVideoBinary();

	void StopRecordingVideo();
	void StopRecordingVideoBinary();

	void TakeAScreenShot();
	void TakeAScreenShotBinary();
	
	int getImageCols();
	int getImageRows();
	bool getClickedPosition(vpImagePoint *clickPos);
	void drawCircle(vpImagePoint center, vpColor color, bool fill);

	void drawCircleWithRadius(vpImagePoint center, int radius, vpColor color, bool fill);
	void drawCross(vpImagePoint center, vpColor color);
	void drawRectangle(vpRect rect, vpColor color, bool fill);

	// Variables
	vpFlyCaptureGrabber *camera;

#ifdef	usingOpenCVDisplay
		vpDisplayOpenCV *binaryDisplay;
		vpDisplayOpenCV *display;
#else
		vpDisplayGDI *display;
		vpDisplayGDI *binaryDisplay;
		vpDisplayGDI* binaryDisplay2;
#endif

	vpImagePoint clickPoint;
	vpImagePoint binaryClickPoint;

	vpDot *dotTracker;

private:
	vpImage<unsigned char> grayImage;
	
	vpImage<unsigned char> grayImageHalf;
	vpImage<vpRGBa> colorImage;
	vpImage<vpRGBa> colorImageHalf;
	cv::Mat srcImageOpenCV;

	cv::Mat particleDetectorBinaryImageOpenCV;
	cv::Mat particleDetectorBinaryImageOpenCVSub;

	cv::Mat particleDetectorGrayImageOpenCV;
	cv::Mat particleDetectorGrayImageOpenCVSub;
	cv::Mat grayImageOpenCV;

	vpImage<unsigned char> binaryImage;
	vpImage<unsigned char> binaryImage2;
	cv::Mat binaryImageOpenCV;

	vpTemplateTrackerWarp *warp;
	vpTemplateTracker *templateTracker;

	vpVideoWriter *writer;
	vpVideoWriter *binaryWriter;

	vpMouseButton::vpMouseButtonType button;

	double videoFPS;
	double recordingVideoFPS;


	std::ofstream cameraPosition;

	int minDetectRadius;
	int	maxDetectRadius;
	std::vector<cv::Vec3f> circles;

	bool detectorInitialized;
	bool foundParticle;
	


	bool isColor;
	bool useHalfDisplay;

	bool opt_click_allowed;
	bool opt_display;
	bool opt_pyramidal;
	long opt_last_frame;

	bool white_foreground = false;

	int num;
};

#endif // VISION_H

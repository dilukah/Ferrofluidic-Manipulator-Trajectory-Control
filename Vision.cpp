/*
Vision.cpp - Video processing algorithms (Adapted from Jean-Antoine Seon)
Date: 2021-07-07
Modified by: P. A. Diluka Harischandra
*/

#include "Vision.h"
 
// Default constructor
Vision::Vision() {}

/**====================================================
* Overloaded constructor
* Input: Image type, Display mode and recording frames 
* per second
*======================================================*/
Vision::Vision(bool imgType, bool halfDisplay, double recording_fps) :cameraPosition("CameraPosition.log")
{
	// video variables
	recordingVideoFPS = recording_fps;
	isColor = imgType;
	useHalfDisplay = halfDisplay;

	// init pointer
	warp = NULL;
	templateTracker = NULL;
	dotTracker = NULL;
	
	camera = new vpFlyCaptureGrabber();		

#ifdef usingOpenCVDisplay
	display = new vpDisplayOpenCV();
	binaryDisplay = new vpDisplayOpenCV();
#else
	display = new vpDisplayGDI();

	binaryDisplay = new vpDisplayGDI();
	binaryDisplay2 = new vpDisplayGDI();

#endif
	writer = NULL;
	binaryWriter = NULL;

    // init screenshot counter
	num = 0;
}

/**====================================================
* Function to initialize the vision module (open the camera and initialize the display)
* Input: size of the image
* Output: NULL
*======================================================*/

void Vision::Initialize(int width, int height)
{
	std::cout << "Number of cameras detected: " << camera->getNumCameras() << std::endl;
	camera->setCameraIndex(0);			// Selected the first camera
	camera->getCameraInfo(std::cout);	// Display camera info
	camera->setShutter(true);			// Turn auto shutter on
	camera->setGain(true);				// Turn auto gain on
	camera->setFormat7VideoMode(FlyCapture2::MODE_1, FlyCapture2::PIXEL_FORMAT_RAW8, width, height);

	// 0: gray scale image - 1: color image
	if (isColor)
	{
		camera->open(colorImage);
		if (useHalfDisplay)
		{
			colorImage.halfSizeImage(colorImageHalf);
			display->init(colorImageHalf, 0, 0, "NegMag");
		}
		else
		{
			display->init(colorImage, 0, 0, "NegMag");
		}
	}
	else
	{
		camera->open(grayImage);
		if (useHalfDisplay)
		{
			grayImage.halfSizeImage(grayImageHalf);
			display->init(grayImageHalf, 0, 0, "NegMag");
		}
		else
		{
			display->init(grayImage, 0, 0, "NegMag");
		}
	}
}

/**====================================================
* Function to get columns of the image.
* Input: NULL
* Output: Number of columns
*======================================================*/
int Vision::getImageCols()
{
	return grayImage.getCols();
}

/**====================================================
* Function to get rows of the image.
* Input: NULL
* Output: Number of rows
*======================================================*/
int Vision::getImageRows()
{
	return grayImage.getRows();
}
/**====================================================
* Function get clicked position of the image
* Input: Pointer to the  clicked position variable
* Output: bool (1 if the user clicked)
*======================================================*/
bool Vision::getClickedPosition(vpImagePoint* clickPos)
{
	bool userClicked = 0;
	vpImagePoint tmp;
	if(isColor)
	{
		userClicked = vpDisplay::getClick(colorImage,tmp , false);
	}
	else
	{
		userClicked = vpDisplay::getClick(grayImage, tmp, false);
	}
	*clickPos = tmp;
	return userClicked;
}

/**====================================================
* Function to draw circles
* Input: center point, color, fill status.
* Output: NULL
*======================================================*/
void Vision::drawCircle(vpImagePoint center, vpColor color, bool fill )
{
	if (isColor)
	{
		vpDisplay::displayCircle(colorImage, center, 10, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCircle(colorImageHalf, center, 10, color, fill, 1);
		}
	}
	else
	{
		vpDisplay::displayCircle(grayImage, center, 10, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCircle(grayImageHalf, center, 10, color, fill, 1);
		}
	}
	
}

/**====================================================
* Function to draw circles
* Input: center point, color, fill status.
* Output: NULL
*======================================================*/
void Vision::drawCircleWithRadius(vpImagePoint center, int radius, vpColor color, bool fill)
{
	if (isColor)
	{
		vpDisplay::displayCircle(colorImage, center, radius, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCircle(colorImageHalf, center, radius, color, fill, 1);
		}
	}
	else
	{
		vpDisplay::displayCircle(grayImage, center, radius, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCircle(grayImageHalf, center, radius, color, fill, 1);
		}
	}

}

/**====================================================
* Function to draw rectangles
* Input: center point, color, fill status.
* Output: NULL
*======================================================*/
void Vision::drawRectangle(vpRect rect, vpColor color, bool fill)
{
	if (isColor)
	{
		vpDisplay::displayRectangle(colorImage, rect, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayRectangle(colorImageHalf, rect, color, fill, 1);
		}
	}
	else
	{
		vpDisplay::displayRectangle(grayImage, rect, color, fill, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayRectangle(grayImageHalf, rect, color, fill, 1);
		}
	}

}

/**====================================================
* Function to draw cross
* Input: center point, color.
* Output: NULL
*======================================================*/
void Vision::drawCross(vpImagePoint center, vpColor color)
{	
	if (isColor)
	{
		vpDisplay::displayCross(colorImage, center.get_i(), center.get_j(), 25, color, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(colorImageHalf, center.get_i(), center.get_j(), 25, color, 1);
		}
	}
	else
	{
		vpDisplay::displayCross(grayImage, center.get_i(), center.get_j(), 25, color, 1);
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(grayImageHalf, center.get_i(), center.get_j(), 25, color, 1);
		}
	}
}


/**====================================================
* Function to init the display for the tresholded image
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::InitializeBinary(int threshold)
{
	ConvertToBinary(threshold); 
	binaryDisplay->init(binaryImage, 0, 0, "Binary image");	// Init the display
}

/**====================================================
* Function to init the display for the tresholded image
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::InitializeBinary2()
{
	vpImageConvert::convert(particleDetectorBinaryImageOpenCVSub, binaryImage2);
	binaryDisplay2->init(binaryImage, 0, 0, "Binary image 2");	// Init the display
}


/**====================================================
* Function to grab an image
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::AcquireImage()
{
	if (isColor)
	{
		camera->acquire(colorImage);
		if (useHalfDisplay)
		{
			colorImage.halfSizeImage(colorImageHalf);
		}
	}
	else
	{
		camera->acquire(grayImage);
		if (useHalfDisplay)
		{
			grayImage.halfSizeImage(grayImageHalf);
		}
	}
}

/**====================================================
* Function to convert the image to a binary image
* Input: threshold value (from 0 to 255)
* Output: NULL
*======================================================*/
void Vision::ConvertToBinary(int threshold)
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpImageConvert::convert(colorImageHalf, grayImageHalf);
			vpImageConvert::convert(grayImageHalf, srcImageOpenCV);
			cv::threshold(srcImageOpenCV, binaryImageOpenCV, threshold, 255, CV_THRESH_BINARY);
			vpImageConvert::convert(binaryImageOpenCV, binaryImage);
		}
		else
		{
			vpImageConvert::convert(colorImage, grayImage);
			vpImageConvert::convert(grayImage, srcImageOpenCV);
			cv::threshold(srcImageOpenCV, binaryImageOpenCV, threshold, 255, CV_THRESH_BINARY);
			vpImageConvert::convert(binaryImageOpenCV, binaryImage);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpImageConvert::convert(grayImageHalf, srcImageOpenCV);
			cv::threshold(srcImageOpenCV, binaryImageOpenCV, threshold, 255, CV_THRESH_BINARY);
			vpImageConvert::convert(binaryImageOpenCV, binaryImage);
		}
		else
		{
			vpImageConvert::convert(grayImage, srcImageOpenCV);
			cv::threshold(srcImageOpenCV, binaryImageOpenCV, threshold, 255, CV_THRESH_BINARY);
			vpImageConvert::convert(binaryImageOpenCV, binaryImage);

		}
	}
}

/**====================================================
* Function to add the last grabbed image to the display
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::DisplayImage()
{
	if (isColor)
	{
		if (useHalfDisplay)
			display->display(colorImageHalf);
		else
			display->display(colorImage);
	}
	else
	{
		if (useHalfDisplay)
			display->display(grayImageHalf);
		else
			display->display(grayImage);
	}
}

/**====================================================
* Function to display the binary image
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::DisplayBinary()
{
	binaryDisplay->display(binaryImage);
}


/**====================================================
* Function to display the opencv processed binary image
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::DisplayBinary2()
{
	binaryDisplay2->display(binaryImage2);
}

/**====================================================
* Function to flush the display
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::Flush()
{
	if (isColor)
	{
		if (useHalfDisplay)
			display->flush(colorImageHalf);
		else
			display->flush(colorImage);
	}
	else
	{
		if (useHalfDisplay)
			display->flush(grayImageHalf);
		else
			display->flush(grayImage);
	}
}

/**====================================================
* Function to flush the display (binary version)
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::FlushBinary()
{
	binaryDisplay->flush(binaryImage);
}

/**====================================================
* Function to flush the display (binary version)
* Input: NULL
* Output: NULL
*======================================================*/

void Vision::FlushBinary2()
{
	binaryDisplay2->flush(binaryImage2);
}



/**====================================================
* Function to initialize the tracking in the binary image
* via mouse left click
* Input: NULL
* Output: 1 : Initialized, 0 :Cannot Initialize
*======================================================*/
int Vision::InitializeBlobTracking()
{
	try{
	dotTracker = NULL;
	dotTracker = new vpDot();
	if (isColor)
	{
		vpImageConvert::convert(colorImage, grayImage);
	}

	vpImagePoint tmp;
	if (isColor)
	{
		vpDisplay::getClick(colorImage, tmp, true);
	}
	else
	{
		vpDisplay::getClick(grayImage, tmp, true);
	}
	
	dotTracker->initTracking(binaryImage,tmp);

	dotTracker->setGraphics(true);
	}
	catch (...)
	{
		std::cout << "Could not initialize tracker" << std::endl;
		return 0;
	}
	return 1;
}


/**====================================================
* Function to initialize the tracking in the binary image
* via given point
* Input: NULL
* Output: 1 : Initialized, 0 :Cannot Initialize
*======================================================*/
int Vision::InitializeBlobTrackingViaIP(vpImagePoint &ip)
{
	try {
		dotTracker = NULL;
		dotTracker = new vpDot();
		

		dotTracker->initTracking(binaryImage, ip);

		dotTracker->setGraphics(true);
	}
	catch (...)
	{
		std::cout << "Could not initialize tracker" << std::endl;
		return 0;
	}
	return 1;
}


/**====================================================
* Function to track the template in the current image
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::TrackTemplate()
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpImageConvert::convert(colorImageHalf, grayImageHalf);
			templateTracker->track(grayImageHalf);
		}
		else
		{
			vpImageConvert::convert(colorImage, grayImage);
			templateTracker->track(grayImage);
		}
	}
	else
	{
		if (useHalfDisplay)
			templateTracker->track(grayImageHalf);
		else
			templateTracker->track(grayImage);
	}
}

/**====================================================
* Function to track the blob
* Input: NULL
* Output: 1 : Tracking, 0 :Cannot Track
*======================================================*/
int Vision::TrackBlob()
{
	try 
	{
		dotTracker->track(binaryImage);
	}
	catch (...) 
	{
		return 0;
	}
	return 1;
	//dotTracker->track(grayImage);
}

/**====================================================
* Function to display the tracker in the image
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::DisplayTemplateTracker()
{
	if (isColor)
	{
		if (useHalfDisplay)
			templateTracker->display(colorImageHalf, vpColor::red, 3);
		else
			templateTracker->display(colorImage, vpColor::red, 3);
	}
	else
	{
		if (useHalfDisplay)
			templateTracker->display(grayImageHalf, vpColor::red, 3);
		else
			templateTracker->display(grayImage, vpColor::red, 3);
	}
}

/**====================================================
* Function to display the tracker in the image
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::DisplayBlobTracker()
{
	std::list<vpImagePoint> edges = dotTracker->getEdges();
	vpImagePoint cog = dotTracker->getCog();
	if (isColor)
	{
		if (useHalfDisplay)
			dotTracker->display(colorImageHalf, cog, edges, vpColor::darkRed, 1);
		else
			dotTracker->display(colorImage, cog, edges, vpColor::darkRed, 1);
	}
	else
	{
		if (useHalfDisplay)
			dotTracker->display(grayImageHalf, cog, edges, vpColor::red, 1);
		else
			dotTracker->display(grayImage, cog, edges, vpColor::red, 1);
	}
}

/**====================================================
* Function to display the tracker in the image
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::DisplayBlobTrackerBinary()
{
	std::list<vpImagePoint> edges = dotTracker->getEdges();
	vpImagePoint cog = dotTracker->getCog();
	dotTracker->display(binaryImage, cog, edges, vpColor::darkRed, 1);
}

/**====================================================
* Function to fill the tracking center variable with the last tracked position
* Input: NULL
* Output: vector containing the tracking center
*======================================================*/
void Vision::GetTemplateTrackerCoG(vpImagePoint &ip_Track)
{
	vpTemplateTrackerTriangle triangle;
	std::vector<vpImagePoint> corners;
	vpColVector p;
	vpTemplateTrackerZone zone_ref = templateTracker->getZoneRef();
	vpTemplateTrackerZone zone_warped;
	double bx = 0;
	double by = 0;

	p = templateTracker->getp();
	warp->warpZone(zone_ref, p, zone_warped);

	for (int i = 0; i < zone_warped.getNbTriangle(); i++)
	{
		zone_warped.getTriangle(i, triangle);
		triangle.getCorners(corners);
		bx = bx + corners[0].get_u() + corners[1].get_u() + corners[2].get_u();
		by = by + corners[0].get_v() + corners[1].get_v() + corners[2].get_v();
	}

	bx = bx / 6.0;
	by = by / 6.0;

	ip_Track.set_uv(bx, by);
	ip_Track.set_ij(by, bx);
}

/**====================================================
* Function to get the center of gravity of the tracked blob
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::GetBlobTrackerCoG(vpImagePoint &ip_Track)
{
	ip_Track = dotTracker->getCog();
}

/**====================================================
* Function to save the tracking center variable
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::RecordImagePoint(vpImagePoint imagePoint)
{
	cameraPosition << imagePoint.get_u() << " " << imagePoint.get_v() << std::endl;
}

/**====================================================
* Function to fill the vector used for homography computation
* Input: vector containing the x coordinate, vector containing the y coordinates, image point to add to the vector
* Output: NULL
*======================================================*/
void Vision::FillHomographyVector(std::vector<double> &x_Vector, std::vector<double> &y_Vector, vpImagePoint ip_Track)
{
	x_Vector.push_back(ip_Track.get_u());
	y_Vector.push_back(ip_Track.get_v());
}

/**====================================================
* Function to display a list of points
* Input: vector containing the x coordinate, vector containing the y coordinates
* Output: NULL
*======================================================*/
void Vision::DisplayPointList(std::vector<double> uList, std::vector<double> vList, vpColor color)
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			for (int i = 0; i < uList.size(); i++)
			{
				vpDisplay::displayCross(colorImageHalf, vList[i], uList[i], 15, color, 2);
			}
		}
		else
		{
			for (int i = 0; i < uList.size(); i++)
			{
				vpDisplay::displayCross(colorImage, vList[i], uList[i], 15, color, 2);
			}
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			for (int i = 0; i < uList.size(); i++)
			{
				vpDisplay::displayCross(grayImageHalf, vList[i], uList[i], 15, color, 2);
			}
		}
		else
		{
			for (int i = 0; i < uList.size(); i++)
			{
				vpDisplay::displayCross(grayImage, vList[i], uList[i], 15, color, 2);
			}
		}
	}
}

/**====================================================
* Function to display a list of points
* Input: matrix containing the list of points, color used for the display
* Output: NULL
*======================================================*/
void Vision::DisplayPointList(vpMatrix iMatrix, vpColor color)
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			for (int i = 0; i < iMatrix.getCols(); i++)
			{
				vpDisplay::displayCross(colorImageHalf, iMatrix[1][i], iMatrix[0][i], 15, color, 2);
			}
		}
		else
		{
			for (int i = 0; i < iMatrix.getCols(); i++)
			{
				vpDisplay::displayCross(colorImage, iMatrix[1][i], iMatrix[0][i], 15, color, 2);
			}
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			for (int i = 0; i < iMatrix.getCols(); i++)
			{
				vpDisplay::displayCross(grayImageHalf, iMatrix[1][i], iMatrix[0][i], 15, color, 2);
			}
		}
		else
		{
			for (int i = 0; i < iMatrix.getCols(); i++)
			{
				vpDisplay::displayCross(grayImage, iMatrix[1][i], iMatrix[0][i], 15, color, 2);
			}
		}
	}
}

/**====================================================
* Function to display a single point
* Input: x coordinate, y coordinate
* Output: NULL
*======================================================*/
void Vision::DisplayPoint(double u, double v)
{
	std::stringstream ss;
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(colorImageHalf, v, u, 15, vpColor::darkRed, 2);
			//ss << "(" << u << ";" << v << ")";
			//std::string s = ss.str();
			//display->displayText(colorImageHalf, v, u, s, vpColor::red);
		}
		else
		{
			vpDisplay::displayCross(colorImage, v, u, 15, vpColor::darkRed, 2);
			//ss << "(" << u << ";" << v << ")";
			//std::string s = ss.str();
			//display->displayText(colorImage, v, u, s, vpColor::red);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(grayImageHalf, v, u, 15, vpColor::darkRed, 2);
			//ss << "(" << u << ";" << v << ")";
			//std::string s = ss.str();
			//display->displayText(grayImageHalf, v, u, s, vpColor::red);
		}
		else
		{
			vpDisplay::displayCross(grayImage, v, u, 15, vpColor::darkRed, 2);
			//ss << "(" << u << ";" << v << ")";
			//std::string s = ss.str();
			//display->displayText(grayImage, v, u, s, vpColor::red);
		}
	}
}

/**====================================================
* Function to display a single point
* Input: image point containing the coordinates
* Output: NULL
*======================================================*/
void Vision::DisplayPoint(vpImagePoint imagePoint)
{
	std::stringstream ss;
	double u = imagePoint.get_u();
	double v = imagePoint.get_v();
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(colorImageHalf, imagePoint, 15, vpColor::darkGreen, 2);
			ss << "(" << u << ";" << v << ")";
			std::string s = ss.str();
			display->displayText(colorImageHalf, v, u, s, vpColor::red);
		}
		else
		{
			vpDisplay::displayCross(colorImage, imagePoint, 15, vpColor::darkGreen, 2);
			ss << "(" << u << ";" << v << ")";
			std::string s = ss.str();
			display->displayText(colorImage, v, u, s, vpColor::red);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayCross(grayImageHalf, imagePoint, 15, vpColor::darkGreen, 2);
			ss << "(" << u << ";" << v << ")";
			std::string s = ss.str();
			display->displayText(grayImageHalf, v, u, s, vpColor::red);
		}
		else
		{
			vpDisplay::displayCross(grayImage, imagePoint, 15, vpColor::darkGreen, 2);
			ss << "(" << u << ";" << v << ")";
			std::string s = ss.str();
			display->displayText(grayImage, v, u, s, vpColor::red);
		}
	}
}

/**====================================================
* Function to display a single point
* Input: x coordinate, y coordinate
* Output: NULL
*======================================================*/
void Vision::DisplayPointBinary(double u, double v)
{
	std::stringstream ss;
	vpDisplay::displayCross(binaryImage, v, u, 15, vpColor::darkRed, 2);
	ss << "(" << u << ";" << v << ")";
	std::string s = ss.str();
	binaryDisplay->displayText(binaryImage, v, u, s, vpColor::red);
}

/**====================================================
* Function to display a single point in the binary image
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::DisplayPointBinary(vpImagePoint imagePoint)
{
	std::stringstream ss;
	double u = imagePoint.get_u();
	double v = imagePoint.get_v();
	vpDisplay::displayCross(binaryImage, imagePoint, 10, vpColor::darkGreen, 5);
	ss << "(" << u << ";" << v << ")";
	std::string s = ss.str();
	binaryDisplay->displayText(binaryImage, v, u, s, vpColor::red);
}

/**====================================================
* Function to display an arrow between two point
* Input: image point containing the coordinates of the src and destination of the arrow
* Output: NULL
*======================================================*/
void Vision::DisplayArrow(vpImagePoint srcImagePoint, vpImagePoint destImagePoint)
{
	std::stringstream ss;
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(colorImageHalf, srcImagePoint, destImagePoint, vpColor::darkGreen, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(colorImage, srcImagePoint, destImagePoint, vpColor::darkGreen, 4, 2, 1);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(grayImageHalf, srcImagePoint, destImagePoint, vpColor::darkGreen, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(grayImage, srcImagePoint, destImagePoint, vpColor::darkGreen, 4, 2, 1);
		}
	}
}

/**====================================================
* Function to display an arrow between two point
* Input: image point containing the coordinates of the src and destination of the arrow
* Output: NULL
*======================================================*/
void Vision::DisplayArrow(vpImagePoint srcImagePoint, vpImagePoint destImagePoint, vpColor color)
{
	std::stringstream ss;
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(colorImageHalf, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(colorImage, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(grayImageHalf, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(grayImage, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
	}
}

/**====================================================
* Function to display an arrow between two point
* Input: vectors containing the coordinates of the src and destination of the arrow
* Output: NULL
*======================================================*/
void Vision::DisplayArrow(vpColVector srcVector, vpColVector destVector, vpColor color)
{
	vpImagePoint srcImagePoint, destImagePoint;
	srcImagePoint.set_u(srcVector[0]);
	srcImagePoint.set_v(srcVector[1]);
	destImagePoint.set_u(destVector[0]);
	destImagePoint.set_v(destVector[1]);
	std::stringstream ss;
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(colorImageHalf, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(colorImage, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::displayArrow(grayImageHalf, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
		else
		{
			vpDisplay::displayArrow(grayImage, srcImagePoint, destImagePoint, color, 4, 2, 1);
		}
	}
}


/**====================================================
* Function to display a text
* Input: string containing the text to be displayed
* Output: NULL
*======================================================*/
void Vision::DisplayText(std::string txt)
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			display->displayText(colorImageHalf, 15, 15, txt, vpColor::red);
		}
		else
		{
			display->displayText(colorImage, 15, 15, txt, vpColor::red);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			display->displayText(grayImageHalf, 15, 15, txt, vpColor::red);
		}
		else
		{
			display->displayText(grayImage, 15, 15, txt, vpColor::red);
		}
	}
}

/**====================================================
* Function to display a text
* Input: string containing the text to be displayed, x position, y position, color
* Output: NULL
*======================================================*/
void Vision::DisplayText(std::string txt, int x, int y, vpColor color)
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			display->displayText(colorImageHalf, y, x, txt, color);
		}
		else
		{
			display->displayText(colorImage, y, x, txt, color);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			display->displayText(grayImageHalf, y, x, txt, color);
		}
		else
		{
			display->displayText(grayImage, y, x, txt, color);
		}
	}
}

/**====================================================
* Function to display the coordinate of the clicked position (non blocking behaviour)
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::ShowClickedPosition()
{
	std::stringstream ss;
	if (isColor)
	{
		if (useHalfDisplay)
		{
			vpDisplay::getClick(colorImageHalf, clickPoint, button, false);
			vpDisplay::displayCross(colorImageHalf, clickPoint, 15, vpColor::darkGreen, 2);
			ss << "(" << clickPoint.get_u() << ";" << clickPoint.get_v() << ")";
			std::string s = ss.str();
			display->displayText(colorImageHalf, clickPoint.get_v(), clickPoint.get_u(), s, vpColor::red);
			//std::cout << clickPoint.get_u() << " " << clickPoint.get_v() << std::endl;
		}
		else
		{
			vpDisplay::getClick(colorImage, clickPoint, button, false);
			vpDisplay::displayCross(colorImage, clickPoint, 15, vpColor::darkGreen, 2);
			ss << "(" << clickPoint.get_u() << ";" << clickPoint.get_v() << ")";
			std::string s = ss.str();
			display->displayText(colorImage, clickPoint.get_v(), clickPoint.get_u(), s, vpColor::red);
			//std::cout << clickPoint.get_u() << " " << clickPoint.get_v() << std::endl;
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			vpDisplay::getClick(grayImageHalf, clickPoint, button, false);
			vpDisplay::displayCross(grayImageHalf, clickPoint, 15, vpColor::darkGreen, 2);
			ss << "(" << clickPoint.get_u() << ";" << clickPoint.get_v() << ")";
			std::string s = ss.str();
			display->displayText(grayImageHalf, clickPoint.get_v(), clickPoint.get_u(), s, vpColor::red);
			//std::cout << clickPoint.get_u() << " " << clickPoint.get_v() << std::endl;
		}
		else
		{
			vpDisplay::getClick(grayImage, clickPoint, button, false);
			vpDisplay::displayCross(grayImage, clickPoint, 15, vpColor::darkGreen, 2);
			ss << "(" << clickPoint.get_u() << ";" << clickPoint.get_v() << ")";
			std::string s = ss.str();
			display->displayText(grayImage, clickPoint.get_v(), clickPoint.get_u(), s, vpColor::red);
			//std::cout << clickPoint.get_u() << " " << clickPoint.get_v() << std::endl;
		}
	}
}

/**====================================================
* Function to display the coordinate of the clicked position (non blocking behaviour)
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::ShowClickedPositionBinary()
{
	std::stringstream ss;
	vpDisplay::getClick(binaryImage, binaryClickPoint, button, false);
	vpDisplay::displayCross(binaryImage, binaryClickPoint, 15, vpColor::darkGreen, 2);
	ss << "(" << binaryClickPoint.get_u() << ";" << binaryClickPoint.get_v() << ")";
	std::string s = ss.str();
	display->displayText(binaryImage, binaryClickPoint.get_v(), binaryClickPoint.get_u(), s, vpColor::red);
	//std::cout << binaryClickPoint.get_u() << " " << binaryClickPoint.get_v() << std::endl;
}



/**====================================================
* Function to start recording a video
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::StartRecordingVideo()
{
	writer = new vpVideoWriter();
	writer->setFramerate(recordingVideoFPS);
	//writer->setCodec(CV_FOURCC('H', '2', '6', '4')); // MPEG-1 codec
	writer->setCodec(cv::VideoWriter::fourcc('H', '2', '6', '4'));
	//writer->setCodec(CODEC_ID_MPEG1VIDEO);

	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	//char buffer[80];
	//strftime(buffer, 80, "%Y-%m-%d.", now);

	char opt_videoname[255];
	strftime(opt_videoname, sizeof(opt_videoname), "%Y_%m_%d_%H_%M_%S.mp4", now);
	//sprintf(opt_videoname, "video%04d.avi", num);
	//std::string opt_videoname = "video-recorded.avi";
	writer->setFileName(opt_videoname);

	if (isColor)
	{
		if (useHalfDisplay)
		{
			writer->open(colorImageHalf);
		}
		else
		{
			writer->open(colorImage);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			writer->open(grayImageHalf);
		}
		else
		{
			writer->open(grayImage);
		}
	}
	num++;
}

/**====================================================
* Function to start recording a video (binary version)
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::StartRecordingVideoBinary()
{
	binaryWriter = new vpVideoWriter();
	binaryWriter->setFramerate(25);
	//binaryWriter->setCodec(CV_FOURCC('H', '2', '6', '4')); // MPEG-1 codec
	binaryWriter->setCodec(cv::VideoWriter::fourcc('H', '2', '6', '4')); // MPEG-1 codec
	
	char opt_videoname[255];
	sprintf(opt_videoname, "binaryVideo%04d.avi", num);
	//std::string opt_videoname = "binaryVideo-recorded.avi";
	binaryWriter->setFileName(opt_videoname);
	binaryWriter->open(binaryImage);
	num++;
}

/**====================================================
* Function to add the last grabbed image to the video
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::AddFrameToVideo()
{
	if (isColor)
	{
		if (useHalfDisplay)
		{
			writer->saveFrame(colorImageHalf);
		}
		else
		{
			writer->saveFrame(colorImage);
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			writer->saveFrame(grayImageHalf);
		}
		else
		{
			writer->saveFrame(grayImage);
		}
	}
}

/**====================================================
* Function to add the last grabbed tresholded image to the video
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::AddFrameToVideoBinary()
{
	binaryWriter->saveFrame(binaryImage);
}

/**====================================================
* Function to stop the video recording
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::StopRecordingVideo()
{
	writer->close();
	delete writer;
	writer = NULL;
}

/**====================================================
* Function to stop the video recording
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::StopRecordingVideoBinary()
{
	binaryWriter->close();
	delete binaryWriter;
	binaryWriter = NULL;
}

/**====================================================
* Function to take a srceenshot (overlay are not saved!)
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::TakeAScreenShot()
{
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);

	if (isColor)
	{
		if (useHalfDisplay)
		{
			char filename[255];
			strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.jpeg", now);
			//char filename[255];
			//sprintf(filename, "image%04d.jpeg", num);
			vpImageIo::writeJPEG(colorImageHalf, filename);
			num++;
		}
		else
		{
			char filename[255];
			strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.jpeg", now);
			//char filename[255];
			//sprintf(filename, "image%04d.jpeg", num);
			vpImageIo::writeJPEG(colorImage, filename);
			num++;
		}
	}
	else
	{
		if (useHalfDisplay)
		{
			char filename[255];
			strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.jpeg", now);
			//char filename[255];
			//sprintf(filename, "image%04d.jpeg", num);
			vpImageIo::writeJPEG(grayImageHalf, filename);
			num++;
		}
		else
		{
			char filename[255];
			strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.jpeg", now);
			//char filename[255];
			//sprintf(filename, "image%04d.jpeg", num);
			vpImageIo::writeJPEG(grayImage, filename);
			num++;
		}
	}
}

/**====================================================
* Function to take a srceenshot of the tresholded image (overlay are not saved!)
* Input: NULL
* Output: NULL
*======================================================*/
void Vision::TakeAScreenShotBinary()
{
	time_t t = time(0);   // get time now
	struct tm * now = localtime(&t);
	char filename[255];
	strftime(filename, sizeof(filename), "%Y-%m-%d-%H-%M-%S.jpeg", now);
	//char filename[255];
	//sprintf(filename, "image%04d.jpeg", num);
	vpImageIo::writeJPEG(binaryImage, filename);
	num++;
}

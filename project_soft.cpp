// Student name: Hichem Rehouma
// Student ID: V00811045
// ELEC 586: Project 

#include <opencv2/core/core.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>


// This function performs the last step of edge detection , the Hysteresis thresholding 
// (A description is provided before its implementation in the Canny detection function)
void  hys_threshold(float upperDir, float lowerDir, cv::Mat mat1, cv::Mat mat2, cv::Mat mat3, int lowT, float p1,
		    float p2, float p3, float p4, float p5, float p6, float p7, float p8, bool is_chg)
			{
			      // Comparing surrounding pixel to the lower Threshold 
		              if (lowT<=mat1.at<float>(p1,p2)&&
			      // If the pixel hasn't already been checked 
			      mat2.at<unsigned char>(p1,p2)!=100 &&
			      // The gradient direction at this pixel is approximately the same  
			      mat3.at<float>(p1,p2) > lowerDir &&
			      mat3.at<float>(p1,p2) <= upperDir &&
			      // Check the grad magnitude of the next surrounding pixels, Similarly as in step 3
			      mat1.at<float>(p3,p4) > mat1.at<float>(p5,p6) &&
			      mat1.at<float>(p3,p4) > mat1.at<float>(p7,p8))
			      // If all conditions are satisfied, mark the neighbor as white(255), which would represent the color of an edge 
			      // and set the image as changed
		 	       { 
				   // Mark the detected edge (using a pointer) as a WHITE colored pixel
				   mat2.ptr<unsigned char> (p1,p2)[0] = 255;
				   is_chg = true;
			       }
}

///// This function is an implementation of a Canny Edge detector for a grayscale image matrix. The goal is 
//    to detect, locate then store each pixel in the image that is deemed to be an "edge" pixel. This 
//    function generates a "Mask"; that is, bright lines representing the edges on a black background.
//    This function uses variables, containers, and iterators from the OpenCV library. 
//
//    The image returned by this function is an essential input for many image processing detection 
//    algorithms such as object recognition and pattern analysis. 
//***************************************************************************************************
// Inputs: 
// - Mat source: this is the source image declared as a matrix of pixels (cv::MAT)
// - int upperThreshold: maximum threshold for pixel value entered by the user
// - int lowerThreshold: minimum threshold for pixel value entered by the user 
// - double size: Size of the Sobel Kernel Matrix 
// Outputs: 
// - returns an binary image matrix (with all the edges detected) 
//*************************************************************************************************** 
cv::Mat MyCannyDetector(cv::Mat sourceImg, int upperThreshold, int lowerThreshold, float size)
{ 
    // Construct a copy of the source image  
    cv::Mat Image1 = cv::Mat(sourceImg);
    Image1 = sourceImg.clone();

    ///////////////////////////////////////////////////////// 
    //Step 1: Noise reduction
    ////////////////////////////////////////////////////////

    // Apply Guassian Filter to smooth the image before Processing 
    cv::GaussianBlur(sourceImg, Image1, cv::Size(5, 5), 1.4);

    /////////////////////////////////////////////////////////
    // Step 2: Computing the Gradient Magnitudes and Directions
    //////////////////////////////////////////////////////////
	
    // The gradient is computed using the Sobel Operator via the
    // Sobel function from the OpenCV library. 
    // This function is a discrete differentiation operator. It computes 
    // an approximation of the gradient of an image intensity function and  
    // and outputs a differentiated matrix.  
      	
    // First, Compute the Gradient magnitude in the x-direction 
    cv::Mat mag_grad_X = cv::Mat(sourceImg.rows, sourceImg.cols, CV_32F);
    cv::Sobel(Image1, mag_grad_X, CV_32F, 1, 0, size);
//
   
    // Compute the Gradient magnitude in the y-direction
    cv::Mat mag_grad_Y = cv::Mat(sourceImg.rows, sourceImg.cols, CV_32F);
    cv::Sobel(Image1, mag_grad_Y, CV_32F, 0, 1, size);

    // At each pixel, compute the slope to find the Gradient direction  
    cv::Mat grad_dir = cv::Mat(Image1.rows, Image1.cols, CV_32F);
    cv::divide(mag_grad_Y, mag_grad_X, grad_dir);

    // Gradient Magnitude Matrix
    cv::Mat mag_grad = cv::Mat(Image1.rows, Image1.cols, CV_64F);

    cv::Mat x_prod = cv::Mat(Image1.rows, Image1.cols, CV_64F);
    cv::Mat y_prod = cv::Mat(Image1.rows, Image1.cols, CV_64F);

    cv::multiply(mag_grad_X, mag_grad_X, x_prod);
    cv::multiply(mag_grad_Y, mag_grad_Y, y_prod);

    mag_grad = x_prod + y_prod;
    cv::sqrt(mag_grad, mag_grad);


    // We will be working with 3 matrices 
    // - Matrix 1(mag_grad) will hold the gradient magnitude information 
    // - Matrix 2(grad_dir) will hold the gradient direction information
    // - Matrix 3(output) will be our output matrix 
    // Matrix 3 will be constructed from a search operation on Matrix 1 and 2

    ///////////////////////////////////////////////////////////
    // Step 3: Non-Maximum Suppression 
    //////////////////////////////////////////////////////////

    // This step will determine whether a point lies on an edge. An edge point 
    // will have a gradient magnitude greater than the upper threshold. 

    // Construct an image for the output of the Edge detector 
    cv::Mat edge_out = cv::Mat(sourceImg.rows, sourceImg.cols, CV_8U);

    // Initialize it to zero 
    edge_out.setTo(cv::Scalar(0));         

    ///// Initialize the iterators for the search operation   
    // Iterator for the direction of the gradient 
    cv::MatIterator_<float>direcIter = grad_dir.begin<float>();
    // Iterator for the output edge detected image 
    cv::MatIterator_<unsigned char>edgeOutIter = edge_out.begin<unsigned char>();

    // Iterate through each pixel of the image 
    for(cv::MatIterator_<float>mag_Iter = mag_grad.begin<float>();mag_Iter!=mag_grad.end<float>();++mag_Iter,++direcIter,
	++edgeOutIter)
    {
        // Calculate the current gradient direction 
        float current_dir = atan(*direcIter) * 180 / (float)M_PI;
	// Store the current pixel 
        const cv::Point current_pix = edgeOutIter.pos();
        // Eliminate negative directions and set the current direction  
	while (current_dir < 0) {
	   current_dir += 180;
	  *direcIter = current_dir;
	} ;
     
        // If the gradient magnitude is less than the upper threshold, 
	// ignore the pixel 
        if(*mag_Iter<upperThreshold)
	 continue;

        // Set the edge to be true, it will need to pass all the following conditions to 
        // remain as true 
	bool is_edge = true;

        // The approach is the following: if a pixel is not greater than its 8 surrounding pixels, then the pixel 
	// is not an edge. This process involves checking the direction of the gradient of the surrounding pixels, 
	// then compare the magnitude of these pixels to the current pixel. 

       // Check the first gradient direction 
       if(current_dir>112.5 && current_dir <=157.5)
        {
            // Bottom right pixel (x+1,y-1) (Additionally, check if pixel is not out of bound, which can cause 
            // the iterator to go past the container size or start before the first element and become invalidated)
 	    if(current_pix.y>0 && current_pix.x<Image1.cols &&
	        *mag_Iter<=mag_grad.at<float>(current_pix.y-1, current_pix.x+1)) {
	        is_edge = false;
	       }
		 
	   // Now, Top left pixel (x-1,y+1)
            if(current_pix.y<Image1.rows && current_pix.x>0 &&
		 *mag_Iter<=mag_grad.at<float>(current_pix.y+1, current_pix.x-1)) {	
		is_edge = false;
	       }	
        }
	// Next direction 
	else if(current_dir>67.5 && current_dir <= 112.5)
        {
	    // Straight below pixel (y-1,x)
            if(current_pix.y>0 && *mag_Iter<=mag_grad.at<float>(current_pix.y-1, current_pix.x)) {
		is_edge = false;
		}
	    // Straight above pixel (y+1,x)
            if(current_pix.y<Image1.rows && 
		*mag_Iter<=mag_grad.at<float>(current_pix.y+1, current_pix.x)) {
		is_edge = false;
		}
        }
	// Next direction
        else if(current_dir > 22.5 && current_dir <= 67.5)
        {
            // Bottom left pixel (x-1,y-x)
            if(current_pix.y>0 && current_pix.x>0 &&
		 *mag_Iter<=mag_grad.at<float>(current_pix.y-1, current_pix.x-1)) {
		 is_edge = false;
		}
	    // Top Right pixel (x-1,y-x)
            if(current_pix.y<Image1.rows && current_pix.x<Image1.cols &&
		 *mag_Iter<=mag_grad.at<float>(current_pix.y+1, current_pix.x+1)) {
		 is_edge = false;
		}
        }

        else
        {
	    // Left pixel (x-1,y)
            if(current_pix.x>0 && *mag_Iter<=mag_grad.at<float>(current_pix.y, current_pix.x-1)) {
		  is_edge = false;
		}
	    // Right pixel (x+1,y)
            if(current_pix.x<Image1.cols && *mag_Iter<=mag_grad.at<float>(current_pix.y, current_pix.x+1)) {
		is_edge = false;
		}
        }

	// If all of the above conditions are satisfied, the pixel is deemed to be an edge, we mark that pixel as white(255)
 	 if(is_edge)
        {
            *edgeOutIter = 255;
        }
	
      }


    ///////////////////////////////////////////////////////////
    // Step 4: Thresholding with Hysteresis
    //////////////////////////////////////////////////////////
   
   // This step is used to remove the weak edges as well as connect the splitted edges. To
   // connect the edges, we need to check each pixel's 8 surrounding pixels. If 
   // the neighbor's gradient magnitude is greater than low threshold, it will become an edge. 
 

   // In order to remove the weak edges and connect the splitted one, we need to check if the image 
   // has been changed in the last iteration. If not, no further processing is needed.
      bool is_img_changed = true;
     
      while(is_img_changed)
      {
        // First, set it to false. If a new edge pixel is found, it will be changed to true
	// and will allow for another iteration 
        is_img_changed = false;

	// Initialize the iterators again
        direcIter = grad_dir.begin<float>();
        edgeOutIter = edge_out.begin<unsigned char>();
       
        // Iterate through each pixel of the image 
        for(cv::MatIterator_<float>mag_Iter = mag_grad.begin<float>();mag_Iter!=mag_grad.end<float>();++mag_Iter,++direcIter,
	++edgeOutIter)
        {

 	    cv::Point current_pix = edgeOutIter.pos();
	    // Skip the pixels that are near the border of the image
	    // This will avoid considering the frame (if any) as edges
            if(current_pix.x<1 || current_pix.x>sourceImg.cols-1 || current_pix.y<1 || current_pix.y>sourceImg.rows-1)
                continue;


           // Change every newly deemed edge pixel's intensity to 100(arbitrary) to keep track of the 
	   // pixels that have already been checked 
            if(*edgeOutIter==255)
            {
                *edgeOutIter=(unsigned char)100;

          //  In addition to checking the 8 surrounding pixels gradient magnitude against the current one,
	  //  we will also check their magnitude against their surrounding pixels.
 
	  //  Check the first gradient direction 
	    if(*direcIter>112.5 && *direcIter <= 157.5)
                {
		    
                    if(current_pix.y>0 && current_pix.x>0)
                    {
			// Bottom left surrounding pixel (x-1,y-1)
			hys_threshold(157.5, 112.5, mag_grad, edge_out, grad_dir, lowerThreshold, 
				      current_pix.y-1, current_pix.x-1, current_pix.y-1,current_pix.x-1,
				      current_pix.y-2, current_pix.x,current_pix.y,current_pix.x-2,is_img_changed);

                    }

                    if(current_pix.y<Image1.rows && current_pix.x<Image1.cols)
                    {
			// Top right surrounding pixel (x+1,y+1)
			hys_threshold(157.5, 112.5, mag_grad, edge_out, grad_dir, lowerThreshold,
				      current_pix.y+1, current_pix.x+1, current_pix.y-1,current_pix.x-1,
				      current_pix.y+2,current_pix.x,current_pix.y,current_pix.x+2,is_img_changed);     
                    }
           
               }

 	    // Similar processing is carried out to the other gradient directions 
            else if(*direcIter>67.5 && *direcIter <= 112.5)
                {
                    if(current_pix.x>0)
                    {
		        // Left surrounding pixel (x-1,y)
			hys_threshold(112.5, 67.5, mag_grad, edge_out, grad_dir, lowerThreshold,
				      current_pix.y, current_pix.x-1, current_pix.y,current_pix.x-1,
			    	      current_pix.y-1,current_pix.x-1,current_pix.y+1,current_pix.x-1,is_img_changed);
                    }

                    if(current_pix.x<Image1.cols)
                    {
			// Right surrounding pixel (x+1,y)
			hys_threshold(112.5, 67.5, mag_grad, edge_out, grad_dir, lowerThreshold,
				      current_pix.y, current_pix.x+1, current_pix.y,current_pix.x+1,
				      current_pix.y-1,current_pix.x+1,current_pix.y+1,current_pix.x+1,is_img_changed);  
                    }
                }

                else if(*direcIter > 22.5 && *direcIter <= 67.5)
                {
                    if(current_pix.y>0 && current_pix.x<Image1.cols)
                    {
			// Bottom right surrounding pixel (x+1,y-1)
			hys_threshold(67.5, 22.5, mag_grad, edge_out, grad_dir, lowerThreshold, 
				      current_pix.y-1, current_pix.x+1, current_pix.y-1,current_pix.x+1,
				      current_pix.y-2,current_pix.x,current_pix.y,current_pix.x+2,is_img_changed);                 
                    }

                    if(current_pix.y<Image1.rows && current_pix.x>0)
                    {
			// Top left surrounding pixel (x-1,y+1)
			hys_threshold(67.5, 22.5, mag_grad, edge_out, grad_dir, lowerThreshold,
				      current_pix.y+1, current_pix.x-1, current_pix.y+1,current_pix.x-1,
				      current_pix.y,current_pix.x-2,current_pix.y+2,current_pix.x,is_img_changed);
                   
                    }
                }

                else
                {
                    if(current_pix.y>0)
                     {
			// Directly below surrounding pixel (x,y-1)
			hys_threshold(157.5, 22.5, mag_grad, edge_out, grad_dir, lowerThreshold, 
				      current_pix.y-1, current_pix.x, current_pix.y-1,current_pix.x,
				      current_pix.y-1,current_pix.x-1,current_pix.y-1,current_pix.x+2,is_img_changed);
                     }

                    if(current_pix.y<Image1.rows)
                    {
                        {
			// Directly above surrounding pixel (x,y+1)
      			hys_threshold(157.5, 22.5, mag_grad, edge_out, grad_dir, lowerThreshold,
				      current_pix.y+1, current_pix.x, current_pix.y+1,current_pix.x,
				      current_pix.y+1,current_pix.x-1,current_pix.y+1,current_pix.x+1,is_img_changed);
		
                        }
                    }

                }
            }
        }
 	
    }

// After step 4 is completed, we change all the pixels that were marked as 100 back to white(255)   
for(cv::MatIterator_<unsigned char>It = edge_out.begin<unsigned char>();It!=edge_out.end<unsigned char>();++It)
    {
        if(*It==100)
            *It = 255;
    }
    return edge_out;
}

// This function returns the cosine of an angle between three points 
// Here, the law of Cosines is applied according the following formula for a given triangle:
// cos(edge1,edge2) = (edge2^2 + edge3^2 - edge1^2)/(2*edge2*edge3))  
static double angle_cosine(cv::Point pt1, cv::Point pt2, cv::Point pt0)
{
	// Get the adjacent and opposite edges in triangle (pt1,pt2,pt0)
	// First edge x-coordinates (1st adjacent)
	double edge1_x = pt1.x - pt0.x;
	// First edge y-coordinates (1st opposite)
	double edge1_y = pt1.y - pt0.y;
	// Second edge x-coordinates (2nd adjacent)
	double edge2_x = pt2.x - pt0.x;
	// Second edge y-coordinates (2nd opposite)
	double edge2_y = pt2.y - pt0.y;

	// Compute cosine 
	return (edge1_x*edge2_x + edge1_y*edge2_y)/sqrt((std::pow(edge1_x,2) + std::pow(edge1_y,2))*(std::pow(edge2_x,2) + std::pow(edge2_y,2))+ 1e-10);
}

// Function for labeling the detected contours 
void labelShape(cv::Mat& im, const std::string label, std::vector<cv::Point>& contour)
{
	int fontface = cv::FONT_HERSHEY_SIMPLEX;
	// Text characteristics
	cv::Size text = cv::getTextSize(label, fontface, 0.4, 1, 0);
	// Find the bounding rectangle of the contour
	cv::Rect r = cv::boundingRect(contour);
	// Center point 
	cv::Point pt(r.x + ((r.width - text.width) / 2), r.y + ((r.height + text.height) / 2));
	// Draw a white rectangle as a background for the text label  
	cv::rectangle(im, pt + cv::Point(0, 0), pt + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
	// Text label 
	cv::putText(im, label, pt, fontface, 0.4, CV_RGB(0, 0, 0), 0.5, 8);
}

// Compute Euclidean distance between two points 
inline float euc_dist(cv::Point& p, cv::Point& q)
{
	cv::Point diff = p - q;
        return cv::sqrt(diff.x*diff.x + diff.y*diff.y);
}

/// Global variables
cv::Mat sourceImg;
cv::Mat src_gray;
cv::Mat outImg;
cv::Mat detected_edges;

const int edgeThresh = 1;
const char* window_name = "Threshold Slider";
const int max_lowThreshold = 100;
const int ratio = 3;

int lowThreshold;
int kernel_size = 3;


////////////////////////////////////////////
// Callback function for the slider trackbar
/////////////////////////////////////////// 
// This function will generate a horizontol control slider for thresholding.
// The user will drag the cursor to change the value of the lower threshold for 
// the canny detector function and visualize the effect of this change on the 
// output image of the Canny edge detector.

// ****NOTE: Conceptually, the effect of increasing the lower threshold should 
// reduce the ability of the edge detector to detect the edges. Here, the Hysteresis thresholding step
// is affected; that is, less edges will be detected due to the increase of the low threshold. This effect
// also has 
void CannyThreshold( int, void* )
{ 
  /// Reduce noise with a kernel 3x3
  cv::blur( src_gray, detected_edges, cv::Size(3,3) );
  /// Canny detector
  detected_edges = MyCannyDetector(sourceImg, lowThreshold, lowThreshold*ratio, kernel_size);

  /// Using Canny's output as a mask, we display our result
  outImg = cv::Scalar::all(0);

  sourceImg.copyTo( outImg, detected_edges);
  imshow(window_name, outImg );
}

int main(int argc, char** argv)
{
	//cv::Mat src = cv::imread("polygon.png");
	if (argc != 3)
	{
	  std::cout << " Wrong number of inputs. Please make sure you enter 2 arguments in the following order:\n 1- Original image filename(.format) \n 2- Text file name(.format)\n";
	  exit(1);
	}

	// Load the source image 
	cv::Size size(1100,700); // Define desired image size 
	sourceImg = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
	std::string textfileName(argv[2]);   
	

	
	
	if (!sourceImg.data)
	{
		std::cerr << "failure to load image!";
		exit(1);
	}


	/// Create a matrix of the same type and size as source (for outImg)
 	outImg.create(sourceImg.size(), sourceImg.type());
 	 /// Convert the image to grayscale
  	cv::cvtColor(sourceImg, src_gray, CV_BGR2GRAY );
	/// Create a window for the Trackbar
 	cv::namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	 /// Create a Trackbar for user to enter threshold 
  	cv::createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );
 	 /// Show the image
 	CannyThreshold(0, 0);
	


	// Text file 
	 std::ofstream myTextFile;
	 myTextFile.open(textfileName.c_str());	
	 
	// Convert input image to grayscale	
	cv::Mat grayscale;
	cv::cvtColor(sourceImg, grayscale, CV_BGR2GRAY);
	
	// Use Canny edge detector to detect corners in the image with gradient shading 
	cv::Mat canny_output;
	canny_output = MyCannyDetector(grayscale, 0, 200, 31);

	//////////////////////////////////////////
	// Detecting Contours in the image
	/////////////////////////////////////////

	// Container of vector of points for image contours
	std::vector<std::vector<cv::Point> > contours;

	// Find contours on a "clone" of the canny function ouput matrix
	cv::findContours(canny_output.clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

	std::vector<cv::Point> approx;

	cv::Mat outImg = sourceImg.clone();
	
	// Number of polygons
	int numTri = 0;
	int numRect = 0;
	int numPent = 0;
	int numHex = 0;
	int numHept = 0;

	/////////////////////////////////////////
	// Circle Detection 
	////////////////////////////////////////	
	cv::vector<cv::Vec3f> circles;	
	// Using the Hough Transform 
	/* Arguments:
	    src_gray: Input image (grayscale)
	    circles: A vector that stores sets of 3 values: x_{c}, y_{c}, r for each detected circle.
	    CV_HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV
	    dp = 1: The inverse ratio of resolution
	    min_dist = src_gray.rows/8: Minimum distance between detected centers
	    param_1 = 200: Upper threshold for the internal Canny edge detector
	    param_2 = 100*: Threshold for center detection.
	    min_radius = 0: Minimum radio to be detected. If unknown, put zero as default.
	    max_radius = 0: Maximum radius to be detected. If unknown, put zero as default*/

	cv::HoughCircles(canny_output, circles, CV_HOUGH_GRADIENT, 2, 100, 200, 100, 0, 70 );	
	
	for( size_t i = 0; i < circles.size(); i++ )
	  {
   		// Find center location and radius	
		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
   		int radius = cvRound(circles[i][2]);
		// Text characteristics
		int fontface = cv::FONT_HERSHEY_SIMPLEX;
		cv::Size text = cv::getTextSize("Circle", fontface, 0.4, 1, 0);	
		// Draw boundary
   		cv::circle( outImg, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
		// Draw white rectangle for label background 
		cv::rectangle(outImg, center + cv::Point(0, 0), center + cv::Point(text.width, -text.height), CV_RGB(255, 255, 255), CV_FILLED);
		// label circle 
		cv::putText(outImg, "Circle", center, cv::FONT_HERSHEY_SIMPLEX, 0.4, CV_RGB(0, 0, 0), 0.8, 8);
		// Center Coordinates
		myTextFile << "Coords of Circle  "<< i+1 << " center :"<< std::setw(7) << center << "\n"; 
		// Circumference 
		myTextFile << "Circum of circle "<< i+1 <<": "<< std::setw(21) << 2*M_PI*radius << "\n";
		// Area 
		myTextFile << "Area of circle "<< i+1 <<": "<< std::setw(23) << M_PI*radius*radius << "\n\n";

		
 	  }
	
	// Initialize points for same contour error check	
	cv::Point p1(0,0);
	cv::Point p2(0,0);
	cv::Point p3(0,0);
	
	for (int i = 0; i < contours.size(); i++)

		{
		// Approximate contour with accuracy proportional
		// to the contour perimeter
		double epsilon = cv::arcLength(cv::Mat(contours[i]), true)*0.02;

		// This function will approximate the number of vertices in a polygon
		cv::approxPolyDP(cv::Mat(contours[i]), approx, epsilon, true);


		// Skip very small objects 
		// For Non-Convex objects: SHOW CONVEXITY DEFECTS
		if (std::fabs(cv::contourArea(contours[i])) < 100)
		continue;

		// Identify Non-Convex Shapes 
		if(!cv::isContourConvex(approx)){
		labelShape(outImg, "NON-CONVEX", contours[i]);}
	
		// Number of vertices
		int vertices = approx.size();

		/////////////////////////////////////////
		// Triangle: total of 3 vertices detected
		////////////////////////////////////////
		if (vertices == 3)
		{

            	  numTri++; 
		  // Draw boundary around detected triangle
		  line(outImg, approx.at(0), approx.at(1), cvScalar(0,76,153),4);
                  line(outImg, approx.at(1), approx.at(2), cvScalar(0,76,153),4);
          	  line(outImg, approx.at(2), approx.at(0), cvScalar(0,76,153),4);

		  // Print the coordinates
		  myTextFile<<"Coords of Tri " << numTri <<": " << std::setw(18)<< approx.at(0) << " " << approx.at(1) 
		         << " " << approx.at(2) << "\n";
		  // Compute circumference
		  float circum_tri = euc_dist(approx.at(0),approx.at(1)) +
				     euc_dist(approx.at(1),approx.at(2)) +
				     euc_dist(approx.at(2),approx.at(0));  
		
		  // WRITE THIS IN TEXT FILE and USE SETW
		  myTextFile<< "Circum Triangle "<< numTri << ": " << std::setw(21)<< circum_tri << "\n";	
		    
		  labelShape(outImg, "TRGLE", contours[i]);
		
		  // Compute Area 
		  float p = 0.5*(circum_tri);
		  float a = euc_dist(approx.at(0),approx.at(1));
		  float b = euc_dist(approx.at(1),approx.at(2));
		  float c = euc_dist(approx.at(2),approx.at(0));
		  float area_tri = cv::sqrt(p*(p-a)*(p-b)*(p-c)); 
		  myTextFile<< "Area Triangle "<<numTri << ": " << std::setw(23)<< area_tri << "\n \n";
		}
	
		///////////////////////////////////////////
		// Rectangles, Pentagons, and Hexagons 
		//////////////////////////////////////////
		else if (vertices >= 4 && vertices <= 6 && cv::isContourConvex(approx)){
	
		// Get the cosines of all corners
		std::vector<double> cosine;
		
		 for (int j = 2; j < vertices + 1; j++){
   
		  // Store every cosine value by calling the angle function for 3 points
		  cosine.push_back(angle_cosine(approx[j%vertices], approx[j - 2], approx[j - 1]));

		  // Sort ascending the cosine values
		  std::sort(cosine.begin(), cosine.end());

		  // Get the lowest and the highest cosine 
		  double minAngleCos = cosine.front();
		  double maxAngleCos = cosine.back();

		// Use the angle cosines obtained above and the number of vertices
		// to determine the shape of the contour
				
		  //Rectangle
		  // An angle of a regular pentagon is 90 deg. Its cosine is 0. 
		  // We give the program some room for error and set the error margin as follow
		     if (vertices == 4 && minAngleCos >= -0.1 && maxAngleCos <= 0.1 
			&& cv::isContourConvex(approx)){
		        
		        numRect++;
			// Make sure the same contour is not detected twice 	
			cv::Point p0 = approx.at(0);
			if (p0==p1)
			{
			  numRect--; break;
			}

 			// Label rectangles 	
	  	       labelShape(outImg, "RECT", contours[i]);
		       // Draw boundary around detected rectangles
		       line(outImg, approx.at(0), approx.at(1), cvScalar(102,204,0),4);
                       line(outImg, approx.at(1), approx.at(2), cvScalar(102,204,0),4);
          	       line(outImg, approx.at(2), approx.at(3), cvScalar(102,204,0),4);
          	       line(outImg, approx.at(3), approx.at(0), cvScalar(102,204,0),4);
		        p1 = approx.at(0);
		
		       // Print the coordinates
		       myTextFile<<"Coords of Rect " << numRect <<": " << std::setw(17)<< approx.at(0) << " " << approx.at(1) 
		       << " " << approx.at(2) << " " << approx.at(3)<<"\n";
 		       // Compute circumference
		       float circum_rect = 2.0*(euc_dist(approx.at(0),approx.at(1)) +
		       euc_dist(approx.at(1),approx.at(2)));  
		       myTextFile<< "Circum Rect "<< numRect << ": "<< std::setw(25) << circum_rect << "\n";	
		       //Compute Area 
		       float area_rect = euc_dist(approx.at(0),approx.at(1))*
		       euc_dist(approx.at(1),approx.at(2));
		       myTextFile<< "Area Rect "<< numRect << ": " << std::setw(28) << area_rect << "\n \n";		  		     
		     }
		  	
		// Pentagon
		// An angle of a regular pentagon is 108 deg. Its cosine is -.301. 
		// We give the program some room for error and set the error margin as follow
		else if (vertices == 5 && minAngleCos >= -0.38 && maxAngleCos <= -0.22
			&& cv::isContourConvex(approx))
		{//AND IS NOT COVEX
			numPent++;
			// Make sure the same contour is not detected twice 	
			cv::Point p = approx.at(0);
			if (p==p2)
			{
			  numPent--; break;
			}
    		      // Draw boundary around detected Pentagons
		       line(outImg, approx.at(0), approx.at(1), cvScalar(32,32,32),4);
                       line(outImg, approx.at(1), approx.at(2), cvScalar(32,32,32),4);
          	       line(outImg, approx.at(2), approx.at(3), cvScalar(32,32,32),4);
          	       line(outImg, approx.at(3), approx.at(4), cvScalar(32,32,32),4);
          	       line(outImg, approx.at(4), approx.at(0), cvScalar(32,32,32),4);

  		       p2 = approx.at(0);
		      // Label 
		       labelShape(outImg, "PENTA", contours[i]);

 		      // Print the coordinates
		       myTextFile<<"Coords of Pent " << numPent <<": " << std::setw(16) << approx.at(0) << " " << approx.at(1) 
		       << " " << approx.at(2) << " " << approx.at(3)<< approx.at(4)<<"\n";

		       // Circumference
		       double circum_pent = cv::arcLength(cv::Mat(contours[i]),true);
		       myTextFile<< "Circum Pent "<< numPent << ": " << std::setw(25) << circum_pent << "\n";      
		       //Area
		       myTextFile<< "Area of Pentagon "<< numPent << ": " << std::setw(20) << contourArea(contours[i]) << "\n \n";
		}
		
		//Hexagon
		else if (vertices == 6 && minAngleCos >= -0.65 && maxAngleCos <= -0.38
			&& cv::isContourConvex(approx) ){
			numHex++;
			// Make sure the same contour is not detected twice 	
			cv::Point p01 = approx.at(0);
			if (p01==p3)
			{
			  numHex--; break;
			}
			// Draw boundary around detected Hexagons
		       line(outImg, approx.at(0), approx.at(1), cvScalar(0,102,204),4);
                       line(outImg, approx.at(1), approx.at(2), cvScalar(0,102,204),4);
          	       line(outImg, approx.at(2), approx.at(3), cvScalar(0,102,204),4);
          	       line(outImg, approx.at(3), approx.at(4), cvScalar(0,102,204),4);
          	       line(outImg, approx.at(4), approx.at(5), cvScalar(0,102,204),4);
          	       line(outImg, approx.at(5), approx.at(0), cvScalar(0,102,204),4);

  		       p3 = approx.at(0);
		       // label
		       labelShape(outImg, "HEXA", contours[i]);
  			// Print the coordinates
		       myTextFile <<"Coords of Hex " << numHex <<": " << std::setw(18) << approx.at(0) << " " << approx.at(1) 
		       << " " << approx.at(2) << " " << approx.at(3)<< approx.at(4)<< approx.at(5)<<"\n";
	       		// Circumference
		       double circum_hex = cv::arcLength(cv::Mat(contours[i]),true);
		       myTextFile << "Circum Hex "<< numHex << ": " << std::setw(25) << circum_hex << "\n";   
		       //Area
		       myTextFile<< "Area of Hex "<< numHex << ": " << std::setw(23) << contourArea(contours[i]) << "\n \n";
		}
	     }
	   }
	}
	// Close text file
	myTextFile.close();
	
	// Display original image
	cv::imshow("Original Image", sourceImg);
	
	// Display processed image, resized
	cv::Mat resized_outImg;
	cv::resize(outImg,resized_outImg,size);
	cv::imshow("Output Image", resized_outImg);
	
	cv::waitKey(0);  
	return 0;
}





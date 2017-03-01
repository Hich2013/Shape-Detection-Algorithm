# 2-D Geometric shape detection and recognition 

   Object recognition is one of the most important aspects of visual perception. Without an embedded computer vision implementation 
   of an object detection algorithm, robots will be unable to avoid obstacles and perform some of the most basic routine tasks. 

   This project implements a C++ algorithm for the detection of 2-d shapes in a given image and recognition of standard geometric shapes.
   Example of shapes to be recongized: triangles, rectangles, polygons, circles, and ellipses. 

## Features 

 * Detecting shapes 
   
   * Labeling the detected shapes 
   * Computing areas and circumferences in pixel units 
   * Checking for convexity defects
   * Drawing boundaries

   Essential steps included in the algorithm 

### Image conversion to greyscale 
  
  The program scan the entire image and examines the file for shapes to be detected 
  Prior to detection, the image will be converted from RBG to a grey-scale image. Greyscale representations are commonly used for extracting descriptors instead of processing color images directly for computational cost reasons.

### Edge and Shape Detection 

  An important step of shape recognition is to first detect all the edges that can form a shape. 
  A commonly used algorithm to perform this particular task is the Canny Edge Detector. This algorithm mimics the behavior of the Canny Edge detector by implementing the following steps: 

   * Gaussian Filter: Applying a Gaussian filter to filter out the noise in the image 
   * Gradient Computation: computing the gradient strength and direction of the image pixels to find possible angles 
   * Hyteresis thresholding: using two thresholds as follows: 
 	For each detected edge, high thresholded edges which are connected to high thresholded edges are retained 
 	Low thresholded edges which are non-connected to high-thresholded edges are removed 

### Polygon Recognition

   X and Y pixel coordinates of the detected edges that form the polygon are extracted.
   Angles between vertices are computed and stored. Angles are then classified by comparison to a predefined range. 
   The nature of the shape is then identified by counting the number of edges, checking angle ranges, and running a convexity check (using an OpenCV library method `isContourConvex`). 

## Installing 

    In order to use the program, the user will have to input the following input arguments:
		- The name of the input image file to be processed. The program will carry out all the processing on this
		image.
		- The name of the output image file. After processing the input image, the algorithm will store the output
		image in another image file.
		- The name of a new text file to output measurement information to. The program will store the
		measurements done on the input image file and store them in this text file. The file will be organized
		following a certain order and will contain the areas and circumferences of the detected shapes.
		 The user will be able to visualize the functionalities of the program in the output image
		and text file. 

		 For testing purposes, the user can run the program with different input image files. One advantage of
		the program is that the shapes that are present in the input image do not have to be perfectly drawn; in other
		words, the program is still be able to process shapes to a certain extent of distortion.
    
## Authors

- Hichem Rehouma

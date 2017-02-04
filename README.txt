// Student name: Hichem Rehouma
// Student ID: V00811045
// ELEC 586: Project 

In order to test the program, the correct number and order of arguments needs to be followed.
The input arguments are the following:

1- argv[1]: the name of the original/source image to be processed followed by its format. For instance,
	     if the image source's name is "test_image1" and its format is "png", the correct input will be 
	     test_image1.png.

2 - argv[2]: the name of the text file that the program will write information to followed by its format. For instance
	     if the desired text file's name is "outText" and its desired format is "txt", the correct input will be 
	     outText.txt

The following script in the command window is an example of how to run the program:
	     ./project test_image1.png outText.txt

After it is executed, the program will display three windows that correspond to the following,

- Window 1: named "Output Image". This window will display the image after being processed. 
- Window 2: named "Original Image". This window will display the original image (before processing).
- Window 3: named "Threshold Slider". This window will display the trackbar/slider that the user can 
            control to change the threshold value used in the edge detection algorithm. The image displayed in this window
	    is the output of the Canny edge detector function. The user should be able to visualize the effect on the ability 
	    to detect the edges. Some shapes can be completely not detected by changing the threshold value. 
	    The edges of each detected shape are visualized on a black background.   

I included 3 test images are provided in the zip file.

The image formats supported by the program are  

Windows bitmaps - *.bmp, *.dib 
PEG files - *.jpeg, *.jpg, *.jpe JPEG 2000 files - *.jp2 
Portable Network Graphics - *.png 
Portable image format - *.pbm, *.pgm, *.ppm 
Sun rasters - *.sr, *.ras 
TIFF files - *.tiff, *.tif 

The program should be able to detect the following regular shapes:
- Circles, triangles, rectangles, pentagons, and hexagons 
- And recognize the non convex shapes 



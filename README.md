# RT_SLAM

Real-time SLAM for Bebop 2

Tested in: Ubuntu 18.04


Download/install dependencies.  Set their path in Makefile

1.- Armadillo C++ library  (tested with version 9.900.2), http://arma.sourceforge.net/

2.- OpenCV (tested with version 4.5), https://opencv.org/

3.- Ceres Solver (tested with version 2.0)    http://ceres-solver.org/

4.- VISP   (tested with version  3.4.0)     https://visp.inria.fr/

5.- Eigen  (tested with 3.3.4)      https://eigen.tuxfamily.org/

Another dependencies (already included by source code) 

1.- anms (from paper: Efficient adaptive non-maximal suppression algorithms for homogeneous spatial keypoint distribution)



//---------------------------------------------------------------------------------------

For compiling:

1.- Configure Makefile for dependencies

2.- run:  
    - make

For fresh compiling:
    - make clean
    - make

//----------------------------------------------------------------------------------------

To set openplas to run in a single thread,
configure  the environment variable 
   
   -export OPENBLAS_NUM_THREADS=1

To set OPENCV to run in a single thread (Avoid OpenCL initialization )
configure  the environment variable, 
    
   -export OPENCV_OPENCL_DEVICE=disabled 

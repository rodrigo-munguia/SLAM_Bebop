# RT_BEBOP_SLAM

Real-time SLAM for Bebop 2

Tested in: Ubuntu 18.04


Download/install dependencies in the following order, and  set their path in the Makefile

1.- Armadillo C++ library  (tested with version 9.900.2), http://arma.sourceforge.net/

2.- Eigen  (tested with 3.3.4)      https://eigen.tuxfamily.org/

3.- Ceres Solver (tested with version 2.0)    http://ceres-solver.org/

4.- VTK package (tested with VTK 7.0: sudo apt-get install -y vtk7) Note: do not use VTK installation from source code   

5.- OpenCV with extra modules installed (opencv_contrib) (tested with OpenCV version 4.5), https://opencv.org/
    Particularly, SFM and VIZ modules are required (VIZ requires VTK package already installed).  

6.- VISP with Parrot SDK 3  (tested with VISP version  3.4.0)  see: https://visp-doc.inria.fr/doxygen/visp-daily/tutorial-bebop2-vs.html    



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

In tests, setting the following environment variables has shown to improve performance

To set openblas to run in a single thread,
configure  the environment variable 
   
   -export OPENBLAS_NUM_THREADS=1

To set OPENCV to run in a single thread (Avoid OpenCL initialization )
configure  the environment variable, 
    
   -export OPENCV_OPENCL_DEVICE=disabled 

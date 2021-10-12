########################################################################
####################### Makefile Template ##############################
########################################################################

OPT = -O2
# Armadillo
## As the Armadillo library uses recursive templates, compilation times depend on the level of optimisation:
##
## -O0: quick compilation, but the resulting program will be slow
## -O1: good trade-off between compilation time and execution speed
## -O2: produces programs which have almost all possible speedups, but compilation takes longer
## -O3: enables auto vectorisation when using gcc
ARMADILLO_LIB := -larmadillo
#-------------------------------------------------------------------------------------------
# Ceres 
CERES_SRC_DIR := /usr/local/include/ # This should point to place where you unpacked or cloned Ceres.
CERES_BIN_DIR := /usr/local/lib # This should point to the place where you built Ceres.
EIGEN_SRC_DIR := /usr/include/eigen3 # The place you unpacked or cloned Eigen.
CERES_INCLUDES := -I$(CERES_SRC_DIR) \
                  -I$(EIGEN_SRC_DIR)
CERES_LIBRARY := -lceres
CERES_LIBRARY_PATH := -L$(CERES_BIN_DIR)
CERES_LIBRARY_DEPENDENCIES = -lgflags -lglog
# If Ceres was built with Suitesparse:
CERES_LIBRARY_DEPENDENCIES +=  -llapack -lblas -lm -lcamd -lamd -lccolamd -lcolamd -lcholmod
# If Ceres was built with CXSparse:
CERES_LIBRARY_DEPENDENCIES += -lcxsparse
# If Ceres was built with OpenMP:
CERES_LIBRARY_DEPENDENCIES += -fopenmp -lpthread -lgomp -lm				  
#-------------------------------------------------------------------------------------------



#-------------------------------------------------------------------------------------------
# OpenCV
OPENCV_SRC_DIR := /usr/local/include/opencv4/
OPENCV = `pkg-config opencv --cflags --libs`
OPENCVLIBRARIES = $(OPENCV)
#-------------------------------------------------------------------------------------------
# Python  (for 2D ploting)
PYTHON_SRC_DIR := /usr/include/python3.6
PYTHON_BIN_DIR := /usr/lib/python3.6/config-3.6m-x86_64-linux-gnu
PYTHON_LIB := -lpython3.6  

# ARDroneSDK3

VISP_BUILD_DIR = /home/rodrigo/RESEARCH/visp-ws/visp-build
VISP_CFLAGS    = `$(VISP_BUILD_DIR)/bin/visp-config --cflags`
VISP_LDFLAGS   = `$(VISP_BUILD_DIR)/bin/visp-config --libs`
VISP_shared = -Wl,-rpath=/home/rodrigo/RESEARCH/visp-ws/visp-build/lib 
VISP_lib = /home/rodrigo/RESEARCH/visp-ws/visp-build/lib 

# Eigen
EIGEN_SRC_DIR := /usr/include/eigen3 # The place you unpacked or cloned Eigen.

#-------------------------------------------------------------------------------------------
# bebopcampure (this app)
LIB_DEPENDENCIES := -lpthread -lboost_system -lboost_thread
LIB_DEPENDENCIES_DIR := /usr/local/lib 
LIB_DIR := lib
LIB_INCLUDE := include
BIN		:= bin
OBJ     := obj
SRC		:= src
# Extra source code directories

CPPsrc = $(wildcard src/*.cpp) \
		 $(wildcard src/ekf/*.cpp) \
         $(wildcard src/Transforms/*.cpp) \
         $(wildcard src/Jacs/*.cpp) \
		 $(wildcard src/vision/*.cpp) \
		 $(wildcard src/map/*.cpp) \
		 $(wildcard src/anms/*.cpp)  \
		 $(wildcard src/loop/*.cpp)  \
		 $(wildcard src/control/*.cpp)
		 


		 

#-------------------------------------------------------------------------------------------
DEBUG = -ggdb

CXX		  := g++
CXX_FLAGS := -pthread -Wfatal-errors -g -Wall -Wextra -std=c++17 -fpermissive -static $(DEBUG) $(DEBUG_ARMA) $(OPT) $(EXTRA_OPT)
 #-fpermissive

INCLUDE	:= -I$(LIB_INCLUDE) -I$(OPENCV_SRC_DIR) -I$(EIGEN_SRC_DIR) $(CERES_INCLUDES) -I$(PYTHON_SRC_DIR)
INCLUDE2 := 
LIB		:= -L$(LIB_DIR) -L$(LIB_DEPENDENCIES_DIR) -L$(VISP_lib) $(CERES_LIBRARY_PATH) -L$(PYTHON_BIN_DIR)


LIBRARIES	:=  $(LIB_DEPENDENCIES) $(ARMADILLO_LIB) $(LIB_ARDrone_DEP) $(CERES_LIBRARY) $(CERES_LIBRARY_DEPENDENCIES) $(PYTHON_LIB)

EXECUTABLE	:= SLAMbebop



APPLICATION_OBJS = $(CPPsrc:.cpp=.o)


.PHONY: all

all: $(BIN)/$(EXECUTABLE) 



%.o: %.cpp 
		$(CXX)  -c -o  $@ $< $(CXX_FLAGS) $(INCLUDE) $(VISP_CFLAGS) 

$(BIN)/$(EXECUTABLE):$(APPLICATION_OBJS) 
		$(CXX)  -o $(BIN)/$(EXECUTABLE) $(APPLICATION_OBJS) $(VISP_shared) $(LIB) $(LIB_DEPENDENCIES)  $(OPENCVLIBRARIES) $(LIBRARIES) $(VISP_LDFLAGS)

$(shell   mkdir -p $(BIN))

run: clean all
	#clear
	./$(BIN)/$(EXECUTABLE)


clean: 
	-rm $(BIN)/*
	-rm $(APPLICATION_OBJS)





#SRCDIR := $(CPPsrc)
#SOURCES     := $(shell find $(SRCDIR) -type f -name *.$(SRCEXT))
#OBJECTS     := $(patsubst $(SRCDIR)/%,$(BUILDDIR)/%,$(SOURCES:.$(SRCEXT)=.$(OBJEXT)))


#$(OBJ)/%.o: %.cpp 
#		$(CXX) -c -o  $@ $< $(CXX_FLAGS) $(INCLUDE) 

#$(OBJ)/$(EXECUTABLE):$(OBJ)/*.o
#		$(CXX) $(APPLICATION_OBJS) $(LIB) $(OPENCVLIBRARIES) $(LIBRARIES) -o $(BIN)/$(EXECUTABLE)









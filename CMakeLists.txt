Project(OpenSimSynergyPlugin)
Project(OpenSimSynergyPlugin)

cmake_minimum_required(VERSION 3.2)

SET(UKIT PLUGIN)

FILE(GLOB SOURCE_FILES *.cpp)
FILE(GLOB INCLUDE_FILES *.h)

# C++11 is required
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(EXPORT_MACRO OSIM${UKIT}_EXPORTS)

SET(OPENSIM_INSTALL_DIR $ENV{OPENSIM_HOME} 
		CACHE PATH "Top-level directory of OpenSim install")
		
SET(PLUGIN_NAME "osimSynergyPlugin" CACHE STRING "Name of shared library to create")

# To add Debug feature add ";Debug" after Release on the line below
SET(CMAKE_CONFIGURATION_TYPES "RelWithDebInfo;Release" 
      CACHE STRING "Semicolon separated list of supported configuration types, only supports Debug, Release, MinSizeRel, and RelWithDebInfo, anything else will be ignored." FORCE )

# Location of headers
SET(SIMTK_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include/SimTK/include)
SET(OPENSIM_HEADERS_DIR ${OPENSIM_INSTALL_DIR}/sdk/include)
INCLUDE_DIRECTORIES(${SIMTK_HEADERS_DIR} ${OPENSIM_HEADERS_DIR})
# Libraries and dlls
SET(OPENSIM_LIBS_DIR ${OPENSIM_INSTALL_DIR}/sdk/lib ${OPENSIM_INSTALL_DIR}/lib ${OPENSIM_INSTALL_DIR}/sdk/SimBody/lib)
SET(OPENSIM_DLLS_DIR ${OPENSIM_INSTALL_DIR}/bin)
LINK_DIRECTORIES(${OPENSIM_LIBS_DIR} ${OPENSIM_DLLS_DIR})

# Namespace
SET(NameSpace "OpenSim_" CACHE STRING "Prefix for simtk lib names, includes trailing '_'. Leave empty to use stock SimTK libraries.")
MARK_AS_ADVANCED(NameSpace)

SET(SIMTK_COMMON_LIB debug SimTKcommon_d optimized SimTKcommon)
SET(SIMTK_MATH_LIB debug SimTKmath_d optimized SimTKmath)
SET(SIMTK_SIMBODY_LIB debug SimTKsimbody_d optimized SimTKsimbody)

SET(SIMTK_ALL_LIBS	${SIMTK_COMMON_LIB} 
                    ${SIMTK_MATH_LIB} 
                    ${SIMTK_SIMBODY_LIB} 
                   )

LINK_LIBRARIES(
	debug osimSimulation_d	optimized osimSimulation
	debug osimActuators_d	optimized osimActuators
	debug osimCommon_d		optimized osimCommon
	debug osimAnalyses_d	optimized osimAnalyses
	debug osimTools_d		optimized osimTools
	${SIMTK_ALL_LIBS}
)

SET(NameSpace "")
IF(APPLE)
  SET(PLATFORM_LIBS  SimTKAtlas)
  ELSE(APPLE)
    SET(PLATFORM_LIBS SimTKAtlas_Lin_generic)
ENDIF(APPLE)

ADD_LIBRARY(${PLUGIN_NAME} SHARED ${SOURCE_FILES} ${INCLUDE_FILES}) 

MARK_AS_ADVANCED(EXECUTABLE_OUTPUT_PATH)
MARK_AS_ADVANCED(LIBRARY_OUTPUT_PATH)

SET_TARGET_PROPERTIES(
	${PLUGIN_NAME} PROPERTIES
	DEFINE_SYMBOL ${EXPORT_MACRO}
	PROJECT_LABEL "Libraries - ${PLUGIN_NAME}")

INSTALL_TARGETS(/plugins RUNTIME_DIRECTORY /plugins  ${PLUGIN_NAME})


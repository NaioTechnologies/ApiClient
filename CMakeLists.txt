cmake_minimum_required(VERSION 3.0)

project(ApiClient)

enable_testing()

#---------------------------------------------------------------------------------------------------
#
#	Target name
#
include( ${PROJECT_SOURCE_DIR}/modules/Helpers.cmake )

#-------------------------------------------------------------------------------
#
#   Additional include directories
#
list_header_folders( HEADER_DIRECTORIES ${ApiClient_SOURCE_DIR}/ext/ApiCodec/src )
set( APICODEC_HEADERS ${HEADER_DIRECTORIES} )

#---------------------------------------------------------------------------------------------------
#
#   Module finding
#
set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/modules )

#---------------------------------------------------------------------------------------------------
#
#	Warning Options / C++ Language Options / Code Generation Options
#
set( CMAKE_C_FLAGS "-fsigned-char -fdiagnostics-color" )

set( CMAKE_CXX_FLAGS "-W -Wall -std=c++14 -Wextra -Winline -Wc++11-compat -Wconversion \
	-Wconversion-null -Wsign-conversion -Wctor-dtor-privacy -Wdelete-non-virtual-dtor -Wnarrowing \
	-Wnon-virtual-dtor -Wreorder -Wshadow -Woverloaded-virtual -Wno-pmf-conversions -Wsign-promo \
	-Wcast-align -Wmissing-declarations -Woverlength-strings -Wunreachable-code \
	-fconstexpr-depth=10 -fno-elide-constructors -ffor-scope -fno-gnu-keywords \
	-ftemplate-backtrace-limit=10 -fuse-cxa-atexit -fexceptions -fsigned-char -fdiagnostics-color" )

#---------------------------------------------------------------------------------------------------
#
#	Compilers specific options
#
if( USE_CLANG )
	set( CMAKE_C_COMPILER "/usr/bin/clang" )
	set( CMAKE_CXX_COMPILER "/usr/bin/clang++" )
	add_compile_options( -Wno-c++98-compat -Wno-c++11-compat )
elseif( CMAKE_CXX_COMPILER_ID STREQUAL "GNU" )
	if( CMAKE_CXX_COMPILER_VERSION VERSION_LESS "4.9" )
		message( FATAL_ERROR "Insufficient gcc version ; should be >= 4.9" )
	endif()
endif()


#---------------------------------------------------------------------------------------------------
#
# Set default cmake build type to Debug (None Debug Release RelWithDebInfo)
#
if( NOT CMAKE_BUILD_TYPE )
	set( CMAKE_BUILD_TYPE "Debug" )
endif()

#---------------------------------------------------------------------------------------------------
#
#   Add builds definitions
#
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
	add_definitions( -DDEBUG )
	add_compile_options( -g -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
			-ftree-loop-if-convert -funroll-loops -mfpmath=sse )

	if( WITH_SANITIZERS )
		add_compile_options( -fno-optimize-sibling-calls -fno-omit-frame-pointer
				-fsanitize=leak -fsanitize=address -fsanitize=undefined
				#[[-fsanitize=thread]] )
	endif( WITH_SANITIZERS )

elseif( CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" )
	add_definitions( -DDEBUG )
	add_compile_options( -O3 -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
			-ftree-loop-if-convert -funroll-loops -mfpmath=sse )
elseif( CMAKE_BUILD_TYPE STREQUAL "Release" )
	add_compile_options( -O3 -march=atom -msse -msse2 -msse3 -mssse3 -msse4.1 -msse4.2 -ffast-math
			-ftree-loop-if-convert -funroll-loops -mfpmath=sse )
endif()


#--------------------------------------------------------------------------------------------------
#
#   Project options
#
option( BUILD_SHARED_LIBS "Set to OFF to build static libraries" ON )
option( INSTALL_DOC "Set to OFF to skip build/install Documentation" OFF )

if( BUILD_SHARED_LIBS )
	set( CMAKE_FIND_LIBRARY_SUFFIXES ".so" )
else()
	set( CMAKE_FIND_LIBRARY_SUFFIXES ".a" )
endif()


#---------------------------------------------------------------------------------------------------
#
#   Source & Include directories
#
list_cpp_source_files( SOURCE_FILES src )


#---------------------------------------------------------------------------------------------------
#
#   Target specific definitions
#
add_definitions( -fPIC -DDECL_SPEC )

add_subdirectory( ext )

add_executable( ApiClient ${SOURCE_FILES} )

target_include_directories( ${PROJECT_NAME} SYSTEM PUBLIC
		${APICODEC_HEADERS}
	 )

#---------------------------------------------------------------------------------------------------
#
#   Target linking
#
if( CMAKE_BUILD_TYPE STREQUAL "Debug" )
	if( WITH_SANITIZERS )
		target_link_libraries( ${PROJECT_NAME} -lasan -lubsan )
	endif( WITH_SANITIZERS )
endif()

target_link_libraries(
		${PROJECT_NAME}
		libapicodec
		-lpthread
		-lSDL2main
        -lSDL2
        -lSDL2_ttf
        -lSDL2_gfx
		-lz
	)

#---------------------------------------------------------------------------------------------------
#
#   Library version
#
file( READ BuildVersion.hpp LIB_VERSION )

string( REGEX MATCH "define[ \t]+LIBRARY_MAJOR_VERSION[ \t]+([0-9]+)+"
		LIBRARY_MAJOR_VERSION_MATCH "${LIB_VERSION}" )
set( ${PROJECT_NAME}_MAJOR_VERSION "${CMAKE_MATCH_1}" )

string( REGEX MATCH "define[ \t]+LIBRARY_MINOR_VERSION[ \t]+([0-9]+)+"
		LIBRARY_MINOR_VERSION_MATCH "${LIB_VERSION}" )
set( ${PROJECT_NAME}_MINOR_VERSION "${CMAKE_MATCH_1}" )

string( REGEX MATCH "define[ \t]+LIBRARY_PATCH_VERSION[ \t]+([0-9]+)+"
		LIBRARY_PATCH_VERSION_MATCH "${LIB_VERSION}" )
set( ${PROJECT_NAME}_PATCH_VERSION "${CMAKE_MATCH_1}" )

set( ${PROJECT_NAME}_VERSION
		${${PROJECT_NAME}_MAJOR_VERSION}.${${PROJECT_NAME}_MINOR_VERSION}.${${PROJECT_NAME}_PATCH_VERSION} )

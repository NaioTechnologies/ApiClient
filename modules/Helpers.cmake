# Function to list all header files in the current directory, recursing into sub-directories
# HEADER_FILES - To be filled with the found header files
function( list_header_folders HEADER_DIRECTORIES PATH_TO_SCAN )
	file( GLOB_RECURSE HEADER_FILES_TMP "${PATH_TO_SCAN}/*.h" "${PATH_TO_SCAN}/*.hpp" "${PATH_TO_SCAN}/*.inl" "${PATH_TO_SCAN}/*.pch" )
	foreach( file_path ${HEADER_FILES_TMP} )
		get_filename_component( dir_path ${file_path} PATH )
		set( dir_list ${dir_list} ${dir_path} )
	endforeach()
	list( REMOVE_DUPLICATES dir_list )
	set( HEADER_DIRECTORIES ${dir_list} )
	set( HEADER_DIRECTORIES ${HEADER_DIRECTORIES} PARENT_SCOPE )
endfunction()

# Function to list all source files in the current directory, recursing into sub-directories
# SOURCE_FILES - To be filled with the found source files
function( list_cpp_source_files SOURCE_FILES PATH_TO_SCAN )
	file( GLOB_RECURSE SOURCE_FILES ${PATH_TO_SCAN}/*.cpp ${PATH_TO_SCAN}/*.c )
	set( SOURCE_FILES ${SOURCE_FILES} PARENT_SCOPE )
endfunction()

function( list_c_source_files SOURCE_FILES PATH_TO_SCAN )
	file( GLOB_RECURSE SOURCE_FILES ${PATH_TO_SCAN}/*.c )
	set( SOURCE_FILES ${SOURCE_FILES} PARENT_SCOPE )
endfunction()

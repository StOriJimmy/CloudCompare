cmake_minimum_required(VERSION 3.0)

include_directories( ${CC_CORE_LIB_SOURCE_DIR}/include )
include_directories( ${CC_FBO_LIB_SOURCE_DIR}/include )

project( QCC_DB_LIB )

file( GLOB header_list *.h)
file( GLOB source_list *.cpp)

set( CMAKE_CXX_VISIBILITY_PRESET hidden )

add_library( ${PROJECT_NAME} SHARED ${header_list} ${source_list} )
set_target_properties(${PROJECT_NAME} PROPERTIES FOLDER "libs") 
target_link_libraries( ${PROJECT_NAME} CC_CORE_LIB )
target_link_libraries( ${PROJECT_NAME} CC_FBO_LIB )

# Qt
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::OpenGL)

find_package( OpenGL REQUIRED )
if( NOT OPENGL_FOUND )
	message( SEND_ERROR "OpenGL required, but not found with 'find_package()'" )
endif()
include_directories( ${OpenGL_INCLUDE_DIR} )
target_link_libraries( ${PROJECT_NAME} ${OPENGL_LIBRARIES} )

# Add custom preprocessor definitions

if ( COMPILE_CC_CORE_LIB_SHARED )
	set_property( TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS CC_USE_AS_DLL )
endif()
target_compile_definitions( ${PROJECT_NAME} PRIVATE QCC_DB_LIBRARY_BUILD )


# install (shared) lib to specified destinations
if( WIN32 OR APPLE )
	foreach( dest ${INSTALL_DESTINATIONS} )
		install_shared( ${PROJECT_NAME} ${dest} 1 )
	endforeach()
else()
	install_shared( ${PROJECT_NAME} ${CMAKE_INSTALL_LIBDIR}/cloudcompare 0 ) #default destination: /usr/lib
endif()

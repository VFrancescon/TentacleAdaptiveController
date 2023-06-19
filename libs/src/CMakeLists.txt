find_package(Threads REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# STEP 1: Include cpp files

file(GLOB_RECURSE AC_VISION_SRC
        "visionClass.cpp")

# STEP 2: Include headers

file(GLOB_RECURSE AV_VISION_HEADERS
        "${CMAKE_SOURCE_DIR}/include/libs/include/AdaptiveControllerVisionClass/visionClass.hpp")

# STEP 3: Create Library
add_library(AC_Vision ${AC_VISION_SRC} ${AV_VISION_HEADERS})

# STEP 4: Link the include directories

target_include_directories(AC_Vision PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
                                                "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>")                    

target_link_libraries(AC_Vision Eigen3::Eigen ${OpenCV_LIBS} Threads::Threads)

#STEP 6: Install bin

# On Linux, "sudo make install" will typically copy the library
# into the folder /usr/local/bin
install(TARGETS AC_Vision DESTINATION lib)

#Step 7: Install local

# On Linux, "sudo make install" will typically copy the
# folder into /usr/local/include
# install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/libs/PSU_LIB DESTINATION include)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/AC_Vision DESTINATION include)
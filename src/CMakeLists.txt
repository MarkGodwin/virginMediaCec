project(vmCecControl)
cmake_minimum_required(VERSION 2.8.9)

set(vmCecControl_NAME cecclient)
set(vmCecControl_DESCRIPTION "libCEC test client")
set(vmCecControl_VERSION_MAJOR ${LIBCEC_VERSION_MAJOR})
set(vmCecControl_VERSION_MINOR ${LIBCEC_VERSION_MINOR})
set(vmCecControl_VERSION_PATCH ${LIBCEC_VERSION_PATCH})

enable_language(CXX)
include(CheckCXXSourceCompiles)
include(CheckLibraryExists)
include(CheckIncludeFiles)
include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-std=c++11" SUPPORTS_CXX11)
if (SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
endif()

find_package(p8-platform REQUIRED)
find_package(Threads REQUIRED)

set(vmCecControl_SOURCES virginMediaCec.cpp
                         helpers.cpp
                         VirginCommandQueue.cpp
                         VirginKeyCodes.cpp)

# curses
check_library_exists(curses initscr "" HAVE_CURSES_API)
if (HAVE_CURSES_API)
  list(APPEND vmCecControl_SOURCES curses/CursesControl.cpp)
endif()

add_executable(virginMediaCec ${vmCecControl_SOURCES})
set_target_properties(virginMediaCec PROPERTIES VERSION ${LIBCEC_VERSION_MAJOR}.${LIBCEC_VERSION_MINOR}.${LIBCEC_VERSION_PATCH})
target_link_libraries(virginMediaCec ${p8-platform_LIBRARIES})
target_link_libraries(virginMediaCec ${CMAKE_THREAD_LIBS_INIT})

if (NOT WIN32)
  # check for dlopen
  check_library_exists(dl dlopen "" HAVE_DLOPEN)
  if (HAVE_DLOPEN)
    target_link_libraries(virginMediaCec dl)
  endif()

  # curses
  if (HAVE_CURSES_API)
    target_link_libraries(virginMediaCec curses)
  endif()

  # rt
  check_library_exists(rt clock_gettime "" HAVE_RT)
  if (HAVE_RT)
    target_link_libraries(virginMediaCec rt)
  endif()

  # CoreVideo
  if (APPLE)
    target_link_libraries(virginMediaCec "-framework CoreVideo")
  endif()
else()
  add_definitions(-DTARGET_WINDOWS -DNOMINMAX -D_CRT_SECURE_NO_WARNINGS -D_WINSOCKAPI_)
  if (NOT ${WIN64})
    add_definitions(-D_USE_32BIT_TIME_T)
  endif()
endif()

include_directories(${p8-platform_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}
                    ${PROJECT_SOURCE_DIR}/../include)

# write env.h
CONFIGURE_FILE(${CMAKE_CURRENT_SOURCE_DIR}/env.h.in ${CMAKE_CURRENT_SOURCE_DIR}/env.h)

if (WIN32)
  install(TARGETS     virginMediaCec
          DESTINATION .)
else()
  install(TARGETS     virginMediaCec
          DESTINATION bin/.)
endif()

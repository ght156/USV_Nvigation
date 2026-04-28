cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

project(workspace_gz)

find_package(ament_cmake REQUIRED)

# --------------------------------------------------
# Gazebo Garden Dependencies
# --------------------------------------------------

find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})

find_package(gz-common5 REQUIRED COMPONENTS graphics)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})

find_package(gz-math7 REQUIRED)
set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

find_package(gz-transport12 REQUIRED)
set(GZ_TRANSPORT_VER ${gz-transport12_VERSION_MAJOR})

find_package(gz-plugin2 REQUIRED COMPONENTS loader)
set(GZ_PLUGIN_VER ${gz-plugin2_VERSION_MAJOR})

find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})

find_package(gz-sensors7 REQUIRED)
set(GZ_SENSORS_VER ${gz-sensors7_VERSION_MAJOR})

find_package(gz-utils2 REQUIRED)
set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

# --------------------------------------------------
# Library Dependencies
# --------------------------------------------------

find_package(sdformat13 REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(std_msgs REQUIRED)

find_package(gz-msgs9 REQUIRED)

find_package(gz-fuel_tools8 REQUIRED)

# --------------------------------------------------
# Build and Link Libraries
# --------------------------------------------------

add_library(Waves SHARED
  plugins/Wavefield.cc
)
target_link_libraries(Waves PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  Eigen3::Eigen
)

# --------------------------------------------------

add_library(PolyhedraBuoyancyDrag SHARED
  plugins/PolyhedraBuoyancyDrag.cc
  plugins/PolyhedronVolume.cc
  plugins/ShapeVolume.cc
)
target_link_libraries(PolyhedraBuoyancyDrag PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  Waves
)

# --------------------------------------------------

add_library(ScoringPlugin SHARED
  plugins/ScoringPlugin.cc
)
target_link_libraries(ScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
)

# --------------------------------------------------

add_library(StationkeepingScoringPlugin SHARED
  plugins/StationkeepingScoringPlugin.cc
  plugins/WaypointMarkers.cc
)
target_link_libraries(StationkeepingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ScoringPlugin
)

# --------------------------------------------------

add_library(WayfindingScoringPlugin SHARED
  plugins/WayfindingScoringPlugin.cc
  plugins/WaypointMarkers.cc
)
target_link_libraries(WayfindingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ScoringPlugin
)

# --------------------------------------------------

add_library(AcousticPerceptionScoringPlugin SHARED
  plugins/AcousticPerceptionScoringPlugin.cc
  plugins/WaypointMarkers.cc
)
target_link_libraries(AcousticPerceptionScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ScoringPlugin
)

# --------------------------------------------------

add_library(AcousticTrackingScoringPlugin SHARED
  plugins/AcousticTrackingScoringPlugin.cc
  plugins/WaypointMarkers.cc
)
target_link_libraries(AcousticTrackingScoringPlugin PUBLIC
  gz-common${GZ_COMMON_VER}::gz-common${GZ_COMMON_VER}
  gz-sim${GZ_SIM_VER}::core
  gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
  ScoringPlugin
)

# --------------------------------------------------

set(SOLAR_BOAT_WS_PLUGINS
  AcousticPingerPlugin
  BallShooterPlugin
  LightBuoyPlugin
  NavigationScoringPlugin
  GymkhanaScoringPlugin
  PerceptionScoringPlugin
  PlacardPlugin
  PublisherPlugin
  ScanDockScoringPlugin
  SimpleHydrodynamics
  Surface
  USVWind
  WaveVisual
  WildlifeScoringPlugin
)

foreach(PLUGIN IN LISTS SOLAR_BOAT_WS_PLUGINS)
  add_library(${PLUGIN} SHARED plugins/${PLUGIN}.cc)
  target_link_libraries(${PLUGIN} PUBLIC
    gz-sim${GZ_SIM_VER}::core
    gz-plugin${GZ_PLUGIN_VER}::gz-plugin${GZ_PLUGIN_VER}
    gz-rendering${GZ_RENDERING_VER}::gz-rendering${GZ_RENDERING_VER}
    gz-sensors${GZ_SENSORS_VER}::gz-sensors${GZ_SENSORS_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    ScoringPlugin
    Waves
    Eigen3::Eigen
  )
endforeach()

# --------------------------------------------------
# Installation Settings
# --------------------------------------------------

install(TARGETS
  Waves
  PolyhedraBuoyancyDrag
  ScoringPlugin
  StationkeepingScoringPlugin
  WayfindingScoringPlugin
  AcousticPerceptionScoringPlugin
  AcousticTrackingScoringPlugin
  ${SOLAR_BOAT_WS_PLUGINS}
  DESTINATION lib
)

install(DIRECTORY
  description
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# --------------------------------------------------
# Export Settings
# --------------------------------------------------

ament_export_libraries(
  Waves
  PolyhedraBuoyancyDrag
  ScoringPlugin
  StationkeepingScoringPlugin
  WayfindingScoringPlugin
  AcousticPerceptionScoringPlugin
  AcousticTrackingScoringPlugin
  ${SOLAR_BOAT_WS_PLUGINS}
)

ament_export_dependencies(
  ament_cmake
  gz-sim7
  gz-common5
  gz-math7
  gz-transport12
  gz-plugin2
  gz-rendering7
  gz-sensors7
  gz-utils2
  sdformat13
  Eigen3
  std_msgs
  gz-msgs9
  gz-fuel_tools8
)

ament_package()
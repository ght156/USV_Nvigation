# =============================================================================
# Sub-library: m_common::rtsp_interface
#
# 简化 RTSP 推流：仅 GStreamer（gst-rtsp-server），H.264 / H.265（含 Jetson nvv4l2* 等）。
# 编译期默认按 pkg-config 探测；可通过 -DRTSP_IF_ENABLE_GSTREAMER=ON 强制启用。
# =============================================================================

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(_RTSP_IF_GST_PROBE QUIET
  gstreamer-1.0>=1.16
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)
if(_RTSP_IF_GST_PROBE_FOUND)
  set(_RTSP_IF_GST_DEFAULT ON)
else()
  set(_RTSP_IF_GST_DEFAULT OFF)
  if(NOT RTSP_IF_ENABLE_GSTREAMER)
    message(WARNING
      "未找到 GStreamer RTSP 开发包。请安装：\n"
      "  sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev")
  endif()
endif()

option(RTSP_IF_ENABLE_GSTREAMER
       "Build m_common::rtsp_interface (GStreamer only)" ${_RTSP_IF_GST_DEFAULT})

if(NOT RTSP_IF_ENABLE_GSTREAMER)
  message(FATAL_ERROR
    "m_common::rtsp_interface 需要 GStreamer（-DRTSP_IF_ENABLE_GSTREAMER=ON）。"
    "请安装 gstreamer-rtsp-server 开发包。")
endif()

# ---- 源文件 ----
set(_RTSP_IF_SRCS
  src/rtsp_publisher.cpp
  src/rtsp_client.cpp
  src/rtsp_client_gst.cpp
  src/video_file_gst.cpp
  src/rtsp_remote_client_push.cpp
)
set(_RTSP_IF_DEFS "")
set(_RTSP_IF_PUBLIC_LINK "")
set(_RTSP_IF_PRIVATE_LINK "")
set(_RTSP_IF_PRIVATE_INCS "")

if(RTSP_IF_ENABLE_GSTREAMER)
  # 本机编译时勿传入空的 PKG_CONFIG_LIBDIR=/ PKG_CONFIG_SYSROOT_DIR=，否则会覆盖 pkg-config 默认搜索路径导致找不到 .pc。
  set(_rtsp_pc_env_vars "")
  if(DEFINED ENV{PKG_CONFIG_LIBDIR} AND NOT "$ENV{PKG_CONFIG_LIBDIR}" STREQUAL "")
    list(APPEND _rtsp_pc_env_vars "PKG_CONFIG_LIBDIR=$ENV{PKG_CONFIG_LIBDIR}")
  endif()
  if(DEFINED ENV{PKG_CONFIG_SYSROOT_DIR} AND NOT "$ENV{PKG_CONFIG_SYSROOT_DIR}" STREQUAL "")
    list(APPEND _rtsp_pc_env_vars "PKG_CONFIG_SYSROOT_DIR=$ENV{PKG_CONFIG_SYSROOT_DIR}")
  endif()

  # 强制启用时，先显式运行 pkg-config 打印错误详情（FindPkgConfig 的 REQUIRED 报错太简略）。
  # 注意：pkg-config 命令行不支持 "name>=ver" 这种写法（那是 pkg_check_modules 的语法）。
  foreach(_m IN ITEMS "gstreamer-1.0" "gstreamer-app-1.0" "gstreamer-rtsp-server-1.0")
    if(_rtsp_pc_env_vars)
      execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env ${_rtsp_pc_env_vars}
          pkg-config --print-errors --modversion "${_m}"
        RESULT_VARIABLE _pc_rc
        OUTPUT_VARIABLE _pc_out
        ERROR_VARIABLE  _pc_err
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    else()
      execute_process(
        COMMAND pkg-config --print-errors --modversion "${_m}"
        RESULT_VARIABLE _pc_rc
        OUTPUT_VARIABLE _pc_out
        ERROR_VARIABLE  _pc_err
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    endif()
    if(NOT _pc_rc EQUAL 0)
      message(FATAL_ERROR
        "RTSP_IF_ENABLE_GSTREAMER=ON 但 pkg-config 失败：${_m}\n"
        "PKG_CONFIG_LIBDIR=$ENV{PKG_CONFIG_LIBDIR}\n"
        "PKG_CONFIG_SYSROOT_DIR=$ENV{PKG_CONFIG_SYSROOT_DIR}\n"
        "modversion='${_pc_out}'\n"
        "stderr='${_pc_err}'\n"
      )
    endif()
  endforeach()

  # 额外检查：版本约束与链接参数（实际会触发 Requires.private 等依赖解析）
  if(_rtsp_pc_env_vars)
    execute_process(
      COMMAND "${CMAKE_COMMAND}" -E env ${_rtsp_pc_env_vars}
        pkg-config --print-errors --exists "gstreamer-1.0 >= 1.16"
      RESULT_VARIABLE _pc_ver_rc
      ERROR_VARIABLE  _pc_ver_err
      ERROR_STRIP_TRAILING_WHITESPACE
    )
  else()
    execute_process(
      COMMAND pkg-config --print-errors --exists "gstreamer-1.0 >= 1.16"
      RESULT_VARIABLE _pc_ver_rc
      ERROR_VARIABLE  _pc_ver_err
      ERROR_STRIP_TRAILING_WHITESPACE
    )
  endif()
  if(NOT _pc_ver_rc EQUAL 0)
    if(_rtsp_pc_env_vars)
      execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env ${_rtsp_pc_env_vars}
          pkg-config --debug --print-errors --exists "gstreamer-1.0 >= 1.16"
        RESULT_VARIABLE _pc_ver_dbg_rc
        ERROR_VARIABLE  _pc_ver_dbg_err
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    else()
      execute_process(
        COMMAND pkg-config --debug --print-errors --exists "gstreamer-1.0 >= 1.16"
        RESULT_VARIABLE _pc_ver_dbg_rc
        ERROR_VARIABLE  _pc_ver_dbg_err
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    endif()
    message(FATAL_ERROR
      "RTSP_IF_ENABLE_GSTREAMER=ON 但 gstreamer-1.0 版本不满足 >= 1.16（pkg-config --exists 失败）。\n"
      "PKG_CONFIG_LIBDIR=$ENV{PKG_CONFIG_LIBDIR}\n"
      "PKG_CONFIG_SYSROOT_DIR=$ENV{PKG_CONFIG_SYSROOT_DIR}\n"
      "stderr='${_pc_ver_err}'\n"
      "\n"
      "pkg-config --debug 输出（rc=${_pc_ver_dbg_rc}）：\n"
      "${_pc_ver_dbg_err}\n"
    )
  endif()

  foreach(_m IN ITEMS "gstreamer-1.0" "gstreamer-app-1.0" "gstreamer-rtsp-server-1.0")
    if(_rtsp_pc_env_vars)
      execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env ${_rtsp_pc_env_vars}
          pkg-config --print-errors --cflags --libs "${_m}"
        RESULT_VARIABLE _pc_cl_rc
        OUTPUT_VARIABLE _pc_cl_out
        ERROR_VARIABLE  _pc_cl_err
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    else()
      execute_process(
        COMMAND pkg-config --print-errors --cflags --libs "${_m}"
        RESULT_VARIABLE _pc_cl_rc
        OUTPUT_VARIABLE _pc_cl_out
        ERROR_VARIABLE  _pc_cl_err
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
      )
    endif()
    if(NOT _pc_cl_rc EQUAL 0)
      message(FATAL_ERROR
        "RTSP_IF_ENABLE_GSTREAMER=ON 但 pkg-config --cflags/--libs 失败：${_m}\n"
        "PKG_CONFIG_LIBDIR=$ENV{PKG_CONFIG_LIBDIR}\n"
        "PKG_CONFIG_SYSROOT_DIR=$ENV{PKG_CONFIG_SYSROOT_DIR}\n"
        "stdout='${_pc_cl_out}'\n"
        "stderr='${_pc_cl_err}'\n"
      )
    endif()
  endforeach()

  pkg_check_modules(RTSP_IF_GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0>=1.16
    gstreamer-app-1.0
    gstreamer-rtsp-server-1.0)
  list(APPEND _RTSP_IF_SRCS src/backend_gstreamer.cpp)
  list(APPEND _RTSP_IF_DEFS RTSP_IF_HAS_GSTREAMER=1)
  list(APPEND _RTSP_IF_PRIVATE_LINK PkgConfig::RTSP_IF_GST)
  list(APPEND _RTSP_IF_PUBLIC_LINK PkgConfig::RTSP_IF_GST)
  message(STATUS "m_common::rtsp_interface: GStreamer backend ENABLED")
endif()

add_library(rtsp_interface STATIC ${_RTSP_IF_SRCS})
set_target_properties(rtsp_interface PROPERTIES
  OUTPUT_NAME m_common_rtsp_interface
  POSITION_INDEPENDENT_CODE ON
)

target_include_directories(rtsp_interface PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_include_directories(rtsp_interface PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/src"
)
target_compile_features(rtsp_interface PUBLIC cxx_std_17)
target_compile_definitions(rtsp_interface PRIVATE ${_RTSP_IF_DEFS})

target_link_libraries(rtsp_interface
  PUBLIC
    ${OpenCV_LIBS}
    Threads::Threads
  PRIVATE
    m_common_mlogger_iface
    ${_RTSP_IF_PRIVATE_LINK}
)
target_include_directories(rtsp_interface PUBLIC ${OpenCV_INCLUDE_DIRS})

install(TARGETS rtsp_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/ DESTINATION include)

# 把启用的 backend 信息写到 install 目录，供 extras.cmake 读取
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/rtsp_interface_features.cmake"
  "set(_M_COMMON_RTSP_IF_HAS_GSTREAMER TRUE)\n"
)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/rtsp_interface_features.cmake"
  DESTINATION share/m_common/cmake
)

# 上抛给顶层
list(APPEND M_COMMON_EXPORT_LIBS m_common_rtsp_interface)
set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)

list(APPEND M_COMMON_EXPORT_DEPENDS OpenCV)
set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)

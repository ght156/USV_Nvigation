# m_common

ros2_perception 工作空间的公共/工具库聚合包。所有"被多个 ROS2 包都需要"的小型工具
（日志、字符串处理、几何/时间小工具等）统一放在这里，并以**命名空间 IMPORTED 目标**
的方式对外暴露：

```cmake
target_link_libraries(my_node PRIVATE
  m_common::mlogger     # header-only，spdlog 日志封装
  m_common::m_utils     # 编译型静态库，示例
)
```

下游只需要：

```xml
<!-- package.xml -->
<depend>m_common</depend>
```

```cmake
# CMakeLists.txt
find_package(m_common REQUIRED)
target_link_libraries(my_target PRIVATE m_common::xxx)
```

include 路径与传递依赖（spdlog 等）由 IMPORTED target 自动注入，**不需要**再写
`${m_common_INCLUDE_DIRS}` 或显式 link `spdlog`。

---

## 1. 目录结构

```
src/m_common/
├── CMakeLists.txt              # 顶层：仅做 add_subdirectory + 收口 export
├── package.xml                 # 包级 manifest，依赖在这里写
├── README.md                   # 本文档
├── cmake/
│   └── m_common-extras.cmake.in   # 集中描述所有 m_common::xxx IMPORTED 目标
├── mlogger/                    # 子库：header-only
│   ├── CMakeLists.txt
│   └── include/mlogger/mlogger.hpp
└── m_utils/                    # 子库：编译型 static lib
    ├── CMakeLists.txt
    ├── include/m_utils/*.hpp
    └── src/*.cpp
```

约定：

- 一个子库 = 一个**子目录** = 一份**自治的 `CMakeLists.txt`**
- 公共头放 `<sub>/include/<sub>/...`，使下游 `#include "<sub>/xxx.hpp"`
- 编译型库的 `OUTPUT_NAME` 加包前缀：`m_common_<sub>`，避免与全局名冲突
- 顶层 `CMakeLists.txt` **不写任何子库实现**，只 `add_subdirectory(<sub>)`
- 命名空间 IMPORTED 目标统一在 `cmake/m_common-extras.cmake.in` 集中描述

---

## 2. 顶层与子目录的接口约定

子目录通过两个 **PARENT_SCOPE 变量**把要 export 的内容上抛给顶层：

| 变量 | 含义 | 顶层用法 |
|---|---|---|
| `M_COMMON_EXPORT_DEPENDS` | 下游 `find_package(m_common)` 时也要拉的依赖（spdlog、Eigen3 等） | `ament_export_dependencies(...)` |
| `M_COMMON_EXPORT_LIBS`    | 下游链接的库名（编译型库才需要，header-only 不必） | `ament_export_libraries(...)` |

写法模板（在子目录 `CMakeLists.txt` 末尾）：

```cmake
list(APPEND M_COMMON_EXPORT_DEPENDS spdlog Eigen3)
set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)

list(APPEND M_COMMON_EXPORT_LIBS m_common_<sub>)
set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)
```

顶层 `CMakeLists.txt` 已经做好 `list(REMOVE_DUPLICATES)` + 一次性调用
`ament_export_*`，新增子库**不需要修改顶层 export 段**。

---

## 3. 新增子库的标准流程

新子库的命名约定（下文以 `<sub>` 代指，例如 `m_geom`、`m_time`）：

- 目录名 / 内部 target 名：`<sub>`
- 命名空间 IMPORTED 目标：`m_common::<sub>`
- 编译型库的 install 文件名：`libm_common_<sub>.a` （即 `OUTPUT_NAME m_common_<sub>`）
- 头文件 install 路径：`include/<sub>/...`

按子库类型分四个场景，照模板抄即可。

### 3.1 场景 A：header-only 库（参考 `mlogger`）

只有头文件、无 `.cpp`，依赖第三方 header（spdlog/Eigen 等）。

**步骤**：

1. 创建目录与头文件：
   ```
   src/m_common/<sub>/include/<sub>/<sub>.hpp
   ```

2. 写 `src/m_common/<sub>/CMakeLists.txt`：
   ```cmake
   find_package(<dep> REQUIRED)   # 例如 spdlog / Eigen3

   install(DIRECTORY include/ DESTINATION include)

   # 同包内其他子库若想在 build 期使用本库，可链接此桥目标
   add_library(m_common_<sub>_iface INTERFACE)
   target_include_directories(m_common_<sub>_iface INTERFACE
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
     "$<INSTALL_INTERFACE:include>"
   )
   target_compile_features(m_common_<sub>_iface INTERFACE cxx_std_17)
   target_link_libraries(m_common_<sub>_iface INTERFACE <dep>::<dep>)

   list(APPEND M_COMMON_EXPORT_DEPENDS <dep>)
   set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
   # header-only 没有库产物，不需要追加 M_COMMON_EXPORT_LIBS
   ```

3. 在 `cmake/m_common-extras.cmake.in` 追加 IMPORTED target：
   ```cmake
   # ---------- m_common::<sub> (header-only) ----------
   if(NOT TARGET m_common::<sub>)
     add_library(m_common::<sub> INTERFACE IMPORTED)
     set_target_properties(m_common::<sub> PROPERTIES
       INTERFACE_INCLUDE_DIRECTORIES "${_m_common_include_dir}"
       INTERFACE_COMPILE_FEATURES "cxx_std_17"
     )
     if(TARGET <dep>::<dep>)
       set_property(TARGET m_common::<sub> APPEND PROPERTY
         INTERFACE_LINK_LIBRARIES <dep>::<dep>)
     endif()
   endif()
   ```

4. 顶层 `src/m_common/CMakeLists.txt` 加一行：
   ```cmake
   add_subdirectory(<sub>)
   ```

5. `package.xml` 添加运行依赖（如未在其它子库中已经声明）：
   ```xml
   <depend><dep></depend>
   ```

### 3.2 场景 B：编译型静态库（参考 `m_utils`）

含 `.cpp` 实现，编译为 `libm_common_<sub>.a`。

**步骤**：

1. 创建目录与源码：
   ```
   src/m_common/<sub>/include/<sub>/*.hpp
   src/m_common/<sub>/src/*.cpp
   ```

2. 写 `src/m_common/<sub>/CMakeLists.txt`：
   ```cmake
   file(GLOB <SUB>_SRCS CONFIGURE_DEPENDS src/*.cpp)

   add_library(<sub> STATIC ${<SUB>_SRCS})
   set_target_properties(<sub> PROPERTIES OUTPUT_NAME m_common_<sub>)

   target_include_directories(<sub> PUBLIC
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
     "$<INSTALL_INTERFACE:include>"
   )
   target_compile_features(<sub> PUBLIC cxx_std_17)

   # 若实现里用了第三方库
   # find_package(<dep> REQUIRED)
   # target_link_libraries(<sub> PRIVATE <dep>::<dep>)

   install(TARGETS <sub>
     ARCHIVE DESTINATION lib
     LIBRARY DESTINATION lib
   )
   install(DIRECTORY include/ DESTINATION include)

   list(APPEND M_COMMON_EXPORT_LIBS m_common_<sub>)
   set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)

   # 若公共头里 #include 了第三方头，则也要上抛依赖：
   # list(APPEND M_COMMON_EXPORT_DEPENDS <dep>)
   # set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
   ```

3. 在 `cmake/m_common-extras.cmake.in` 追加 IMPORTED target：
   ```cmake
   # ---------- m_common::<sub> (compiled static lib) ----------
   if(NOT TARGET m_common::<sub>)
     add_library(m_common::<sub> STATIC IMPORTED)
     set_target_properties(m_common::<sub> PROPERTIES
       IMPORTED_LOCATION "${_m_common_lib_dir}/libm_common_<sub>.a"
       INTERFACE_INCLUDE_DIRECTORIES "${_m_common_include_dir}"
       INTERFACE_COMPILE_FEATURES "cxx_std_17"
     )
     # 若 .a 内含第三方未解析符号，必须把它挂到 INTERFACE_LINK_LIBRARIES
     # if(TARGET <dep>::<dep>)
     #   set_property(TARGET m_common::<sub> APPEND PROPERTY
     #     INTERFACE_LINK_LIBRARIES <dep>::<dep>)
     # endif()
   endif()
   ```

   > **重要**：静态库的符号在下游 link 阶段才解析，因此 `.a` 内部用到的所有外部库
   > 都必须通过 `INTERFACE_LINK_LIBRARIES` 显式声明，否则下游会报
   > `undefined reference to ...`。

4. 顶层加 `add_subdirectory(<sub>)`，必要时在 `package.xml` 加 `<depend>`。

### 3.3 场景 C：编译型动态库（SHARED）

含 `.cpp` 实现，编译为 `libm_common_<sub>.so`。

动态库与静态库的关键差异：

| 维度 | STATIC（`.a`） | SHARED（`.so`） |
|---|---|---|
| 符号解析 | 下游 link 阶段解析 | **本库链接阶段就解析**并写入 DT_NEEDED |
| 传递依赖声明 | IMPORTED target 必须 `INTERFACE_LINK_LIBRARIES` 挂齐 | IMPORTED target 只需挂**下游也要用到的**（public 头引用的） |
| PIC | 通常不强制 | **必须 PIC**（SHARED 目标 CMake 默认开，显式更稳） |
| 运行时 | 无 | 需要 `libm_common_<sub>.so` 在 `LD_LIBRARY_PATH` / RPATH；ament 环境已帮你处理 |
| ABI 版本 | 无 | 通常设置 `VERSION` / `SOVERSION` 以避免多版本冲突 |
| 符号可见性 | 不关心 | 建议 `CXX_VISIBILITY_PRESET hidden` + 导出宏（Windows 必需） |

**步骤**：

1. 创建目录与源码（与静态库一致）：
   ```
   src/m_common/<sub>/include/<sub>/*.hpp
   src/m_common/<sub>/src/*.cpp
   ```

2. 写 `src/m_common/<sub>/CMakeLists.txt`：
   ```cmake
   file(GLOB <SUB>_SRCS CONFIGURE_DEPENDS src/*.cpp)

   add_library(<sub> SHARED ${<SUB>_SRCS})
   set_target_properties(<sub> PROPERTIES
     OUTPUT_NAME              m_common_<sub>
     VERSION                  ${PROJECT_VERSION}   # 例：0.0.1
     SOVERSION                0                     # ABI 主版本
     POSITION_INDEPENDENT_CODE ON
     # 可选：隐藏默认符号可见性，仅导出带标记的符号（跨平台推荐）
     # CXX_VISIBILITY_PRESET    hidden
     # VISIBILITY_INLINES_HIDDEN ON
   )

   target_include_directories(<sub> PUBLIC
     "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
     "$<INSTALL_INTERFACE:include>"
   )
   target_compile_features(<sub> PUBLIC cxx_std_17)

   # 第三方依赖：SHARED 里用 PRIVATE 足矣，
   # 符号会在本库 link 阶段直接解析并写进 DT_NEEDED；
   # 若公共头 #include 了第三方头，则改为 PUBLIC。
   # find_package(<dep> REQUIRED)
   # target_link_libraries(<sub> PRIVATE <dep>::<dep>)

   install(TARGETS <sub>
     LIBRARY DESTINATION lib        # .so 走 LIBRARY
     ARCHIVE DESTINATION lib        # Windows 导入库 .lib 走 ARCHIVE
     RUNTIME DESTINATION bin        # Windows .dll 走 RUNTIME
   )
   install(DIRECTORY include/ DESTINATION include)

   list(APPEND M_COMMON_EXPORT_LIBS m_common_<sub>)
   set(M_COMMON_EXPORT_LIBS "${M_COMMON_EXPORT_LIBS}" PARENT_SCOPE)

   # 公共头里 #include 第三方头时才需要上抛：
   # list(APPEND M_COMMON_EXPORT_DEPENDS <dep>)
   # set(M_COMMON_EXPORT_DEPENDS "${M_COMMON_EXPORT_DEPENDS}" PARENT_SCOPE)
   ```

3. 在 `cmake/m_common-extras.cmake.in` 追加 IMPORTED target：
   ```cmake
   # ---------- m_common::<sub> (compiled shared lib) ----------
   if(NOT TARGET m_common::<sub>)
     add_library(m_common::<sub> SHARED IMPORTED)
     set_target_properties(m_common::<sub> PROPERTIES
       IMPORTED_LOCATION "${_m_common_lib_dir}/libm_common_<sub>.so"
       # 若设置了 SOVERSION，也可显式指向 soname：
       # IMPORTED_SONAME  "libm_common_<sub>.so.0"
       INTERFACE_INCLUDE_DIRECTORIES "${_m_common_include_dir}"
       INTERFACE_COMPILE_FEATURES "cxx_std_17"
     )
     # 仅当下游也要用到这个依赖（公共头里 #include）时才挂接，
     # 否则 SHARED 内部依赖不会污染下游 link 命令。
     # if(TARGET <dep>::<dep>)
     #   set_property(TARGET m_common::<sub> APPEND PROPERTY
     #     INTERFACE_LINK_LIBRARIES <dep>::<dep>)
     # endif()
   endif()
   ```

   > **对比 STATIC**：STATIC 里哪怕只有实现用到了 spdlog，IMPORTED target 也**必须**
   > 把 `spdlog::spdlog` 挂到 `INTERFACE_LINK_LIBRARIES`，否则下游 link 报
   > `undefined reference`。SHARED 则相反：只有**公共头**里也 `#include` 的依赖才需挂，
   > 实现细节的依赖已经在 `.so` 自己的 DT_NEEDED 里，运行时动态加载器会解决。

4. 顶层加 `add_subdirectory(<sub>)`，必要时在 `package.xml` 加 `<depend>`。

**可选：符号可见性控制（跨平台）**

如果你希望默认隐藏所有符号、只显式导出 API，可在公共头里加一个 visibility 头：

```cpp
// include/<sub>/visibility.h
#pragma once
#if defined(_WIN32)
  #ifdef M_COMMON_<SUB>_BUILDING
    #define M_COMMON_<SUB>_API __declspec(dllexport)
  #else
    #define M_COMMON_<SUB>_API __declspec(dllimport)
  #endif
#else
  #define M_COMMON_<SUB>_API __attribute__((visibility("default")))
#endif
```

并在 CMakeLists 里：

```cmake
target_compile_definitions(<sub> PRIVATE M_COMMON_<SUB>_BUILDING)
set_target_properties(<sub> PROPERTIES
  CXX_VISIBILITY_PRESET     hidden
  VISIBILITY_INLINES_HIDDEN ON)
```

然后在公共头里给要导出的函数/类加 `M_COMMON_<SUB>_API`。

> ament 也提供了 `ament_export_dependencies` 之外的可见性工具，复杂情况可用
> `ros2/rcutils`、`rclcpp_components` 里的 `visibility_control.h` 模板做参考。

### 3.4 场景 D：子库依赖同包内其他子库（参考 `m_utils` 用 `mlogger`）

例如 `<sub>` 的实现里要 `#include "mlogger/mlogger.hpp"`。**关键点**：build
阶段 `m_common::mlogger` 这个 IMPORTED 目标尚未创建（它由 install 后的
extras.cmake 生成），所以包内**不能**用命名空间目标互链，必须经由
**build-time INTERFACE 桥目标**。

桥目标已由各子库自己定义并暴露，命名约定为 `m_common_<sub>_iface`。

**步骤**（在 `<sub>/CMakeLists.txt` 里）：

```cmake
# 对方子目录由顶层 add_subdirectory() 引入后，桥目标即可见
target_link_libraries(<sub> PRIVATE m_common_mlogger_iface)
```

`PRIVATE` vs `PUBLIC` 的选择：

| 用法 | 选择 |
|---|---|
| 仅在 `.cpp` 实现里 `#include` 对方头 | `PRIVATE`（封装，下游不需要再 link mlogger） |
| 公共头 `*.hpp` 也 `#include` 对方头 | `PUBLIC`（必须把对方头/链接也传给下游） |

如果选了 `PRIVATE`，由于静态库符号晚解析，下游 link `m_common::<sub>` 时仍需要
mlogger 的传递依赖（spdlog 等）。处理方式是在 `m_common-extras.cmake.in` 里给
`m_common::<sub>` 把 `m_common::mlogger` 作为 `INTERFACE_LINK_LIBRARIES`：

```cmake
if(TARGET m_common::mlogger)
  set_property(TARGET m_common::<sub> APPEND PROPERTY
    INTERFACE_LINK_LIBRARIES m_common::mlogger)
endif()
```

> **顺序要求**：在 `m_common-extras.cmake.in` 中，被依赖方（`m_common::mlogger`）
> 必须先于依赖方（`m_common::<sub>`）声明，因为后者 `if(TARGET ...)` 检测的是
> 前者是否已存在。

---

## 4. 检查清单（PR 前自查）

- [ ] 子目录有自己的 `CMakeLists.txt`，顶层只多了一行 `add_subdirectory(<sub>)`
- [ ] 公共头位于 `<sub>/include/<sub>/...`，下游 `#include "<sub>/xxx.hpp"` 自然成立
- [ ] 编译型库 `OUTPUT_NAME = m_common_<sub>`，install 后是 `libm_common_<sub>.a`
- [ ] `m_common-extras.cmake.in` 里有 `m_common::<sub>` 段落，且 IMPORTED target
      的 `INTERFACE_LINK_LIBRARIES` 涵盖了所有传递依赖
- [ ] `package.xml` 中第三方依赖已声明（`<depend>spdlog</depend>` 等）
- [ ] 同包互依赖通过 `m_common_<sub>_iface` 桥目标，而非 `m_common::<sub>`
- [ ] 用一个独立的小 CMake 工程（或 `find_package(m_common)` 的下游包）实测：
      只 `target_link_libraries(... PRIVATE m_common::<sub>)` 即可编译并链接通过

---

## 5. 编译命令

仅构建本包：

```bash
make pkgd-r PKGS="m_common"
```

清建（修改 CMake 结构后推荐）：

```bash
rm -rf build/x86/m_common install/x86/m_common
make pkgd-r PKGS="m_common"
```

> 完整命令请参考 `.cursor/rules/ros2-make-build.mdc`。

---

## 6. 当前已有子库

| 子库 | 类型 | 命名空间目标 | 说明 |
|---|---|---|---|
| `mlogger`         | header-only | `m_common::mlogger`         | spdlog 日志封装；提供 `MLOGGER_INFO/WARN/ERROR` 等宏 |
| `m_utils`         | static lib  | `m_common::m_utils`         | 示例骨架：`split` / `trim` / `log_demo`；演示同包内调用 `m_common::mlogger` |
| `rtsp_interface`  | static lib  | `m_common::rtsp_interface`  | RTSP 推流：`start` / `push(cv::Mat)` / 暂停；**仅 GStreamer** |

---

## 7. `m_common::rtsp_interface` 快速使用

简化封装，**任何 ROS2 / 纯 C++ 包**只要能拿到 `cv::Mat`，三行就能推成 RTSP 流。
**仅 GStreamer**（`gst-rtsp-server`），H.264 / H.265（含 Jetson 硬件编码插件）。

CMake：默认按 pkg-config 探测 GStreamer；需安装 `libgstrtspserver-1.0-dev`。
```bash
make pkgd-r PKGS="m_common"
```

### 7.1 下游依赖声明

```xml
<!-- package.xml -->
<depend>m_common</depend>
<depend>libopencv-dev</depend>
```

```cmake
# CMakeLists.txt
find_package(m_common REQUIRED)
find_package(OpenCV REQUIRED)

target_link_libraries(my_node PRIVATE
  m_common::rtsp_interface
)
```

OpenCV / GStreamer / pthread 由 `m_common::rtsp_interface` 的
`INTERFACE_LINK_LIBRARIES` 自动注入，不需要下游再挂。

### 7.2 代码用法（最小示例）

```cpp
#include <m_common/rtsp_interface/rtsp_publisher.hpp>
#include <opencv2/opencv.hpp>

int main() {
  m_common::RtspPublisher pub;

  m_common::RtspPublisherConfig cfg;
  cfg.port = 8554;
  cfg.backend = m_common::RtspBackend::kAuto;   // auto / kGStreamer
  // GStreamer-only（auto 或 kGStreamer 时生效）：
  // cfg.h264_encoder = "auto";   // auto | x264 | nvv4l2 | jetson | hw

  m_common::RtspStreamSpec s;
  s.mount_path   = "cam_live";    // URL: rtsp://host:8554/live/cam_live
  s.fps          = 15;
  s.bitrate_kbps = 4000;
  s.codec        = m_common::RtspCodec::kH264;  // kH264 / kH265（MJPEG 不支持）
  cfg.streams.push_back(s);

  pub.start(cfg);                 // 失败抛 std::runtime_error
  std::printf("backend=%s, URL=%s\n", pub.backend_name().c_str(), pub.url(0).c_str());

  cv::VideoCapture cap(0);
  cv::Mat frame;
  while (true) {
    cap >> frame;
    if (frame.empty()) break;
    pub.push(0, frame);           // 非阻塞；leaky 队列只保留最新一帧
  }
  // 析构自动 stop
  return 0;
}
```

### 7.3 多路 + 暂停

```cpp
cfg.streams.push_back({.mount_path = "cam_a", .fps = 15, .bitrate_kbps = 4000});
cfg.streams.push_back({.mount_path = "cam_b", .fps = 25, .bitrate_kbps = 6000,
                       .codec = m_common::RtspCodec::kH265});
pub.start(cfg);

// 第 0 路持续 push；第 1 路暂停
pub.set_stream_paused("cam_b", true);

// 全局暂停/恢复（恢复时 H.264 路自动发 IDR，客户端立即可解）
pub.set_global_paused(true);
pub.set_global_paused(false);
```

### 7.4 拉流验证

```bash
ffplay rtsp://127.0.0.1:8554/live/cam_live
```

### 7.5 注意事项

- **URL**：`rtsp://host:port/<app>/<stream>`；mount_path 不含 `/` 时默认 `app="live"`。
- **同一进程同一端口仅一个 `RtspPublisher` 实例**。
- **运行期**：`apt install gstreamer1.0-plugins-base -good -ugly` 等。
- **`pub.push()` 线程安全**：可从多个线程对不同 `stream_idx` 同时调用。
- **首帧锁定尺寸**：后续不同尺寸的 `push` 会自动 `cv::resize` 到 locked 尺寸。
- **`pub.backend_name()`**：恒为 `"gstreamer"`。

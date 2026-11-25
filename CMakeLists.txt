cmake_minimum_required(VERSION 3.16)
project(Practical_04_MavlinkGroundStation LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Core Network)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Core Network)
# Модель и сетевой слой: без виджетов, общие для НСУ, эмулятора и тестов.
add_library(groundstation_network
    mavlinkmessageparser.h mavlinkmessageparser.cpp telemetrymodel.h telemetrymodel.cpp
    mavlinkmanager.h mavlinkmanager.cpp uavsimulator.h uavsimulator.cpp point3d.h)
target_include_directories(groundstation_network PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(groundstation_network PUBLIC Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Network)
if(MSVC)
    target_compile_options(groundstation_network PUBLIC /utf-8)
endif()

option(GROUNDSTATION_BUILD_GUI "Build the ground station and emulator windows" ON)
if(GROUNDSTATION_BUILD_GUI)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS
        Widgets Charts 3DCore 3DRender 3DExtras 3DInput Multimedia)
    add_library(groundstation_gui STATIC
        mainwindow.h mainwindow.cpp mainwindow.ui scene3d.h scene3d.cpp
        dronevisual.h dronevisual.cpp flightaudio.h flightaudio.cpp)
    target_link_libraries(groundstation_gui PUBLIC groundstation_network
        Qt${QT_VERSION_MAJOR}::Widgets Qt${QT_VERSION_MAJOR}::Charts
        Qt${QT_VERSION_MAJOR}::3DCore Qt${QT_VERSION_MAJOR}::3DRender
        Qt${QT_VERSION_MAJOR}::3DExtras Qt${QT_VERSION_MAJOR}::3DInput
        Qt${QT_VERSION_MAJOR}::Multimedia)
    add_executable(Practical_04_MavlinkGroundStation main.cpp assets.qrc)
    target_link_libraries(Practical_04_MavlinkGroundStation PRIVATE groundstation_gui)
    # Отдельное приложение «Эмулятор БПЛА» для проверки обмена между двумя процессами.
    add_executable(Practical_04_UavEmulator uavemulator.cpp)
    target_link_libraries(Practical_04_UavEmulator PRIVATE groundstation_network Qt${QT_VERSION_MAJOR}::Widgets)
endif()

include(CTest)
if(BUILD_TESTING)
    find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Test)
    add_executable(groundstation_tests tests.cpp)
    target_link_libraries(groundstation_tests PRIVATE groundstation_network Qt${QT_VERSION_MAJOR}::Test)
    add_test(NAME groundstation_network COMMAND groundstation_tests)
    set_tests_properties(groundstation_network PROPERTIES TIMEOUT 90)
    if(GROUNDSTATION_BUILD_GUI)
        add_executable(groundstation_window_tests window_tests.cpp assets.qrc)
        target_link_libraries(groundstation_window_tests PRIVATE groundstation_gui Qt${QT_VERSION_MAJOR}::Test)
        add_test(NAME groundstation_window COMMAND groundstation_window_tests)
        set_tests_properties(groundstation_window PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen" TIMEOUT 120)
    endif()
endif()

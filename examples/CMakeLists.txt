add_executable(ws_m1
ws_m1.cpp
)

target_link_libraries(ws_m1
WorkSpaceAnalysis
)

add_executable(ws_m1_load
ws_m1_load.cpp
)

target_link_libraries(ws_m1_load
WorkSpaceAnalysis
)

add_executable(visualize
visualize.cpp
)

target_link_libraries(visualize
WorkSpaceAnalysis
)

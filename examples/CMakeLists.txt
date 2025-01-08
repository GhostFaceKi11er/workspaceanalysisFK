add_executable(main
main.cpp
)

target_link_libraries(main
WorkSpaceAnalysis
)

add_executable(testload_m1_ws
testload_m1_ws.cpp
)

target_link_libraries(testload_m1_ws
WorkSpaceAnalysis
)

add_executable(visualize
visualize.cpp
)

target_link_libraries(visualize
WorkSpaceAnalysis
)

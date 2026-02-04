# Declare the executable
add_executable(pleyo_fps main.cpp)

# Use the target-based linking (same as fastpointcloud example)
# This automatically handles include paths for the Orbbec wrapper
target_link_libraries(pleyo_fps PRIVATE 
    k4a::k4a)

# Ensure the .exe is placed in the global bin folder
set_target_properties(pleyo_fps PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
)
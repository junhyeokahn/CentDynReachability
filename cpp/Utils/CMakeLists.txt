FILE(GLOB_RECURSE headers *.hpp)
FILE(GLOB_RECURSE sources *.cpp)
add_library (dartUtils SHARED ${headers} ${sources})

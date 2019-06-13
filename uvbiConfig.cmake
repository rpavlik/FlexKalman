set(_uvbi_old_mod_path ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR} ${CMAKE_MODULE_PATH})
include(CMakeFindDependencyMacro)
find_dependency(JsonCpp)
include("${CMAKE_CURRENT_LIST_DIR}/uvbiTargets.cmake")

set(CMAKE_MODULE_PATH ${_uvbi_old_mod_path})

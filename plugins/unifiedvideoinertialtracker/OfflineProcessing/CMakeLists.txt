if(Boost_FOUND)
    add_executable(uvbi-offline-processing
        $<TARGET_OBJECTS:uvbi-hdkdata>
        OfflineProcessing.cpp
        CSVCellGroup.h
        QuatToEuler.h
        ../MakeHDKTrackingSystem.h
        ${KALMANFRAMEWORK_VIDEOTRACKERSHARED_SOURCES_IO})

    set_target_properties(uvbi-offline-processing PROPERTIES
        FOLDER "${PROJ_FOLDER}")
    target_link_libraries(uvbi-offline-processing
        PRIVATE
        uvbi-core
        JsonCpp::JsonCpp)
    target_include_directories(uvbi-offline-processing PRIVATE ${BOOST_INCLUDE_DIRS})
endif()

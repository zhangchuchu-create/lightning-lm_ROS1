add_executable(run_loc_online
        run_loc_online.cc
)

target_link_libraries(run_loc_online
        ${PROJECT_NAME}
        ${third_party_libs}
)


install(TARGETS  run_loc_online
        DESTINATION lib/${PROJECT_NAME})

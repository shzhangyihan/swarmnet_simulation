add_library(util lib_loader.cpp motion_log.cpp config_parser.cpp)

target_link_libraries(util dl JsonCpp config)


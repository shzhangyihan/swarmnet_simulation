add_library(kilo_comm_event rx_event.cpp tx_event.cpp)
target_link_libraries(kilo_comm_event event arena)
add_library(IROS_2021_fixed_default SHARED default.cpp)
target_link_libraries(IROS_2021_fixed_default kilobot)
add_library(IROS_2021_fixed_bloom_64_12 SHARED bloom_64_12.cpp)
target_link_libraries(IROS_2021_fixed_bloom_64_12 kilobot)
add_library(IROS_2021_fixed_store_all SHARED store_all.cpp)
target_link_libraries(IROS_2021_fixed_store_all kilobot)

add_library(IROS_2021_fixed_bloom_128_12 SHARED bloom_128_12.cpp)
target_link_libraries(IROS_2021_fixed_bloom_128_12 kilobot)
add_library(IROS_2021_fixed_bloom_64_24 SHARED bloom_64_24.cpp)
target_link_libraries(IROS_2021_fixed_bloom_64_24 kilobot)


# add_library(IROS_2021_fixed_bloom_32_4 SHARED bloom_32_4.cpp)
# target_link_libraries(IROS_2021_fixed_bloom_32_4 kilobot)
# add_library(IROS_2021_fixed_bloom_64_2 SHARED bloom_64_2.cpp)
# target_link_libraries(IROS_2021_fixed_bloom_64_2 kilobot)

# add_library(IROS_2021_fixed_bloom_32_8 SHARED bloom_32_8.cpp)
# target_link_libraries(IROS_2021_fixed_bloom_32_8 kilobot)
# add_library(IROS_2021_fixed_bloom_64_4 SHARED bloom_64_4.cpp)
# target_link_libraries(IROS_2021_fixed_bloom_64_4 kilobot)
# add_library(IROS_2021_fixed_bloom_64_8 SHARED bloom_64_8.cpp)
# target_link_libraries(IROS_2021_fixed_bloom_64_8 kilobot)
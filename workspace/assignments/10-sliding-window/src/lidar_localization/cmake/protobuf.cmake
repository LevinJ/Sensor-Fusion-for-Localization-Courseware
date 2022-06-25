find_package (Protobuf REQUIRED)

include_directories(${Protobuf_INCLUDE_DIRS})
list(APPEND ALL_TARGET_LIBRARIES /usr/local/lib/libprotobuf.so.25;-lpthread)

message(STATUS "Protobuf_LIBRARIES: ${Protobuf_LIBRARIES}")

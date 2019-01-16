#project(MapTools)
#
#set(MESSAGE_HEADER Common.h)
#MESSAGE(STATUS "Message Headers : ${MESSAGE_HEADER}")
#
##add_executable(LaneMatch LaneMatch.cpp LaneMessage.h ${MESSAGE_HEADER} BlockMessage.h)
##target_link_libraries(LaneMatch MapLib)
#
#add_executable(MapMatch MapMatch.cpp BlockMessage.h ${MESSAGE_HEADER})
#target_link_libraries(MapMatch MapLib)
#
#add_executable(MapApp MapApp.cpp BlockMessage.h ${MESSAGE_HEADER})
#target_link_libraries(MapApp MapLib)
#
#add_executable(TrajectoryApp TrajectoryApp.cpp ${MESSAGE_HEADEAR})
#target_link_libraries(TrajectoryApp MapLib)


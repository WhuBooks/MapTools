project(MapTools)

add_executable(ModelTest ModelTest.cpp)
target_link_libraries(ModelTest MapLib)

add_executable(ReceiveMapTest ReceiveMapTest.cpp)
target_link_libraries(ReceiveMapTest MapLib)

add_executable(ClothoidTest ClothoidTest.cpp)
target_link_libraries(ClothoidTest MapLib)

#add_executable(SmoothTest SmoothTest.cpp)
#target_link_libraries(SmoothTest MapLib)

add_executable(HT_ConvertGPSData HT_ConvertGPSData.cpp)
target_link_libraries(HT_ConvertGPSData MapLib)

add_executable(RandomPickRoadPoint RandomPickRoadPoint.cpp)
target_link_libraries(RandomPickRoadPoint MapLib)

add_executable(CheckMapCoordinate CheckMapCoordinate.cpp)
target_link_libraries(CheckMapCoordinate MapLib)

add_executable(BuildNewConn BuildNewConn.cpp)
target_link_libraries(BuildNewConn MapLib)

add_executable(BuildnewRoad BuildnewRoad.cpp)
target_link_libraries(BuildnewRoad MapLib)

add_executable(CleanMapIDs CleanMapIDs.cpp)
target_link_libraries(CleanMapIDs MapLib)

add_executable(InitConnLaneIds InitConnLaneIds.cpp)
target_link_libraries(InitConnLaneIds MapLib)


add_executable(MoveLeftTest MoveLeftTest.cpp)
target_link_libraries(MoveLeftTest MapLib)

add_executable(RecordGPSPts RecordGPSPts.cpp)
target_link_libraries(RecordGPSPts MapLib)

add_executable(SparseTest SparseTest.cpp)
target_link_libraries(SparseTest MapLib)


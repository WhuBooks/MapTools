project(MapTools)

add_library(MapLib STATIC
        Log.h Log.cpp
        Geometry.h Geometry.cpp
        Coordinate.h Coordinate.cpp
        Grid.h Grid.cpp
        FileIO.h FileIO.cpp
        RoadModel.h RoadModel.cpp
        Graph.cpp Graph.h
        Cluster.h Cluster.cpp
        DouglasPeucker.h DouglasPeucker.cpp
        Clothoid.h Clothoid.cpp)

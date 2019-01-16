
add_library(MapLib
        Geometry/Geometry.h
        Geometry/Point2D.cpp Geometry/Point2D.h
        Util/AngleUtil.h
        Util/Coordinate.h Util/Coordinate.cpp
        RoadModel/GnssPoint2D.h
        RoadModel/Lane.cpp RoadModel/Lane.h
#        RoadModel/Road.cpp RoadModel/Road.h
#        RoadModel/Link.cpp RoadModel/Link.h
        Graph/Graph.h
        Graph/Floyd.cpp Graph/Floyd.h
        Curve/Curve.h
        Curve/DouglasPeucker.hpp Curve/Clothoid.hpp
        RoadModel/Block.cpp RoadModel/Block.h
        Cluster/GridCluster.h Cluster/GridCluster.hpp
#        Util/Log.cpp Util/Log.h
        Util/AngleUtil.cpp Geometry/Geometry.cpp Graph/Dijkstra.cpp Graph/Dijkstra.h Graph/AStar.cpp Graph/AStar.h)
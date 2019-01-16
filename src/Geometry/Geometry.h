//
// Created by books on 8/29/18.
//

#ifndef MAPTOOLS_GEOMETRY_H
#define MAPTOOLS_GEOMETRY_H

#include <iostream>
#include <vector>

namespace maplib
{
    namespace geometry
    {
        class GeometryBase
        {
        public:
            GeometryBase()
            {
                m_geo_id = GeoId++;
            }

        protected:
            long long m_geo_id;
            static long long GeoId;
        };

    }
}

#endif //MAPTOOLS_GEOMETRY_H

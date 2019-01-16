//
// Created by books on 2017/7/25.
//

#ifndef MAPTOOLS_GRID_H
#define MAPTOOLS_GRID_H

#include <cmath>
#include <numeric>
#include <vector>
#include <algorithm>
#include "Geometry.h"
#include "RoadModel.h"
#include <cstdint>

namespace MapBase
{
    class Grid
    {
    public:
        Grid()= default;

        ~Grid()= default;

        Grid(int _row, int _col, double _cellwidth, double _cellheight, const std::vector<int8_t> &_data);

        int Rows() { return row; };

        int Cols() { return col; };

        void InitLBWorldPos(const PointXYA &_cen)
        {
            double localx = -col * cellwidth / 2.0;
            double localy = -row * cellheight / 2.0;
            PointXYA lblocal = PointXYA(localx, localy, _cen.yaw);
            lbworld = lblocal.ToWorld(_cen);
        };

        void SetLBWorldPos(const PointXYA &pt)
        {
            lbworld = PointXYA(pt.x, pt.y, pt.yaw);
        };

        PointXYA WorldPos(int _row, int _col) const;

        PointXYA LocalPos(int _row, int _col) const;

        //overwrite the operator()
        int8_t operator()(int _row, int _col) const
        {
            if (_row > row || _col > col || _row < 0 || _col < 0)
            {
                return 0;
            }
            int pos = _row * col + _col;
            return data[pos];
        };

        Grid Clip(int startrow, int startcol, int endrow, int endcol);

        Grid Clip(const PointXYA &center, double radius);

    private:
        int row, col;
        double cellwidth, cellheight;
        std::vector<std::int8_t> data;
        //left-bottom world coordinate
        PointXYA lbworld;

    };

}



#endif //MAPTOOLS_GRID_H

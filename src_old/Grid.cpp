//
// Created by books on 2017/7/25.
//

#include "Grid.h"

namespace MapBase
{

    Grid::Grid(int _row, int _col, double _cellwidth, double _cellheight, const std::vector<int8_t> &_data)
    {
        row = _row;
        col = _col;
        cellwidth = _cellwidth;
        cellheight = _cellheight;
        data.resize((unsigned long) (row * col));
        data.assign(_data.begin(), _data.end());
    }

    PointXYA Grid::WorldPos(int _row, int _col) const
    {
        if (_row > row || _col > col || _row < 0 || _col < 0)
        {
            return PointXYA();
        }
        double dxlocal = _col * cellwidth + cellwidth * 0.5;
        double dylocal = _row * cellheight + cellheight * 0.5;

        PointXYA tmplocal = PointXYA(dxlocal, dylocal, lbworld.yaw);
        PointXYA tmpworld = tmplocal.ToWorld(lbworld);
        return tmpworld;

    }

    PointXYA Grid::LocalPos(int _row, int _col) const
    {
        if (_row > row || _col > col || _row < 0 || _col < 0)
        {
            return PointXYA();
        }
        double dxlocal = _col * cellwidth + cellwidth * 0.5;
        double dylocal = _row * cellheight + cellheight * 0.5;

        return PointXYA(dxlocal, dylocal);
    }

    Grid Grid::Clip(int startrow, int startcol, int endrow, int endcol)
    {
        if (startrow < 0 || startcol < 0 || endrow > row || endcol > col || endrow <= startrow || endcol <= startcol)
            return Grid();

        startrow = startrow < 0 ? 0 : startrow;
        startcol = startcol < 0 ? 0 : startcol;
        endrow = endrow > row ? row : endrow;
        endcol = endcol > col ? col : endcol;

        int cliprow = endrow - startrow;
        int clipcol = endcol - startcol;
        std::vector<int8_t> clipdata;

        for (int i = startrow; i < endrow; i++)
        {
            for (int j = startcol; j < endcol; j++)
            {
                clipdata.push_back(data[i * col + j]);
            }
        }
        Grid clipgrid(cliprow, clipcol, cellwidth, cellheight, clipdata);
        PointXYA cliplbworld = WorldPos(startrow, startcol);
        clipgrid.SetLBWorldPos(cliplbworld);

        return clipgrid;
    }

    Grid Grid::Clip(const PointXYA &centerworld, double radius)
    {
        PointXYA centerlocal = centerworld.ToLocal(lbworld);

        double minx = centerlocal.x - radius;
        double maxx = centerlocal.x + radius;
        double miny = centerlocal.y - radius;
        double maxy = centerlocal.y + radius;

        if (minx < 0 || maxx < 0 || miny < 0 || maxy < 0)
            return Grid();

        if (minx > col * cellwidth || miny > row * cellheight)
            return Grid();

        int srow = static_cast<int>(std::floor(miny / cellheight));
        int erow = static_cast<int>(std::ceil(maxy / cellheight));
        int scol = static_cast<int>(std::floor(minx / cellwidth));
        int ecol = static_cast<int>(std::ceil(maxx / cellwidth));
        return Clip(srow, scol, erow, ecol);
    }


}
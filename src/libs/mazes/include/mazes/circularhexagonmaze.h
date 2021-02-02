#ifndef CIRCULARHEXAGONMAZE_H
#define CIRCULARHEXAGONMAZE_H

#include "hexagonalmaze.h"

class CircularHexagonMaze : public HexagonalMaze {
public:
    explicit CircularHexagonMaze(int);

protected:
    std::shared_ptr<CellBorder> GetEdge(int, int, int, int) const override;
    std::tuple<double, double, double, double> GetCoordinateBounds() const override;
};

#endif /* end of include guard: CIRCULARHEXAGONMAZE_H */

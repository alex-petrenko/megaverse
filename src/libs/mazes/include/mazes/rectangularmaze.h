#include "maze.h"

class RectangularMaze : public Maze {
public:
    RectangularMaze(int, int);
    void InitialiseGraph() override;

private:
    int width_, height_;

    int VertexIndex(int, int);
    std::tuple<double, double, double, double> GetCoordinateBounds() const override;
};

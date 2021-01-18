#ifndef HEXAGONALMAZE_H
#define HEXAGONALMAZE_H

#include "maze.h"

class HexagonalMaze : public Maze {
 public:
  explicit HexagonalMaze(int);
  void InitialiseGraph() override;

 protected:
  int size_;

  virtual std::shared_ptr<CellBorder> GetEdge(int, int, int, int) const;
  int VertexIndex(int, int, int, int) const;
  std::tuple<double, double, double, double> GetCoordinateBounds() const override;
};

#endif /* end of include guard: HEXAGONALMAZE_H */

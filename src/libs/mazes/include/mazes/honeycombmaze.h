#ifndef HONEYCOMBMAZE_H
#define HONEYCOMBMAZE_H

#include "maze.h"

class HoneyCombMaze : public Maze {
 public:
  HoneyCombMaze(int);
  void InitialiseGraph();

  virtual std::tuple<double, double, double, double> GetCoordinateBounds() const;

public:
    bool bordersForEntranceAndExit = true;

protected:
    int size_;
    static const int neigh[6][2];

    [[nodiscard]] int VertexIndex(int, int) const;
    [[nodiscard]] virtual std::tuple<double, double, double, double> GetEdge(int, int, int) const;

    [[nodiscard]] static std::pair<double, double> GetCenter(int u, int v) ;

    std::pair<int, int> VExtent(int);

    bool IsValidNode(int, int);
};

#endif /* end of include guard: HONEYCOMBMAZE_H */

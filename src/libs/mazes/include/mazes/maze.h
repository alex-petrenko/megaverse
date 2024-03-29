#ifndef MAZE_H
#define MAZE_H

#include "cellborder.h"
#include "spanningtreealgorithm.h"
#include <memory>
#include <vector>


using AdjList = std::vector<std::vector<std::pair<int, std::shared_ptr<CellBorder>>>>;

class Maze {
 public:
  explicit Maze(int = 0, int = 0, int = 1);
  virtual ~Maze() = default;
  void GenerateMaze(SpanningtreeAlgorithm*);
  void PrintMazeGnuplot(const std::string&) const;
  void PrintMazeSVG(const std::string&) const;
  void RemoveBorders(const std::vector<std::pair<int, int>>&);
  virtual void InitialiseGraph() = 0;

  AdjList & getAdjacencyList() { return adjacencylist_; }
  std::vector<std::pair<double, double>> & getCellCenters() { return cellCenters; }

  virtual std::tuple<double, double, double, double> GetCoordinateBounds() const = 0;

protected:
    // Solving a maze is equivalent to finding a path in a graph
    int vertices_;
    AdjList adjacencylist_;
    int startvertex_, endvertex_;
    std::vector<std::pair<double, double>> cellCenters;
};

#endif /* end of include guard: MAZE_H */

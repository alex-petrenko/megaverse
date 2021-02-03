#ifndef CELLBORDER_H
#define CELLBORDER_H

#include <tuple>
#include <string>


class CellBorder {
 public:
  virtual ~CellBorder() = default;
  virtual std::string GnuplotPrintString() const = 0;
  virtual std::string SVGPrintString() const = 0;
};

class LineBorder : public CellBorder {
public:
    std::string GnuplotPrintString() const override;
    std::string SVGPrintString() const override;
    LineBorder(double, double, double, double);
    explicit LineBorder(std::tuple<double, double, double, double>);

    [[nodiscard]] std::tuple<double, double, double, double> getBorderCoords() const
    {
        return std::make_tuple(x1_, y1_, x2_, y2_);
    }

protected:
  double x1_, y1_, x2_, y2_;
};

class ArcBorder : public CellBorder {
 public:
  std::string GnuplotPrintString() const override;
  std::string SVGPrintString() const override;
  ArcBorder(double, double, double, double, double);

 protected:
  double cx_, cy_, r_, theta1_, theta2_;
};

#endif /* end of include guard: CELLBORDER_H */

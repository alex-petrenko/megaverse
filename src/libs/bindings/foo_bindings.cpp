#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

namespace py = pybind11;


int add(int i, int j)
{
    return i + j;
}

cv::Mat readImg()
{
    static auto img = cv::imread("/home/alex/Pictures/lenna.png");
//    cv::imshow("test", img);
//    cv::waitKey();
    return img;
}


class TestClass
{
public:
    TestClass(int number) : number{number}
    {}

    int getNumber() const { return number; }

    py::array_t<uint8_t> getBuffer()
    {
        cv::Mat mat = readImg();
        auto w = mat.cols;
        auto h = mat.rows;
        auto ch = 3;

        return py::array_t<uint8_t>({h, w, ch}, mat.data, py::none{});  // numpy object does not own memory
    }

private:
    int number;
};


PYBIND11_MODULE(foo_bindings, m)
{
    m.doc() = "pybind11 example plugin"; // optional module docstring
    m.def("add", &add, "A function which adds two numbers");

    m.def("readImg", []() {
        cv::Mat mat = readImg();
        auto w = mat.cols;
        auto h = mat.rows;
        auto ch = 3;

//        auto capsule = py::capsule(mat.data, [](void *v) { return; });
        return py::array_t<uint8_t>({h, w, ch}, mat.data, py::none{});
    });

    py::class_<TestClass>(m, "TestClass")
        .def(py::init<int>())
        .def("getNumber", &TestClass::getNumber)
        .def("getBuffer", &TestClass::getBuffer);
}
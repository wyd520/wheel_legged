//
// Created by yeyinong on 2023/12/20.
//

#ifndef MASTERSTACK_DAMIAO_PLOT_H
#define MASTERSTACK_DAMIAO_PLOT_H

#include "matplotlibcpp.h"
#include "vector"
#include "mutex"

using namespace std;
namespace plt = matplotlibcpp;

extern vector<vector<float>> plotData;

extern vector<float> plotXAxis;

extern const int plotSize;
extern int beginIndex;
extern mutex mutex_plot_data;

template<typename Numeric>
PyObject* get_array_user(const std::vector<Numeric>& v);

template<typename Numeric>
PyObject* get_array_user(const std::vector<Numeric>& v)
{
    npy_intp vsize = v.size();
    NPY_TYPES type = plt::detail::select_npy_type<Numeric>::type;
//    if (type == NPY_NOTYPE) {
    size_t memsize = v.size()*sizeof(double);
    double* dp = static_cast<double*>(::malloc(memsize));
    for (size_t i=0; i<v.size(); ++i){
        dp[i] = v[(beginIndex+i)%v.size()];
    }
    PyObject* varray = PyArray_SimpleNewFromData(1, &vsize, NPY_DOUBLE, dp);
    PyArray_UpdateFlags(reinterpret_cast<PyArrayObject*>(varray), NPY_ARRAY_OWNDATA);
    return varray;
//    }

//    PyObject* varray = PyArray_SimpleNewFromData(1, &vsize, type, (void*)(v.data()));
//    return varray;
}

template<typename NumericX, typename NumericY>
bool plotUser(const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& s="")
{
    assert(x.size() == y.size());
    plt::detail::_interpreter::get();

    PyObject* xarray = plt::detail::get_array(x);
    PyObject* yarray = get_array_user(y);

    PyObject* pystring = PyString_FromString(s.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_CallObject(plt::detail::_interpreter::get().s_python_function_plot, plot_args);

    Py_DECREF(plot_args);
    if(res) Py_DECREF(res);

    return res;
}

template<typename NumericX, typename NumericY>
bool named_plot_user(const string& name, const std::vector<NumericX>& x, const std::vector<NumericY>& y, const std::string& format="")
{
    plt::detail::_interpreter::get();

    PyObject* kwargs = PyDict_New();
    PyDict_SetItemString(kwargs, "label", PyString_FromString(name.c_str()));

    PyObject* xarray = plt::detail::get_array(x);

    mutex_plot_data.lock();
    PyObject* yarray = get_array_user(y);
    mutex_plot_data.unlock();

    PyObject* pystring = PyString_FromString(format.c_str());

    PyObject* plot_args = PyTuple_New(3);
    PyTuple_SetItem(plot_args, 0, xarray);
    PyTuple_SetItem(plot_args, 1, yarray);
    PyTuple_SetItem(plot_args, 2, pystring);

    PyObject* res = PyObject_Call(plt::detail::_interpreter::get().s_python_function_plot, plot_args, kwargs);

    Py_DECREF(kwargs);
    Py_DECREF(plot_args);
    if (res) Py_DECREF(res);

    return res;
}

void plotGraph();
#endif //MASTERSTACK_DAMIAO_PLOT_H

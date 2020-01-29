#ifndef filter_H
#define filter_H

#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>

// Templates for N-d matrices / arrays only!
// Template for N-d vectors, i.e. N-by-1 matrices
template <class T, int M>
using Vector = Eigen::Matrix<T, M, 1>;

template <class T, int M>
using VectorX = Vector<T, M>;

template <int N>
using VectorXd = VectorX<double, N>;

// Template for N-d matrices
template <class T, int M, int N>
using Matrix = Eigen::Matrix<T, M, N>;

template <class T, int M>
using MatrixX = Matrix<T, M, M>;

template <int N>
using MatrixXd = MatrixX<double, N>;

template <class T>
struct Constants
{
  static T epsilon() { return T(1e-8); }

  static T grav() { return T(9.80665); }

  static T pi()
  {
    return T(3.141592653589793238462643383279502884);
  }

  static T epsilonRads() { return T(3.1415926535897932384626433832e-8); }
};
#endif
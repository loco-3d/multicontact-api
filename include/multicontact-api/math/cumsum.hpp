// Copyright (c) 2015-2018, CNRS
// Authors: Justin Carpentier <jcarpent@laas.fr>

#ifndef __multicontact_api_math_cumsum_hpp__
#define __multicontact_api_math_cumsum_hpp__

#include <Eigen/Dense>

namespace multicontact_api {
namespace math {

namespace details {
template <typename DenseDerived1, typename DenseDerived2,
          bool IsVector = DenseDerived1::ColsAtCompileTime == 1 || DenseDerived1::RowsAtCompileTime == 1>
struct CumSumAlgo {
  static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res,
                  const int direction);

  template <int Direction>
  static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res);
};

template <typename DenseDerived1, typename DenseDerived2>
struct CumSumAlgo<DenseDerived1, DenseDerived2, true> {
  static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res) {
    Eigen::DenseBase<DenseDerived2> &res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
    res_[0] = dense[0];
    for (int k = 1; k < dense.size(); ++k) res_[k] = res_[k - 1] + dense[k];
  }
};

template <typename DenseDerived1, typename DenseDerived2>
struct CumSumAlgo<DenseDerived1, DenseDerived2, false> {
  template <int Direction, typename Dummy = void>
  struct Func {
    static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res) {
      Eigen::DenseBase<DenseDerived2> &res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
      res_.col(0) = dense.col(0);
      for (int k = 1; k < dense.cols(); ++k) res_.col(k) = res_.col(k - 1) + dense.col(k);
    }
  };

  template <typename Dummy>
  struct Func<Eigen::Horizontal, Dummy> {
    static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res) {
      Eigen::DenseBase<DenseDerived2> &res_ = const_cast<Eigen::DenseBase<DenseDerived2> &>(res);
      res_.row(0) = dense.row(0);
      for (int k = 1; k < dense.rows(); ++k) res_.row(k) = res_.row(k - 1) + dense.row(k);
    }
  };

  template <int Direction>
  static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res) {
    Func<Direction>::run(dense, res);
  }

  static void run(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res,
                  const int direction) {
    if (direction == Eigen::Vertical)
      run<Eigen::Vertical>(dense, res);
    else
      run<Eigen::Horizontal>(dense, res);
  }
};

}  // namespace details

template <typename DenseDerived1, typename DenseDerived2>
void cumsum(const Eigen::DenseBase<DenseDerived1> &dense, const Eigen::DenseBase<DenseDerived2> &res) {
  details::CumSumAlgo<DenseDerived1, DenseDerived2>::run(dense, res);
}

template <typename DenseDerived>
typename DenseDerived::PlainObject cumsum(const Eigen::DenseBase<DenseDerived> &dense) {
  typename DenseDerived::PlainObject res(dense.rows(), dense.cols());
  details::CumSumAlgo<DenseDerived, typename DenseDerived::PlainObject>::run(dense, res);
  return res;
}
}  // namespace math
}  // namespace multicontact_api

#endif  // ifndef __multicontact_api_math_cumsum_hpp__

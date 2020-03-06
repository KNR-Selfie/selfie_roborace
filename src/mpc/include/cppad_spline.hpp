#ifndef CPPAD_SPLINE_HPP
#define CPPAD_SPLINE_HPP

#include <spline.h>
#include <cppad/cppad.hpp>

namespace CppAD
{

class spline
{
public:
  spline(std::vector<double> x, std::vector<double> y);
  AD<double> operator() (AD<double> x) const;
  AD<double> deriv(AD<double> x) const;

private:
  tk::spline impl_;
};

spline::spline(std::vector<double> x, std::vector<double> y)
{
  impl_.set_points(x, y, false);
}

AD<double> spline::operator()(AD<double> x) const
{
  size_t n = impl_.m_x.size();

  // extrapolation to the left
  AD<double> h = x - impl_.m_x[0];
  AD<double> result = (impl_.m_b0*h + impl_.m_c0)*h + impl_.m_y[0];

  for (int i = 0; i < n-1; i++) {
    // interpolation
    h = x - impl_.m_x[i];
    result = CppAD::CondExpGt(x, (AD<double>)impl_.m_x[i], ((impl_.m_a[i]*h + impl_.m_b[i])*h + impl_.m_c[i])*h + impl_.m_y[i], result);
  }

  // extrapolation to the right
  h = x - impl_.m_x[n-1];
  result = CppAD::CondExpGt(x, (AD<double>)impl_.m_x[n-1], (impl_.m_b[n-1]*h + impl_.m_c[n-1])*h + impl_.m_y[n-1], result);

  return result;
}

AD<double> spline::deriv(AD<double> x) const
{
  size_t n = impl_.m_x.size();

  // extrapolation to the left
  AD<double> h = x - impl_.m_x[0];
  AD<double> result = 2*impl_.m_b0*h + impl_.m_c0;

  for (int i = 0; i < n-1; i++) {
    // interpolation
    h = x - impl_.m_x[i];
    result = CppAD::CondExpGt(x, (AD<double>)impl_.m_x[i], 3*impl_.m_a[i]*h*h + 2*impl_.m_b[i]*h + impl_.m_c[i], result);
  }

  // extrapolation to the right
  h = x - impl_.m_x[n-1];
  result = CppAD::CondExpGt(x, (AD<double>)impl_.m_x[n-1], 2*impl_.m_b[n-1]*h + impl_.m_c[n-1], result);

  return result;
}


} // namespace CppAD

#endif // CPPAD_SPLINE_HPP

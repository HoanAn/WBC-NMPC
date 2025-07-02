#ifndef LABROB_TIMING_LAW_HPP_
#define LABROB_TIMING_LAW_HPP_

#include <cmath>

namespace labrob {

class TimingLaw {
 public:
  TimingLaw(double duration) : duration_(duration) { }
  virtual ~TimingLaw() = 0;

  virtual double eval(double t) const = 0;
  virtual double eval_dt(double t) const = 0;
  virtual double eval_dt_dt(double t) const {
    return 0.0;
  }

  double get_duration() const {
    return duration_;
  }

 protected:
  double duration_;
}; // end class TimingLaw

TimingLaw::~TimingLaw() {
  
}

class LinearTimingLaw : public TimingLaw {
 public:
  LinearTimingLaw(double duration) : TimingLaw(duration) {

  }

  ~LinearTimingLaw() = default;

  double eval(double t) const override {
    return t / duration_;
  }

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
  double eval_dt(double t) const override {
    return 1.0 / duration_;
  }
#pragma GCC diagnostic pop

}; // end class LinearTimingLaw

class QuinticPolynomialTimingLaw : public TimingLaw {
 public:
  // Setup a problem of the kind:
  // s(t) = a * t ^ 5 + b * t ^ 4 + c * t ^ 3 + d * t ^ 2 + e * t + f
  // s(0) = s_dot(0) = s_ddot(0) = s_dot(duration) = s_ddot(duration) = 0
  // s(duration) = 1
  QuinticPolynomialTimingLaw(double duration) :
    TimingLaw(duration),
    a_(  6.0 / std::pow(duration, 5.0)),
    b_(-15.0 / std::pow(duration, 4.0)),
    c_( 10.0 / std::pow(duration, 3.0)) {
    
  }

  ~QuinticPolynomialTimingLaw() = default;

  double eval(double t) const override {
    return a_ * std::pow(t, 5.0) +
           b_ * std::pow(t, 4.0) +
           c_ * std::pow(t, 3.0);
  }

  double eval_dt(double t) const override {
    return 5.0 * a_ * std::pow(t, 4.0) +
           4.0 * b_ * std::pow(t, 3.0) +
           3.0 * c_ * std::pow(t, 2.0);
  }

  double eval_dt_dt(double t) const override {
    return 20.0 * a_ * std::pow(t, 3.0) +
           12.0 * b_ * std::pow(t, 2.0) +
           6.0 * c_ * t;
  }

 private:
  const double a_, b_, c_;
}; // end class QuinticPolynomialTimingLaw

/**
 * \brief Class for timing law with trapezoidal acceleration.
 */
class TrapezoidalAccelerationTimingLaw : public TimingLaw {
 public:
  /**
   * \brief Constructor of a timing law with trapezoidal acceleration.
   * \param duration Duration of the timing law.
   * \param alpha Define where to have maximum acceleration. 
   */
  TrapezoidalAccelerationTimingLaw(double duration, double alpha)
  : TimingLaw(duration), alpha_(alpha) {
    t0_ = 0.0;
    ta_ = t0_ + alpha_ * duration_;
    tb_ = t0_ + (1.0 - alpha_) * duration_;
    tf_ = t0_ + duration_;

    a_max_ = std::pow(
        (std::pow(ta_, 2.0) / 2.0 - t0_ * ta_ + std::pow(t0_, 2.0) / 2.0) + 
        (ta_ - t0_) * (tf_ - ta_) -
        (std::pow(tf_, 2.0) / 2.0 - tb_ * tf_ + std::pow(tb_, 2.0) / 2.0), -1.0);
  }

  ~TrapezoidalAccelerationTimingLaw() = default;

  /**
   * \brief Evaluate the timing law at a given time
   * \param t Time used to evaluate the timing law.
   */
  double eval(double t) const override {
    if (t <= ta_) {
      return s_0(t);
    } else if(t <= tb_) {
      return s_1(t);
    }
    return s_2(t);
  }

  /**
   * \brief Evaluate the timing law derivative at a given time
   * \param t Time used to evaluate the timing law derivative.
   */
  double eval_dt(double t) const override {
    if (t <= ta_) {
      return sdot_0(t);
    } else if (t <= tb_) {
      return sdot_0(ta_);
    }
    return sdot_2(t);
  }

 private:
  /**
   * Evaluate the timing law at \param t with \param t less than
   * or equal to \param ta_.
   */
  double s_0(double t) const {
    return a_max_ * 
        (std::pow(t, 2.0) / 2.0 - t0_ * t + std::pow(t0_, 2.0) / 2.0);
  }

  /**
   * Evaluate the timing law at \param t with \param t between
   * \param ta_ and \param tb_.
   */
  double s_1(double t) const {
    return s_0(ta_) + a_max_ * (ta_ - t0_) * (t - ta_);
  }

  /**
   * Evaluate the timing law at \param t with \param t greater than
   * or equal to \param tb_.
   */
  double s_2(double t) const {
    return s_1(t) - a_max_ *
        (std::pow(t, 2.0) / 2.0 - tb_ * t + std::pow(tb_, 2.0) / 2.0);
  }

  /**
   * Evaluate the timing law derivative at \param t with \param t less than
   * or equal to \param ta_.
   */
  double sdot_0(double t) const {
    return a_max_ * (t - t0_);
  }

  /**
   * Evaluate the timing law derivative at \param t with \param t greater than
   * or equal to \param tb_.
   */
  double sdot_2(double t) const {
    return sdot_0(ta_) - a_max_ * (t - tb_);
  }

  double alpha_;
  double t0_, ta_, tb_, tf_;
  double a_max_;

}; // end class TrapezoidalAccelerationTimingLaw

} // end namespace labrob

#endif // LABROB_TIMING_LAW_HPP_
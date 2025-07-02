#ifndef LABROB_DOUBLE_SUPPORT_CONFIGURATION_HPP_
#define LABROB_DOUBLE_SUPPORT_CONFIGURATION_HPP_

#include <hrp4_locomotion/Foot.hpp>
#include <hrp4_locomotion/SE3.hpp>

namespace labrob {

class DoubleSupportConfiguration {
 public:
  DoubleSupportConfiguration(
      const labrob::SE3& qLeft,
      const labrob::SE3& qRight,
      const labrob::Foot& support_foot
  );

  const labrob::SE3& getLeftFootConfiguration() const;
  const labrob::SE3& getRightFootConfiguration() const;
  const labrob::SE3& getSupportFootConfiguration() const;
  const labrob::SE3& getSwingFootConfiguration() const;
  const labrob::Foot& getSupportFoot() const;

  void setFeetConfiguration(const labrob::SE3& qLeft, const labrob::SE3& qRight);
  void setLeftFootConfiguration(const labrob::SE3& qLeft);
  void setRightFootConfiguration(const labrob::SE3& qRight);
  void setSupportFootConfiguration(const labrob::SE3& qSupport);
  void setSwingFootConfiguration(const labrob::SE3& qSwing);
  void setSupportFoot(const labrob::Foot& support_foot);

 protected:
  labrob::SE3 qLeft_;
  labrob::SE3 qRight_;
  labrob::Foot support_foot_;

}; // end class DoubleSupportConfiguration

} // end namespace labrob

#endif // LABROB_DOUBLE_SUPPORT_CONFIGURATION_HPP_
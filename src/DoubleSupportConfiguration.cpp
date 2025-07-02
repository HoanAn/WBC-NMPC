#include <hrp4_locomotion/DoubleSupportConfiguration.hpp>

namespace labrob {

DoubleSupportConfiguration::DoubleSupportConfiguration(
    const labrob::SE3& qLeft,
    const labrob::SE3& qRight,
    const labrob::Foot& support_foot
)  : qLeft_(qLeft),
      qRight_(qRight),
      support_foot_(support_foot) {

}

const labrob::SE3&
DoubleSupportConfiguration::getLeftFootConfiguration() const {
  return qLeft_;
}

const labrob::SE3&
DoubleSupportConfiguration::getRightFootConfiguration() const {
  return qRight_;
}

const labrob::SE3&
DoubleSupportConfiguration::getSupportFootConfiguration() const {
  if (support_foot_ == labrob::Foot::LEFT) return qLeft_;
  else return qRight_;
}

const labrob::SE3&
DoubleSupportConfiguration::getSwingFootConfiguration() const {
  if (support_foot_ == labrob::Foot::LEFT) return qRight_;
  else return qLeft_;
}

const labrob::Foot&
DoubleSupportConfiguration::getSupportFoot() const {
  return support_foot_;
}

void
DoubleSupportConfiguration::setFeetConfiguration(
    const labrob::SE3& qLeft, const labrob::SE3& qRight) {
  qLeft_ = qLeft;
  qRight_ = qRight;
}


void
DoubleSupportConfiguration::setLeftFootConfiguration(const labrob::SE3& qLeft) {
  qLeft_ = qLeft;
}

void
DoubleSupportConfiguration::setRightFootConfiguration(
    const labrob::SE3& qRight) {
  qRight_ = qRight;
}

void
DoubleSupportConfiguration::setSupportFootConfiguration(
    const labrob::SE3& qSupport) {
  if (support_foot_ == labrob::Foot::LEFT) qLeft_ = qSupport;
  else qRight_ = qSupport;
}

void
DoubleSupportConfiguration::setSwingFootConfiguration(
    const labrob::SE3& qSwing) {
  if (support_foot_ == labrob::Foot::LEFT) qRight_ = qSwing;
  else qLeft_ = qSwing;
}

void
DoubleSupportConfiguration::setSupportFoot(
    const labrob::Foot& support_foot) {
  support_foot_ = support_foot;
}

} // end namespace labrob
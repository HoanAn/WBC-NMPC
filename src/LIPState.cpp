#include <hrp4_locomotion/LIPState.hpp>

namespace labrob {

LIPState::LIPState(
    const Eigen::Vector3d& com_pos,
    const Eigen::Vector3d& com_vel,
    const Eigen::Vector3d& zmp_pos
) : com_pos_(com_pos),
    com_vel_(com_vel),
    zmp_pos_(zmp_pos) {

}

} // end namespace labrob
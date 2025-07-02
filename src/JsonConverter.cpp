#include <hrp4_locomotion/JsonConverter.hpp>

namespace labrob {
namespace JsonConverter {

void toJson(const labrob::LIPState& lip_state, Json::Value& lip_state_json) {
    Json::Value com_pos_json;
    com_pos_json["x"] = lip_state.com_pos_.x();
    com_pos_json["y"] = lip_state.com_pos_.y();
    com_pos_json["z"] = lip_state.com_pos_.z();
    Json::Value com_vel_json;
    com_vel_json["x"] = lip_state.com_vel_.x();
    com_vel_json["y"] = lip_state.com_vel_.y();
    com_vel_json["z"] = lip_state.com_vel_.z();
    Json::Value zmp_pos_json;
    zmp_pos_json["x"] = lip_state.zmp_pos_.x();
    zmp_pos_json["y"] = lip_state.zmp_pos_.y();
    zmp_pos_json["z"] = lip_state.zmp_pos_.z();
    lip_state_json["com_pos"] = com_pos_json;
    lip_state_json["com_vel"] = com_vel_json;
    lip_state_json["zmp_pos"] = zmp_pos_json;
}


} // end namespace JsonConverter
} // end namespace labrob
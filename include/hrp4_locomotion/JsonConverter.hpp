#ifndef LABROB_LOCOMOTION_JSON_CONVERTER
#define LABROB_LOCOMOTION_JSON_CONVERTER

#include <json/json.h>

#include <hrp4_locomotion/LIPState.hpp>

namespace labrob {
namespace JsonConverter {

void toJson(const labrob::LIPState& lip_state, Json::Value& lip_state_json);

} // end namespace JsonConverter
} // end namespace labrob

#endif
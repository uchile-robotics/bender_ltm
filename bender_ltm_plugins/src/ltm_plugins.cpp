#include <pluginlib/class_list_macros.h>

#include <bender_ltm_plugins/emotion_plugin.h>
#include <bender_ltm_plugins/location_plugin.h>
#include <bender_ltm_plugins/human_entity_plugin.h>
#include <bender_ltm_plugins/crowd_entity_plugin.h>

PLUGINLIB_EXPORT_CLASS(bender_ltm_plugins::EmotionPlugin, ltm::plugin::EmotionBase)
PLUGINLIB_EXPORT_CLASS(bender_ltm_plugins::LocationPlugin, ltm::plugin::LocationBase)
PLUGINLIB_EXPORT_CLASS(bender_ltm_plugins::HumanEntityPlugin, ltm::plugin::EntityBase)
PLUGINLIB_EXPORT_CLASS(bender_ltm_plugins::CrowdEntityPlugin, ltm::plugin::EntityBase)

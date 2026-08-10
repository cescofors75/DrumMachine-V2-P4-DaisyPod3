#pragma once

#include "master/protocol.h"

bool pod_config_store_load(PodConfigPayload& config);
bool pod_config_store_save(const PodConfigPayload& config);
void pod_config_store_sanitize(PodConfigPayload& config);
void pod_config_store_factory_defaults(PodConfigPayload& config);

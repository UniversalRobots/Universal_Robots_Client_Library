/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once
#include <inttypes.h>

#ifdef ROS_BUILD
#include <console_bridge/console.h>

#define URCL_LOG_DEBUG CONSOLE_BRIDGE_logDebug
#define URCL_LOG_WARN CONSOLE_BRIDGE_logWarn
#define URCL_LOG_INFO CONSOLE_BRIDGE_logInform
#define URCL_LOG_ERROR CONSOLE_BRIDGE_logError
#define URCL_LOG_FATAL CONSOLE_BRIDGE_logError

#else

#define URCL_LOG_DEBUG(format, ...) printf("[DEBUG]: " format "\n", ##__VA_ARGS__)
#define URCL_LOG_WARN(format, ...) printf("[WARNING]: " format "\n", ##__VA_ARGS__)
#define URCL_LOG_INFO(format, ...) printf("[INFO]: " format "\n", ##__VA_ARGS__)
#define URCL_LOG_ERROR(format, ...) printf("[ERROR]: " format "\n", ##__VA_ARGS__)
#define URCL_LOG_FATAL(format, ...) printf("[FATAL]: " format "\n", ##__VA_ARGS__)

#endif

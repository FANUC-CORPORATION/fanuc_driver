// SPDX-FileCopyrightText: 2025-2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2025-2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>

#include "rmi/struct.hpp"

namespace rmi
{

template <typename T>
std::string ToJSON(const T& data);

template <typename T>
std::optional<T> FromJSON(const std::string& json);

std::optional<InstructionResponse> IsInstruction(const std::string& json);

}  // namespace rmi

// SPDX-FileCopyrightText: 2026, FANUC America Corporation
// SPDX-FileCopyrightText: 2026, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <string>
#include <variant>

namespace rmi
{
/* Not a real packet type */
struct InstructionResponse
{
  std::string Instruction;
  int ErrorID;
  int SequenceID;
};

using RMICallParam = std::pair<std::string, std::variant<int, float, std::string>>;
}  // namespace rmi

# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

# 0 running
# 1 paused
# 2 aborted
uint32 program_status

uint32 next_sequence

uint32 last_sequence
int32 last_sequence_error
string last_sequence_instruction
string last_sequence_error_message

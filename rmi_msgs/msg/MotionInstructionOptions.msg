# SPDX-FileCopyrightText: 2026, FANUC America Corporation
# SPDX-FileCopyrightText: 2026, FANUC CORPORATION
#
# SPDX-License-Identifier: Apache-2.0

bool use_acc false
uint8 acc

bool use_offset_pr_number false
uint16 offset_pr_number

bool use_vision_pr_number false
uint16 vision_pr_number

bool use_wrist_joint false
string wrist_joint "ON"

bool use_mrot false
string mrot "ON"

bool use_lcb_type false
string lcb_type
string LCB_TYPE_TB="TB" # Time Before
string LCB_TYPE_TA="TA" # Time After
string LCB_TYPE_DB="DB" # Distance Before

bool use_lcb_value false
uint16 lcb_value

bool use_port_type false
uint8 port_type
uint8 PORT_TYPE_DOUT=1
uint8 PORT_TYPE_DO=1
uint8 PORT_TYPE_ROUT=2
uint8 PORT_TYPE_RO=2

bool use_port_number false
uint16 port_number

bool use_port_value false
string port_value

bool use_tool_offset_pr_number false
uint16 tool_offset_pr_number

bool use_alim false
int32 alim

bool use_alim_reg false
uint16 alim_reg

bool use_no_blend false
string no_blend "ON"

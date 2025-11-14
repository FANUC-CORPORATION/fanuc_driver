// SPDX-FileCopyrightText: 2025, FANUC America Corporation
// SPDX-FileCopyrightText: 2025, FANUC CORPORATION
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <optional>
#include <string>
#include <variant>

namespace rmi
{
// Note: The struct names in this file use capitalized names to match the JSON packet keys exactly.
// This is required for correct serialization/deserialization with the cpp-reflect library.
// The struct naming cannot be changed to another style for this reason.
// The packet definitions are from Remote Motion Interface OPERATOR'S MANUAL B-84184EN/03.

struct FrameData
{
  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;
  float W = 0.0;
  float P = 0.0;
  float R = 0.0;
};

struct PositionData
{
  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;
  float W = 0.0;
  float P = 0.0;
  float R = 0.0;
  float Ext1 = 0.0;
  float Ext2 = 0.0;
  float Ext3 = 0.0;
};

struct ConfigurationData
{
  int UToolNumber = 0;
  int UFrameNumber = 0;
  int Front = 0;
  int Up = 0;
  int Left = 0;
  int Flip = 0;
  int Turn4 = 0;
  int Turn5 = 0;
  int Turn6 = 0;
};

struct JointAngleData
{
  float J1 = 0.0f;
  float J2 = 0.0f;
  float J3 = 0.0f;
  float J4 = 0.0f;
  float J5 = 0.0f;
  float J6 = 0.0f;
  float J7 = 0.0f;
  float J8 = 0.0f;
  float J9 = 0.0f;
};

struct CartesianMotionGroup
{
  ConfigurationData Configuration;
  PositionData Position;
};

// 2.2.1 Packet Exchange
struct ConnectPacket
{
  struct Request
  {
    std::string Communication = "FRC_Connect";
  };
  struct Response
  {
    std::string Communication = "FRC_Connect";
    int ErrorID;
    uint16_t PortNumber;
    uint16_t MajorVersion;
    uint16_t MinorVersion;
  };
};

// 2.2.2 Controller Disconnect Packet
struct DisconnectPacket
{
  struct Request
  {
    std::string Communication = "FRC_Disconnect";
  };
  struct Response
  {
    std::string Communication = "FRC_Disconnect";
    int ErrorID;
  };
};

// 2.2.3 Timeout Terminate Packet
struct TimeoutTerminatePacket
{
  std::string Communication = "FRC_Terminate";
};

// 2.2.4 System Fault Packet
struct SystemFaultPacket
{
  std::string Communication = "FRC_SystemFault";
  int SequenceID;
};

// 2.3.1 Packet To Initialize Remote Motion
struct InitializePacket
{
  struct Request
  {
    std::string Command = "FRC_Initialize";
    std::optional<uint8_t> GroupMask;
    std::optional<std::string> RTSA;
    std::optional<std::string> PLTZMODE;
  };

  struct Response
  {
    std::string Command = "FRC_Initialize";
    int ErrorID;
    std::optional<uint8_t> GroupMask;
  };
};

// 2.3.2 Packet To Abort Remote Motion TP Program
struct AbortPacket
{
  struct Request
  {
    std::string Command = "FRC_Abort";
  };

  struct Response
  {
    std::string Command = "FRC_Abort";
    int ErrorID;
  };
};

// 2.3.3 Packet To Pause Remote Motion Program
struct PausePacket
{
  struct Request
  {
    std::string Command = "FRC_Pause";
  };

  struct Response
  {
    std::string Command = "FRC_Pause";
    int ErrorID;
  };
};

// 2.3.4 Packet To Continue Remote Motion Program
struct ContinuePacket
{
  struct Request
  {
    std::string Command = "FRC_Continue";
  };

  struct Response
  {
    std::string Command = "FRC_Continue";
    int ErrorID;
  };
};

// 2.3.5 Packet To Read Controller Error
struct ReadErrorPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadError";
    std::optional<uint8_t> Count;
  };

  struct Response
  {
    std::string Command = "FRC_ReadError";
    int ErrorID;
    std::optional<uint8_t> Count;  // valid range [1, 5]
    std::string ErrorData;
    std::optional<std::string> ErrorData2;
    std::optional<std::string> ErrorData3;
    std::optional<std::string> ErrorData4;
    std::optional<std::string> ErrorData5;
  };
};

// 2.3.6 Packet To Set The Current UFrame-UTool Number
struct SetUFrameToolFramePacket
{
  struct Request
  {
    std::string Command = "FRC_SetUFrameUTool";
    int UFrameNumber;
    int UToolNumber;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_SetUFrameUTool";
    int ErrorID;
    std::optional<uint8_t> Group;
  };
};

// 2.3.7 Packet To Get Controller Status
struct StatusRequestPacket
{
  struct Request
  {
    std::string Command = "FRC_GetStatus";
  };

  struct Response
  {
    std::string Command = "FRC_GetStatus";
    int ErrorID;
    uint8_t ServoReady;
    uint8_t TPMode;
    uint8_t RMIMotionStatus;
    uint8_t ProgramStatus;
    uint8_t SingleStepMode;
    uint8_t NumberUTool;
    int NextSequenceID;
    uint8_t NumberUFrame;
    uint8_t Override;
  };
};

// 2.3.8 Packet To Read User Frame Data
struct ReadUFrameDataPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadUFrameData";
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_ReadUFrameData";
    int ErrorID;
    uint8_t FrameNumber;
    FrameData Frame;
    std::optional<uint8_t> Group;
  };
};

// 2.3.9 Packet To Set User Frame Data
struct WriteUFrameDataPacket
{
  struct Request
  {
    std::string Command = "FRC_WriteUFrameData";
    uint8_t FrameNumber;
    FrameData Frame;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_WriteUFrameData";
    int ErrorID;
    std::optional<uint8_t> Group;
  };
};

// 2.3.10 Packet To Read User Tool Data
struct ReadUToolDataPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadUToolData";
    int ToolNumber;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_ReadUToolData";
    int ErrorID;
    uint8_t ToolNumber;
    FrameData Frame;
    ConfigurationData Configuration;
    std::optional<uint8_t> Group;
  };
};

// 2.3.11 Packet To Set User Tool Data
struct WriteUToolDataPacket
{
  struct Request
  {
    std::string Command = "FRC_WriteUToolData";
    uint8_t ToolNumber;
    FrameData Frame;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_WriteUToolData";
    int ErrorID;
    std::optional<uint8_t> Group;
  };
};

// 2.3.12 Packet To Read Digital Input Port
struct ReadDigitalInputPortPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadDIN";
    uint16_t PortNumber;
  };

  struct Response
  {
    std::string Command = "FRC_ReadDIN";
    int ErrorID;
    uint16_t PortNumber;
    uint8_t PortValue;
  };
};

// 2.3.13 Packet To Write Digital Output Port
struct WriteDigitalOutputPacket
{
  struct Request
  {
    std::string Command = "FRC_WriteDOUT";
    uint16_t PortNumber;
    std::string PortValue;  // "ON" or "OFF"
  };

  struct Response
  {
    std::string Command = "FRC_WriteDOUT";
    int ErrorID;
  };
};

// 2.3.14 Packet To Read Current Robot Cartesian Position
struct GetCartesianPositionPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadCartesianPosition";
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_ReadCartesianPosition";
    int ErrorID;
    int TimeTag;
    ConfigurationData Configuration;
    PositionData Position;
    std::optional<uint8_t> Group;
  };
};

// 2.3.15 Packet To Read Current Robot Joint Angles
struct ReadJointAnglesPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadJointAngles";
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_ReadJointAngles";
    int ErrorID;
    int TimeTag;
    JointAngleData JointAngle;
    std::optional<uint8_t> Group;
  };
};

// 2.3.16 Packet To Set Speed Override
struct SetSpeedOverridePacket
{
  struct Request
  {
    std::string Command = "FRC_SetOverRide";
    int Value;
  };

  struct Response
  {
    std::string Command = "FRC_SetOverRide";
    int ErrorID;
  };
};

// 2.3.17 Packet To Get Current User/Tool Frame Number
struct GetUFrameToolFramePacket
{
  struct Request
  {
    std::string Command = "FRC_GetUFrameUTool";
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_GetUFrameUTool";
    int ErrorID;
    uint8_t UFrameNumber;
    uint8_t UToolNumber;
    std::optional<uint8_t> Group;
  };
};

// 2.3.18 Packet To Read Position Register Data
struct ReadPositionRegisterPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadPositionRegister";
    uint16_t RegisterNumber;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_ReadPositionRegister";
    int ErrorID;
    uint16_t RegisterNumber;
    std::string Representation;
    std::optional<ConfigurationData> Configuration;
    std::optional<PositionData> Position;
    std::optional<JointAngleData> JointAngle;
    std::optional<uint8_t> Group;
  };
};

// 2.3.19 Packet To Write Position Register Data
struct WritePositionRegisterPacket
{
  struct Request
  {
    std::string Command = "FRC_WritePositionRegister";
    uint16_t RegisterNumber;
    std::optional<std::string> Representation;
    std::optional<ConfigurationData> Configuration;
    std::optional<PositionData> Position;
    std::optional<JointAngleData> JointAngle;
    std::optional<uint8_t> Group;
  };

  struct Response
  {
    std::string Command = "FRC_WritePositionRegister";
    int ErrorID;
    uint16_t RegisterNumber;
    std::optional<uint8_t> Group;
  };
};

// 2.3.20 Packet To Reset Robot Controller
struct ResetRobotPacket
{
  struct Request
  {
    std::string Command = "FRC_Reset";
  };

  struct Response
  {
    std::string Command = "FRC_Reset";
    int ErrorID;
  };
};

// 2.3.21 Packet To Read Current Tool Center Point Speed
struct GetTCPSpeedPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadTCPSpeed";
  };

  struct Response
  {
    std::string Command = "FRC_ReadTCPSpeed";
    int ErrorID;
    int TimeTag;
    float Speed;  // in mm/sec
  };
};

// 2.4.2 Packet To Set User Frame Instruction
struct WaitForDINPacket
{
  struct Request
  {
    std::string Instruction = "FRC_WaitDIN";
    int SequenceID;
    uint16_t PortNumber;
    std::string PortValue;  // "ON" or "OFF"
  };

  struct Response
  {
    std::string Instruction = "FRC_WaitDIN";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.3 Packet To Set User Tool Instruction
struct SetUFramePacket
{
  struct Request
  {
    std::string Instruction = "FRC_SetUFrame";
    int SequenceID;
    uint8_t FrameNumber;
  };

  struct Response
  {
    std::string Instruction = "FRC_SetUFrame";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.4 Packet To Add Wait Time Instruction
struct SetToolFramePacket
{
  struct Request
  {
    std::string Instruction = "FRC_SetUTool";
    int SequenceID;
    uint8_t ToolNumber;
  };

  struct Response
  {
    std::string Instruction = "FRC_SetUTool";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.4 Packet To Add Wait Time Instruction
struct WaitForTimePacket
{
  struct Request
  {
    std::string Instruction = "FRC_WaitTime";
    int SequenceID;
    float Time;  // in seconds
  };

  struct Response
  {
    std::string Instruction = "FRC_WaitTime";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.5 Packet To Set Payload Instruction
struct SetPayloadPacket
{
  struct Request
  {
    std::string Command = "FRC_SetPayloadID";
    uint8_t ScheduleNumber;
    std::optional<uint8_t> Group;
  };
  struct Response
  {
    std::string Command = "FRC_SetPayloadID";
    int ErrorID;
  };
};

// 2.4.6 Packet To Call A Program
struct ProgramCallPacket
{
  struct Request
  {
    std::string Instruction = "FRC_Call";
    int SequenceID;
    std::string ProgramName;
  };

  struct Response
  {
    std::string Instruction = "FRC_Call";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.7 Packet To Add Linear Motion Instruction
struct LinearMotionPacket
{
  struct Request
  {
    std::string Instruction = "FRC_LinearMotion";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> WristJoint;  // "ON"
    std::optional<std::string> MROT;        // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<int> ALIM;
    std::optional<uint16_t> ALIMREG;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_LinearMotion";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.8 Packet To Add Linear Incremental Motion Instruction
struct LinearRelativePacket
{
  struct Request
  {
    std::string Instruction = "FRC_LinearRelative";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> WristJoint;  // "ON"
    std::optional<std::string> MROT;        // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<int> ALIM;
    std::optional<uint16_t> ALIMREG;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_LinearRelative";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.9 Packet To Add Joint Motion Instruction
struct JointMotionPacket
{
  struct Request
  {
    std::string Instruction = "FRC_JointMotion";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> MROT;  // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_JointMotion";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.10 Packet To Add Joint Incremental Motion Instruction
struct JointRelativePacket
{
  struct Request
  {
    std::string Instruction = "FRC_JointRelative";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> MROT;  // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_JointRelative";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.11 Packet To Add Circular Motion Instruction
struct CircularMotionPacket
{
  struct Request
  {
    std::string Instruction = "FRC_CircularMotion";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    ConfigurationData ViaConfiguration;
    PositionData ViaPosition;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> WristJoint;  // "ON"
    std::optional<std::string> MROT;        // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_CircularMotion";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.12 Packet To Add Circular Incremental Motion Instruction
struct CircularRelativePacket
{
  struct Request
  {
    std::string Instruction = "FRC_CircularRelative";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    ConfigurationData ViaConfiguration;
    PositionData ViaPosition;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> WristJoint;  // "ON"
    std::optional<std::string> MROT;        // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_CircularRelative";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.13 Packet To Add Joint Motion With Joint Representation
struct JointMotionJRepPacket
{
  struct Request
  {
    std::string Instruction = "FRC_JointMotionJRep";
    int SequenceID;
    JointAngleData JointAngle;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> MROT;  // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_JointMotionJRep";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.14 Packet To Add Joint Incremental Motion With Joint Representation
struct JointRelativeJRepPacket
{
  struct Request
  {
    std::string Instruction = "FRC_JointRelativeJRep";
    int SequenceID;
    JointAngleData JointAngle;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> MROT;  // "ON"
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
    std::optional<std::string> NoBlend;  // "ON"
  };

  struct Response
  {
    std::string Instruction = "FRC_JointRelativeJRep";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.15 Packet To Add Linear Motion With Joint Representation
struct LinearMotionJRepPacket
{
  struct Request
  {
    std::string Instruction = "FRC_LinearMotionJRep";
    int SequenceID;
    JointAngleData JointAngle;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
  };

  struct Response
  {
    std::string Instruction = "FRC_LinearMotionJRep";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.16 Packet To Add Linear Incremental Motion With Joint Representation
struct LinearRelativeJRepPacket
{
  struct Request
  {
    std::string Instruction = "FRC_LinearRelativeJRep";
    int SequenceID;
    JointAngleData JointAngle;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
  };

  struct Response
  {
    std::string Instruction = "FRC_LinearRelativeJRep";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.17 Packet To Add Spline Motion Instruction
struct SplineMotionPacket
{
  struct Request
  {
    std::string Instruction = "FRC_SplineMotion";
    int SequenceID;
    ConfigurationData Configuration;
    PositionData Position;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
  };
  struct Response
  {
    std::string Instruction = "FRC_SplineMotion";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.18 Packet To Add Spline Motion With Joint Representation
struct SplineMotionJRepPacket
{
  struct Request
  {
    std::string Instruction = "FRC_SplineMotionJRep";
    int SequenceID;
    JointAngleData JointAngle;
    std::string SpeedType;
    uint16_t Speed;
    std::string TermType;
    uint8_t TermValue;
    std::optional<uint8_t> ACC;
    std::optional<uint16_t> OffsetPRNumber;
    std::optional<uint16_t> VisionPRNumber;
    std::optional<std::string> LCBType;
    std::optional<uint16_t> LCBValue;
    std::optional<uint8_t> PortType;
    std::optional<uint16_t> portNumber;
    std::optional<std::string> portValue;
    std::optional<uint16_t> ToolOffsetPRNumber;
  };

  struct Response
  {
    std::string Instruction = "FRC_SplineMotionJRep";
    int ErrorID;
    int SequenceID;
  };
};

// 2.4.19 Unknown Packet Handling
struct UnknownPacket
{
  std::string Command = "Unknown";
  int ErrorID;
  std::optional<int> SequenceID;
};

// 5.3.1 ASCII String Packet For Single Group Controller
struct CreateASCIIPacket
{
  struct Request
  {
    const std::string ASCII;
    int SequenceID;
  };

  struct Response
  {
    const std::string ASCII;
    int ErrorID;
    int SequenceID;
  };
};

// 5.3.3 Communication Packet
struct CommunicationPacket
{
  std::string Communication = "FRC_AsbnReady";
};

// New packets not defined in B-84184EN/03
struct ReadNumericRegisterPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadRegister";
    int RegisterNumber;
  };

  struct Response
  {
    std::string Command = "FRC_ReadRegister";
    int ErrorID;
    std::string DataType;
    std::variant<int, float> RegisterValue;
  };
};

struct WriteNumericRegisterPacket
{
  struct Request
  {
    std::string Command = "FRC_WriteRegister";
    int RegisterNumber;
    std::variant<int, float> RegisterValue;
    std::string DataType;
  };

  struct Response
  {
    std::string Command = "FRC_WriteRegister";
    int ErrorID;
  };
};

struct ReadIOPortPacket
{
  struct Request
  {
    std::string Command = "FRC_ReadIOPort";
    std::string PortType;
    int PortNumber;
  };

  struct Response
  {
    std::string Command = "FRC_ReadIOPort";
    int ErrorID;
    std::string PortType;
    uint16_t PortNumber;
    std::variant<int, float> PortValue;
  };
};

struct WriteIOPortPacket
{
  struct Request
  {
    std::string Command = "FRC_WriteIOPort";
    std::string PortType;  // “AO”, “GO”, “DO”, “RO”, or “FLAG”
    uint16_t PortNumber;
    std::variant<int, float> PortValue;
  };

  struct Response
  {
    std::string Command = "FRC_WriteIOPort";
    int ErrorID;
    std::string PortType;
    int PortNumber;
  };
};

struct ReadVariablePacket
{
  struct Request
  {
    std::string Command = "FRC_ReadVariable";
    std::string VariableName;  // max 64 bytes, includes leading '$'
  };

  struct Response
  {
    std::string Command = "FRC_ReadVariable";
    int ErrorID;
    std::string VariableName;  // max 64 bytes, includes leading '$'
    std::string VariableType;  // "Integer" or "Float"
    std::variant<int, float> VariableValue;
  };
};

struct WriteVariablePacket
{
  struct Request
  {
    std::string Command = "FRC_WriteVariable";
    std::string VariableName;  // max 64 bytes, includes leading '$'
    std::string VariableType;  // "Integer" or "Float"
    std::variant<int, float> VariableValue;
  };

  struct Response
  {
    std::string Command = "FRC_WriteVariable";
    int ErrorID;
    std::string VariableName;  // max 64 bytes, includes leading '$'
  };
};

struct GetExtendedStatusPacket
{
  struct Request
  {
    std::string Command = "FRC_GetExtStatus";
  };

  struct Response
  {
    std::string Command = "FRC_GetExtStatus";
    int ErrorID;
    std::optional<std::string> ErrorCode;
    int InMotion;
    std::optional<std::string> ControlMode;
    int DrivesPowered;
    int GenOverride;
    std::optional<float> SpeedClampLimit;
  };
};

struct ConnectROS2Packet
{
  struct Request
  {
    std::string Communication = "FRC_Connect_STMO";
  };
  struct Response
  {
    std::string Communication = "FRC_Connect_STMO";
    int ErrorID;
    uint16_t PortNumber;
    uint16_t MajorVersion;
    uint16_t MinorVersion;
  };
};

struct SetPayloadValuePacket
{
  struct Request
  {
    std::string Command = "FRC_SetPayloadValue";
    uint8_t ScheduleNumber;
    std::optional<uint8_t> Group;
    float Mass;
    float CG_X;
    float CG_Y;
    float CG_Z;
    std::optional<float> IN_X;
    std::optional<float> IN_Y;
    std::optional<float> IN_Z;
  };
  struct Response
  {
    std::string Command = "FRC_SetPayloadValue";
    int ErrorID;
  };
};

struct SetPayloadCompPacket
{
  struct Request
  {
    std::string Command = "FRC_SetPayloadComp";
    uint8_t ScheduleNumber;
    std::optional<uint8_t> Group;
    float Mass;
    float CG_X;
    float CG_Y;
    float CG_Z;
    float IN_X;
    float IN_Y;
    float IN_Z;
  };
  struct Response
  {
    std::string Command = "FRC_SetPayloadComp";
    int ErrorID;
  };
};

}  // namespace rmi

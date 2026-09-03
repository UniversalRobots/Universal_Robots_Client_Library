// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2026 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!
 * \file
 *
 * AUTO-GENERATED FILE — DO NOT EDIT MANUALLY.
 * Re-generate with:
 *   cmake --build <build-dir> --target generate_error_codes
 * or directly:
 *   scripts/generate_error_codes.py --output <this-file>
 *
 * Source: UR ErrorCodes JSON v40.121.0
 */
//----------------------------------------------------------------------

#pragma once

#include <cstdint>
#include <string_view>
#include <unordered_map>

namespace urcl
{
namespace primary_interface
{

/// Version of the UR ErrorCodes JSON this header was generated from.
constexpr std::string_view ERROR_CODE_JSON_VERSION = "40.121.0";

/// Returns a static map from packed (code, arg) keys to human-readable
/// error texts sourced from the UR ErrorCodes JSON.
///
/// Key encoding: upper 32 bits = error code, lower 32 bits = argument.
/// Entries without an argument in the source JSON use 0xFFFFFFFF as the
/// lower 32 bits (matches message_argument_ == -1 when cast to uint32_t).
inline const std::unordered_map<uint64_t, const char*>& getErrorCodeTexts()
{
  // clang-format off
  static const std::unordered_map<uint64_t, const char*> error_code_map = {
    { UINT64_C(0x00000000FFFFFFFF), "No error" },  // C0 no arg
    { UINT64_C(0x0000000100000001), "Outbuffer overflow: Buffer of stored warnings overflowed" },  // C1 arg=1
    { UINT64_C(0x0000000100000002), "Outbuffer overflow: Outbuffer to RS485 overflowed (problem with Controller message)" },  // C1 arg=2
    { UINT64_C(0x00000002FFFFFFFF), "Inbuffer overflow" },  // C2 no arg
    { UINT64_C(0x00000003FFFFFFFF), "Processor overloaded" },  // C3 no arg
    { UINT64_C(0x0000000400000001), "Communication issue: Lost communication with Controller" },  // C4 arg=1
    { UINT64_C(0x0000000400000002), "Communication issue: Lost communication with Safety Control Board A uP" },  // C4 arg=2
    { UINT64_C(0x0000000400000003), "Communication issue: Communication with Safety Control Board B uP lost" },  // C4 arg=3
    { UINT64_C(0x0000000400000004), "Communication issue: Communication with primary Teach Pendant uP lost" },  // C4 arg=4
    { UINT64_C(0x0000000400000005), "Communication issue: Communication with secondary Teach Pendant uP lost" },  // C4 arg=5
    { UINT64_C(0x0000000400000006), "Communication issue: Communication with primary EUROMAP67 uP lost" },  // C4 arg=6
    { UINT64_C(0x0000000400000007), "Communication issue: Communication with secondary EUROMAP67 uP lost" },  // C4 arg=7
    { UINT64_C(0x0000000400000008), "Communication issue: Primary EUROMAP67 uP present, but euromap67 is disabled" },  // C4 arg=8
    { UINT64_C(0x0000000400000009), "Communication issue: Secondary EUROMAP67 uP present, but euromap67 is disabled" },  // C4 arg=9
    { UINT64_C(0x000000040000000A), "Communication issue: Primary Teach Pendant present, but Teach Pendant safety is disabled" },  // C4 arg=10
    { UINT64_C(0x000000040000000B), "Communication issue: Secondary Teach Pendant uP present, Teach Pendant safety is disabled" },  // C4 arg=11
    { UINT64_C(0x000000040000000C), "Communication issue: Communication with joint 0 lost" },  // C4 arg=12
    { UINT64_C(0x000000040000000D), "Communication issue: Communication with joint 1 lost" },  // C4 arg=13
    { UINT64_C(0x000000040000000E), "Communication issue: Communication with joint 2 lost" },  // C4 arg=14
    { UINT64_C(0x000000040000000F), "Communication issue: Communication with joint 3 lost" },  // C4 arg=15
    { UINT64_C(0x0000000400000010), "Communication issue: Communication with joint 4 lost" },  // C4 arg=16
    { UINT64_C(0x0000000400000011), "Communication issue: Communication with joint 5 lost" },  // C4 arg=17
    { UINT64_C(0x0000000400000012), "Communication issue: Communication with tool lost" },  // C4 arg=18
    { UINT64_C(0x0000000400000041), "Communication issue: Lost package from Primary Teach Pendant" },  // C4 arg=65
    { UINT64_C(0x0000000400000042), "Communication issue: Lost package from Secondary Teach Pendant" },  // C4 arg=66
    { UINT64_C(0x0000000400000043), "Communication issue: Lost package from Primary Euromap67" },  // C4 arg=67
    { UINT64_C(0x0000000400000044), "Communication issue: Lost package from Secondary Euromap67" },  // C4 arg=68
    { UINT64_C(0x0000000400000045), "Communication issue: Lost package from Secondary Masterboard" },  // C4 arg=69
    { UINT64_C(0x0000000400000046), "Communication issue: Lost package from joint 0" },  // C4 arg=70
    { UINT64_C(0x0000000400000047), "Communication issue: Lost package from joint 1" },  // C4 arg=71
    { UINT64_C(0x0000000400000048), "Communication issue: Lost package from joint 2" },  // C4 arg=72
    { UINT64_C(0x0000000400000049), "Communication issue: Lost package from joint 3" },  // C4 arg=73
    { UINT64_C(0x000000040000004A), "Communication issue: Lost package from joint 4" },  // C4 arg=74
    { UINT64_C(0x000000040000004B), "Communication issue: Lost package from joint 5" },  // C4 arg=75
    { UINT64_C(0x000000040000004C), "Communication issue: Lost package from tool" },  // C4 arg=76
    { UINT64_C(0x000000040000004D), "Communication issue: Lost package from uPA to joints" },  // C4 arg=77
    { UINT64_C(0x000000040000004E), "Communication issue: Lost package from uPA to teach pendant" },  // C4 arg=78
    { UINT64_C(0x000000040000004F), "Communication issue: Lost package from uPA to uPB" },  // C4 arg=79
    { UINT64_C(0x0000000400000050), "Communication issue: Lost package from uPB" },  // C4 arg=80
    { UINT64_C(0x0000000400000051), "Communication issue: Packet counter disagreement in packet from Primary Screen" },  // C4 arg=81
    { UINT64_C(0x0000000400000052), "Communication issue: Packet counter disagreement in packet from Secondary Screen" },  // C4 arg=82
    { UINT64_C(0x0000000400000053), "Communication issue: Packet counter disagreement in packet from Primary Euromap67" },  // C4 arg=83
    { UINT64_C(0x0000000400000054), "Communication issue: Packet counter disagreement in packet from Secondary Euromap67" },  // C4 arg=84
    { UINT64_C(0x0000000400000055), "Communication issue: Packet counter disagreement in packet from Safety Control Board B" },  // C4 arg=85
    { UINT64_C(0x0000000400000056), "Communication issue: Packet counter disagreement in packet from joint 0" },  // C4 arg=86
    { UINT64_C(0x0000000400000057), "Communication issue: Packet counter disagreement in packet from joint 1" },  // C4 arg=87
    { UINT64_C(0x0000000400000058), "Communication issue: Packet counter disagreement in packet from joint 2" },  // C4 arg=88
    { UINT64_C(0x0000000400000059), "Communication issue: Packet counter disagreement in packet from joint 3" },  // C4 arg=89
    { UINT64_C(0x000000040000005A), "Communication issue: Packet counter disagreement in packet from joint 4" },  // C4 arg=90
    { UINT64_C(0x000000040000005B), "Communication issue: Packet counter disagreement in packet from joint 5" },  // C4 arg=91
    { UINT64_C(0x000000040000005C), "Communication issue: Packet counter disagreement in packet from tool" },  // C4 arg=92
    { UINT64_C(0x000000040000005D), "Communication issue: Packet counter disagreement in packet from processor A to joints" },  // C4 arg=93
    { UINT64_C(0x000000040000005E), "Communication issue: Packet counter disagreement in packet from processor A to B" },  // C4 arg=94
    { UINT64_C(0x000000040000005F), "Communication issue: Packet counter disagreement in packet from processor A to Teach Pendant and EUROMAP" },  // C4 arg=95
    { UINT64_C(0x0000000400000064), "Communication issue: Communication lost due to Packet counter disagreements" },  // C4 arg=100
    { UINT64_C(0x00000005FFFFFFFF), "Heavy processor load warning" },  // C5 no arg
    { UINT64_C(0x0000000A00000001), "Controller communication issue: Lost packet from Controller" },  // C10 arg=1
    { UINT64_C(0x0000000A00000065), "Controller communication issue: Controller packet received too early" },  // C10 arg=101
    { UINT64_C(0x0000000A00000066), "Controller communication issue: Packet counter does not match" },  // C10 arg=102
    { UINT64_C(0x0000000A00000067), "Controller communication issue: Controller is sending packets too often" },  // C10 arg=103
    { UINT64_C(0x0000000BFFFFFFFF), "Bad CRC" },  // C11 no arg
    { UINT64_C(0x0000000CFFFFFFFF), "Unknown message error" },  // C12 no arg
    { UINT64_C(0x0000000E00000001), "Debug message: {float}" },  // C14 arg=1
    { UINT64_C(0x0000000E00000002), "Debug message: {signed}" },  // C14 arg=2
    { UINT64_C(0x0000000E00000003), "Debug message: {unsigned}" },  // C14 arg=3
    { UINT64_C(0x00000011FFFFFFFF), "Communication error between Safety Control Board and Motherboard" },  // C17 no arg
    { UINT64_C(0x00000019FFFFFFFF), "Motor Encoder index missing" },  // C25 no arg
    { UINT64_C(0x0000001AFFFFFFFF), "Motor Encoder index drift detected" },  // C26 no arg
    { UINT64_C(0x0000001BFFFFFFFF), "Calibration data is invalid or does not exist, selftest is needed!" },  // C27 no arg
    { UINT64_C(0x0000001DFFFFFFFF), "Online Calibration data checksum failed" },  // C29 no arg
    { UINT64_C(0x0000001EFFFFFFFF), "Master received data from too many joints" },  // C30 no arg
    { UINT64_C(0x0000001FFFFFFFFF), "Caught wrong message (not from master)" },  // C31 no arg
    { UINT64_C(0x00000020FFFFFFFF), "Flash write verify failed" },  // C32 no arg
    { UINT64_C(0x00000021FFFFFFFF), "Calibration flash checksum failed" },  // C33 no arg
    { UINT64_C(0x0000002200000000), "Program flash checksum failed: Program flash checksum failed during bootloading" },  // C34 arg=0
    { UINT64_C(0x0000002200000001), "Program flash checksum failed: Program flash checksum failed at runtime" },  // C34 arg=1
    { UINT64_C(0x00000023FFFFFFFF), "Joint ID is undefined" },  // C35 no arg
    { UINT64_C(0x00000024FFFFFFFF), "Illegal bootloader command" },  // C36 no arg
    { UINT64_C(0x00000025FFFFFFFF), "Inbuffer parse error" },  // C37 no arg
    { UINT64_C(0x0000002600000001), "Online RAM test failed: Data-bus test failed" },  // C38 arg=1
    { UINT64_C(0x0000002600000002), "Online RAM test failed: Address-bus stuck-high test failed" },  // C38 arg=2
    { UINT64_C(0x0000002600000003), "Online RAM test failed: Address-bus stuck-low test failed" },  // C38 arg=3
    { UINT64_C(0x0000002600000004), "Online RAM test failed: Address-bus shorted test failed" },  // C38 arg=4
    { UINT64_C(0x0000002600000005), "Online RAM test failed: Memory-cell test failed" },  // C38 arg=5
    { UINT64_C(0x0000002700000001), "Logic and Temporal Monitoring Fault: Max current deviation failure" },  // C39 arg=1
    { UINT64_C(0x0000002700000002), "Logic and Temporal Monitoring Fault: Max joint-encoder speed exceeded" },  // C39 arg=2
    { UINT64_C(0x0000002700000003), "Logic and Temporal Monitoring Fault: Max motor-encoder speed exceeded" },  // C39 arg=3
    { UINT64_C(0x0000002700000004), "Logic and Temporal Monitoring Fault: Illegal state change in joint detected" },  // C39 arg=4
    { UINT64_C(0x0000002700000005), "Logic and Temporal Monitoring Fault: A timing issue occurred during startup." },  // C39 arg=5
    { UINT64_C(0x0000002700000006), "Logic and Temporal Monitoring Fault: 5V regulator voltage too low" },  // C39 arg=6
    { UINT64_C(0x0000002700000007), "Logic and Temporal Monitoring Fault: 5V regulator voltage too high" },  // C39 arg=7
    { UINT64_C(0x0000002700000064), "Logic and Temporal Monitoring Fault: Watchpoint fault: ADC task timeout" },  // C39 arg=100
    { UINT64_C(0x0000002700000065), "Logic and Temporal Monitoring Fault: Watchpoint fault: Motor-Control task timeout" },  // C39 arg=101
    { UINT64_C(0x0000002700000066), "Logic and Temporal Monitoring Fault: Watchpoint fault: Motor-encoder task timeout" },  // C39 arg=102
    { UINT64_C(0x0000002700000067), "Logic and Temporal Monitoring Fault: Watchpoint fault: Joint-encoder task timeout" },  // C39 arg=103
    { UINT64_C(0x0000002700000068), "Logic and Temporal Monitoring Fault: Watchpoint fault: Communication task timeout" },  // C39 arg=104
    { UINT64_C(0x0000002700000069), "Logic and Temporal Monitoring Fault: Watchpoint fault: RAM-test task timeout" },  // C39 arg=105
    { UINT64_C(0x000000270000006A), "Logic and Temporal Monitoring Fault: Watchpoint fault: CalVal-test task timeout" },  // C39 arg=106
    { UINT64_C(0x000000270000006B), "Logic and Temporal Monitoring Fault: Watchpoint fault: ROM-test task timeout" },  // C39 arg=107
    { UINT64_C(0x00000028FFFFFFFF), "AD-Converter hit high limit joint" },  // C40 no arg
    { UINT64_C(0x00000029FFFFFFFF), "RC Oscillator Trim register hit high limit" },  // C41 no arg
    { UINT64_C(0x0000002AFFFFFFFF), "RC Oscillator Trim register hit low limit" },  // C42 no arg
    { UINT64_C(0x0000002B00000001), "Change in invariant memory detected: Current sensor gain" },  // C43 arg=1
    { UINT64_C(0x0000002C00000000), "CRC check failure on primary bus: Base" },  // C44 arg=0
    { UINT64_C(0x0000002C00000001), "CRC check failure on primary bus: Shoulder" },  // C44 arg=1
    { UINT64_C(0x0000002C00000002), "CRC check failure on primary bus: Elbow" },  // C44 arg=2
    { UINT64_C(0x0000002C00000003), "CRC check failure on primary bus: Wrist 1" },  // C44 arg=3
    { UINT64_C(0x0000002C00000004), "CRC check failure on primary bus: Wrist 2" },  // C44 arg=4
    { UINT64_C(0x0000002C00000005), "CRC check failure on primary bus: Wrist 3" },  // C44 arg=5
    { UINT64_C(0x0000002C00000006), "CRC check failure on primary bus: Tool" },  // C44 arg=6
    { UINT64_C(0x0000002C00000050), "CRC check failure on primary bus: CRC Check failure on primary bus." },  // C44 arg=80
    { UINT64_C(0x0000002DFFFFFFFF), "AD-Converter error" },  // C45 no arg
    { UINT64_C(0x0000002EFFFFFFFF), "Loose gearbox or bad encoder mounting" },  // C46 no arg
    { UINT64_C(0x0000002FFFFFFFFF), "AD-Converter hit low limit" },  // C47 no arg
    { UINT64_C(0x00000031000000C8), "RS485 receive warning: Secondary RS485 bus is down" },  // C49 arg=200
    { UINT64_C(0x0000003200000001), "Robot powerup issue: Voltage detected at 24V rail before startup" },  // C50 arg=1
    { UINT64_C(0x0000003200000002), "Robot powerup issue: Voltage present at unpowered robot" },  // C50 arg=2
    { UINT64_C(0x0000003200000005), "Robot powerup issue: Powersupply voltage too low" },  // C50 arg=5
    { UINT64_C(0x0000003200000006), "Robot powerup issue: Powersupply voltage too high" },  // C50 arg=6
    { UINT64_C(0x000000320000000B), "Robot powerup issue: Voltage not detected at 24V rail after startup" },  // C50 arg=11
    { UINT64_C(0x000000320000000F), "Robot powerup issue: Warning, waiting for SafetySYS2" },  // C50 arg=15
    { UINT64_C(0x0000003200000010), "Robot powerup issue: The Teach Pendant does not respond" },  // C50 arg=16
    { UINT64_C(0x0000003200000011), "Robot powerup issue: The Euromap67 interface does not respond" },  // C50 arg=17
    { UINT64_C(0x0000003200000012), "Robot powerup issue: Warning, waiting for SafetySYS1" },  // C50 arg=18
    { UINT64_C(0x0000003200000013), "Robot powerup issue: Warning, Waiting for a valid \"euromap67 activated\" status bit from secondary Safety Control Board" },  // C50 arg=19
    { UINT64_C(0x0000003200000014), "Robot powerup issue: 5V, 3V3 or ADC error (5V too high)" },  // C50 arg=20
    { UINT64_C(0x0000003200000015), "Robot powerup issue: 5V, 3V3 or ADC error (5V too low)" },  // C50 arg=21
    { UINT64_C(0x0000003200000016), "Robot powerup issue: Robot current sensor reading too high" },  // C50 arg=22
    { UINT64_C(0x0000003200000017), "Robot powerup issue: Robot current sensor reading too low" },  // C50 arg=23
    { UINT64_C(0x0000003200000018), "Robot powerup issue: 48V not present (Check internal connection)" },  // C50 arg=24
    { UINT64_C(0x0000003200000019), "Robot powerup issue: Robot voltage present at 48V PSU powereup" },  // C50 arg=25
    { UINT64_C(0x000000320000001A), "Robot powerup issue: Voltage present on unpowered 48V power supply" },  // C50 arg=26
    { UINT64_C(0x000000320000001B), "Robot powerup issue: 12V, 3V3 or ADC error (12V too high)" },  // C50 arg=27
    { UINT64_C(0x000000320000001C), "Robot powerup issue: 12V, 3V3 or ADC error (12V too low)" },  // C50 arg=28
    { UINT64_C(0x000000320000001D), "Robot powerup issue: Analog I/O error (-12V too high)" },  // C50 arg=29
    { UINT64_C(0x000000320000001E), "Robot powerup issue: Analog I/O error (-12V too low)" },  // C50 arg=30
    { UINT64_C(0x000000320000001F), "Robot powerup issue: The other safetySYS do not initialize" },  // C50 arg=31
    { UINT64_C(0x0000003200000028), "Robot powerup issue: Wrong voltage from PSU1" },  // C50 arg=40
    { UINT64_C(0x0000003200000029), "Robot powerup issue: Wrong voltage from PSU2" },  // C50 arg=41
    { UINT64_C(0x000000320000002A), "Robot powerup issue: Voltage will not disappear from PSU" },  // C50 arg=42
    { UINT64_C(0x000000320000002B), "Robot powerup issue: Warning, waiting for CB2 type answer from primary processor" },  // C50 arg=43
    { UINT64_C(0x0000003200000032), "Robot powerup issue: Processor A 3.3V supply voltage out of bounds" },  // C50 arg=50
    { UINT64_C(0x0000003200000033), "Robot powerup issue: Robot voltage below threshold" },  // C50 arg=51
    { UINT64_C(0x0000003200000034), "Robot powerup issue: Robot voltage above threshold" },  // C50 arg=52
    { UINT64_C(0x0000003200000035), "Robot powerup issue: 58V generator deviation error" },  // C50 arg=53
    { UINT64_C(0x0000003200000036), "Robot powerup issue: 5V regulator too low" },  // C50 arg=54
    { UINT64_C(0x0000003200000037), "Robot powerup issue: 5V regulator too high" },  // C50 arg=55
    { UINT64_C(0x0000003200000038), "Robot powerup issue: -4V generator too low" },  // C50 arg=56
    { UINT64_C(0x0000003200000039), "Robot powerup issue: -4V generator too high" },  // C50 arg=57
    { UINT64_C(0x0000003200000050), "Robot powerup issue: Last CPU reset caused by Low-Power-Reset" },  // C50 arg=80
    { UINT64_C(0x0000003200000051), "Robot powerup issue: Last CPU reset caused by Window-Watchdog-Reset" },  // C50 arg=81
    { UINT64_C(0x0000003200000052), "Robot powerup issue: Last CPU reset caused by Independent-Watchdog-Reset" },  // C50 arg=82
    { UINT64_C(0x0000003200000053), "Robot powerup issue: Last CPU reset caused by Software-Reset" },  // C50 arg=83
    { UINT64_C(0x0000003200000054), "Robot powerup issue: Last CPU reset caused by External-Pin-Reset" },  // C50 arg=84
    { UINT64_C(0x0000003200000055), "Robot powerup issue: Last CPU reset caused by Brown-Out-Reset" },  // C50 arg=85
    { UINT64_C(0x0000003200000063), "Robot powerup issue: Wrong software on PCB" },  // C50 arg=99
    { UINT64_C(0x0000003200000064), "Robot powerup issue: Cable not connected" },  // C50 arg=100
    { UINT64_C(0x0000003200000065), "Robot powerup issue: Short circuit in robot detected or wrong robot connected to Control Box" },  // C50 arg=101
    { UINT64_C(0x0000003200000066), "Robot powerup issue: Voltage rising too slowly" },  // C50 arg=102
    { UINT64_C(0x0000003200000067), "Robot powerup issue: Voltage failed to reach acceptable level" },  // C50 arg=103
    { UINT64_C(0x0000003200000068), "Robot powerup issue: The IMMI module does not respond" },  // C50 arg=104
    { UINT64_C(0x0000003300000000), "CRC check failure on secondary bus: Processor B" },  // C51 arg=0
    { UINT64_C(0x0000003300000001), "CRC check failure on secondary bus: Primary screen processor" },  // C51 arg=1
    { UINT64_C(0x0000003300000002), "CRC check failure on secondary bus: Secondary screen processor" },  // C51 arg=2
    { UINT64_C(0x0000003300000003), "CRC check failure on secondary bus: Primary E67" },  // C51 arg=3
    { UINT64_C(0x0000003300000004), "CRC check failure on secondary bus: Secondary E67" },  // C51 arg=4
    { UINT64_C(0x0000003500000001), "IO overcurrent detected: , max is 800mA" },  // C53 arg=1
    { UINT64_C(0x0000003500000002), "IO overcurrent detected: , max is 600mA" },  // C53 arg=2
    { UINT64_C(0x0000003700000017), "Safety system error: Safety relay error (minus connection)" },  // C55 arg=23
    { UINT64_C(0x0000003700000018), "Safety system error: Safety relay error (plus connection)" },  // C55 arg=24
    { UINT64_C(0x0000003700000021), "Safety system error: Safety relay error (a relay is stuck)" },  // C55 arg=33
    { UINT64_C(0x0000003700000022), "Safety system error: Safety relay error (relays are not on)" },  // C55 arg=34
    { UINT64_C(0x0000003700000032), "Safety system error: Voltage present at unpowered robot" },  // C55 arg=50
    { UINT64_C(0x0000003700000033), "Safety system error: Voltage will not disappear from robot" },  // C55 arg=51
    { UINT64_C(0x0000003700000034), "Safety system error: 5V, 3V3 or ADC error (5V too low)" },  // C55 arg=52
    { UINT64_C(0x0000003700000035), "Safety system error: 5V, 3V3 or ADC error (5V too high)" },  // C55 arg=53
    { UINT64_C(0x000000370000005A), "Safety system error: Bootloader error, robot voltage too low or current too high" },  // C55 arg=90
    { UINT64_C(0x000000370000005B), "Safety system error: Bootloader error, robot voltage too high" },  // C55 arg=91
    { UINT64_C(0x0000003700000064), "Safety system error: Safety violation" },  // C55 arg=100
    { UINT64_C(0x0000003700000065), "Safety system error: Safety Channel Error In Safety Control Board" },  // C55 arg=101
    { UINT64_C(0x0000003700000066), "Safety system error: Safety Channel Error In Screen" },  // C55 arg=102
    { UINT64_C(0x0000003700000067), "Safety system error: Safety Channel Error In Euromap67 Interface" },  // C55 arg=103
    { UINT64_C(0x000000370000006D), "Safety system error: Received fault message from Controller" },  // C55 arg=109
    { UINT64_C(0x000000370000006E), "Safety system error: Safety State is changing too often" },  // C55 arg=110
    { UINT64_C(0x000000370000006F), "Safety system error: On/Off State is changing too often" },  // C55 arg=111
    { UINT64_C(0x0000003700000070), "Safety system error: Robot current sensors readings differ" },  // C55 arg=112
    { UINT64_C(0x0000003700000078), "Safety system error: Robot current is too high while emergency stopped" },  // C55 arg=120
    { UINT64_C(0x0000003700000079), "Safety system error: Robot current is too high while safeguard stopped" },  // C55 arg=121
    { UINT64_C(0x00000038FFFFFFFF), "Overvoltage shutdown" },  // C56 no arg
    { UINT64_C(0x0000003900000001), "Brake release failure: Joint did not move or motor encoder is not functioning" },  // C57 arg=1
    { UINT64_C(0x0000003900000002), "Brake release failure: Large movement detected during brake release" },  // C57 arg=2
    { UINT64_C(0x0000003900000003), "Brake release failure: Robot was not able to brake release, see log for details" },  // C57 arg=3
    { UINT64_C(0x0000003AFFFFFFFF), "Motor encoder not calibrated" },  // C58 no arg
    { UINT64_C(0x0000003BFFFFFFFF), "Overcurrent shutdown" },  // C59 no arg
    { UINT64_C(0x0000003CFFFFFFFF), "Energy surplus shutdown" },  // C60 no arg
    { UINT64_C(0x0000003DFFFFFFFF), "Idle power consumption to high" },  // C61 no arg
    { UINT64_C(0x0000003E00000001), "Thermal issue: Joint temperature: High (80(C)" },  // C62 arg=1
    { UINT64_C(0x0000003E00000003), "Thermal issue: Warning: Static load too high" },  // C62 arg=3
    { UINT64_C(0x0000003E0000000B), "Thermal issue: Joint temperature: Shut down (85(C)" },  // C62 arg=11
    { UINT64_C(0x0000003E0000000D), "Thermal issue: Shutdown: Static load too high" },  // C62 arg=13
    { UINT64_C(0x0000003FFFFFFFFF), "Motor test failed in step {unsigned}." },  // C63 no arg
    { UINT64_C(0x00000041FFFFFFFF), "PSU voltage to high" },  // C65 no arg
    { UINT64_C(0x00000044FFFFFFFF), "SPI error" },  // C68 no arg
    { UINT64_C(0x00000046FFFFFFFF), "Close to gearbox shear limit" },  // C70 no arg
    { UINT64_C(0x0000004700000000), "Startup check error: Hardware is size0, wrong firmware at the joint" },  // C71 arg=0
    { UINT64_C(0x0000004700000001), "Startup check error: Hardware is size1, wrong firmware at the joint" },  // C71 arg=1
    { UINT64_C(0x0000004700000002), "Startup check error: Hardware is size2, wrong firmware at the joint" },  // C71 arg=2
    { UINT64_C(0x0000004700000003), "Startup check error: Hardware is size3, wrong firmware at the joint" },  // C71 arg=3
    { UINT64_C(0x0000004700000004), "Startup check error: Hardware is size4, wrong firmware at the joint" },  // C71 arg=4
    { UINT64_C(0x0000004700000005), "Startup check error: Invalid hardware revision" },  // C71 arg=5
    { UINT64_C(0x0000004700000006), "Startup check error: ADC calibration failed" },  // C71 arg=6
    { UINT64_C(0x0000004700000007), "Startup check error: Unknown error result" },  // C71 arg=7
    { UINT64_C(0x0000004700000008), "Startup check error: Motor short circuit to ground or H-bridge problems" },  // C71 arg=8
    { UINT64_C(0x0000004700000009), "Startup check error: Motor indication signal does not work" },  // C71 arg=9
    { UINT64_C(0x000000470000000A), "Startup check error: Phase 1 is unconnected or not working" },  // C71 arg=10
    { UINT64_C(0x000000470000000B), "Startup check error: Phase 2 is unconnected or not working" },  // C71 arg=11
    { UINT64_C(0x000000470000000C), "Startup check error: Phase 3 or multiple phases is unconnected or not working" },  // C71 arg=12
    { UINT64_C(0x0000004700000032), "Startup check error: Current sensor test failed" },  // C71 arg=50
    { UINT64_C(0x0000004700000033), "Startup check error: Current sensor test failed" },  // C71 arg=51
    { UINT64_C(0x0000004700000034), "Startup check error: Current sensor test failed" },  // C71 arg=52
    { UINT64_C(0x0000004700000065), "Startup check error: Wrong firmware on motor encoder" },  // C71 arg=101
    { UINT64_C(0x0000004800000001), "Power Supply Unit failure: 0 PSUs are active" },  // C72 arg=1
    { UINT64_C(0x0000004800000002), "Power Supply Unit failure: 1 PSU active, but we expect 2 (UR10)" },  // C72 arg=2
    { UINT64_C(0x0000004800000003), "Power Supply Unit failure: 2 PSUs active, but we expect 1 (UR5)" },  // C72 arg=3
    { UINT64_C(0x00000049FFFFFFFF), "Brake test failed during selftest, check brakepin" },  // C73 no arg
    { UINT64_C(0x0000004A00000001), "Joint encoder warning: Invalid decode: Readhead misalignment, ring damaged or external magnetic field present." },  // C74 arg=1
    { UINT64_C(0x0000004A00000002), "Joint encoder warning: Speed reading is not valid" },  // C74 arg=2
    { UINT64_C(0x0000004A00000004), "Joint encoder warning: System error=malfunction or inconsistent calibration detected" },  // C74 arg=4
    { UINT64_C(0x0000004A00000008), "Joint encoder warning: Supply voltage is out of range" },  // C74 arg=8
    { UINT64_C(0x0000004A00000010), "Joint encoder warning: Temperature is out of range" },  // C74 arg=16
    { UINT64_C(0x0000004A00000020), "Joint encoder warning: Signal lost =Misaligned readhead or damaged ring" },  // C74 arg=32
    { UINT64_C(0x0000004A00000040), "Joint encoder warning: Signal low =Too far from magnetic ring" },  // C74 arg=64
    { UINT64_C(0x0000004A00000080), "Joint encoder warning: Signal saturation =Too close to magnetic ring" },  // C74 arg=128
    { UINT64_C(0x0000004B00000001), "Joint encoder error: Invalid decode: Readhead misalignment, ring damaged or external magnetic field present." },  // C75 arg=1
    { UINT64_C(0x0000004B00000002), "Joint encoder error: Speed reading is not valid" },  // C75 arg=2
    { UINT64_C(0x0000004B00000004), "Joint encoder error: System error=malfunction or inconsistent calibration detected" },  // C75 arg=4
    { UINT64_C(0x0000004B00000008), "Joint encoder error: Supply voltage is out of range" },  // C75 arg=8
    { UINT64_C(0x0000004B00000010), "Joint encoder error: Temperature is out of range" },  // C75 arg=16
    { UINT64_C(0x0000004B00000020), "Joint encoder error: Signal lost =Misaligned readhead or damaged ring" },  // C75 arg=32
    { UINT64_C(0x0000004B00000040), "Joint encoder error: Signal low =Too far from magnetic ring" },  // C75 arg=64
    { UINT64_C(0x0000004B00000080), "Joint encoder error: Signal saturation =Too close to magnetic ring" },  // C75 arg=128
    { UINT64_C(0x0000004B000000C8), "Joint encoder error: Position from joint encoder does not change while motor is running" },  // C75 arg=200
    { UINT64_C(0x0000004CFFFFFFFF), "Joint encoder communication CRC issue" },  // C76 no arg
    { UINT64_C(0x0000004DFFFFFFFF), "Sudden position change detected on the joint-encoder" },  // C77 no arg
    { UINT64_C(0x0000004EFFFFFFFF), "Large sudden position change detected on the joint-encoder" },  // C78 no arg
    { UINT64_C(0x00000055000000C8), "Motor encoder error: Position from motor encoder does not change while motor is running" },  // C85 arg=200
    { UINT64_C(0x00000064FFFFFFFF), "Robot changed mode" },  // C100 no arg
    { UINT64_C(0x00000065FFFFFFFF), "Real Robot Connected" },  // C101 no arg
    { UINT64_C(0x00000066FFFFFFFF), "Real Robot not connected - Simulating Robot" },  // C102 no arg
    { UINT64_C(0x0000006700000001), "Communication issue: Connection to Safety Control Board lost" },  // C103 arg=1
    { UINT64_C(0x0000006700000002), "Communication issue: Package lost from Safety Control Board" },  // C103 arg=2
    { UINT64_C(0x0000006700000003), "Communication issue: Ethernet connection initialization with Safety Control Board failed" },  // C103 arg=3
    { UINT64_C(0x00000068FFFFFFFF), "Error=Empty command sent to robot" },  // C104 no arg
    { UINT64_C(0x0000006FFFFFFFFF), "Something is pulling the robot" },  // C111 no arg
    { UINT64_C(0x00000073FFFFFFFF), "Unknown robot type" },  // C115 no arg
    { UINT64_C(0x00000074FFFFFFFF), "Realtime part warning" },  // C116 no arg
    { UINT64_C(0x00000075FFFFFFFF), "Restart SCB failed" },  // C117 no arg
    { UINT64_C(0x00000096FFFFFFFF), "Position close to joint limits" },  // C150 no arg
    { UINT64_C(0x00000097FFFFFFFF), "Tool orientation close to limits" },  // C151 no arg
    { UINT64_C(0x00000098FFFFFFFF), "Position close to safety plane limits" },  // C152 no arg
    { UINT64_C(0x0000009900000000), "Position deviates from path: Detected by the Base joint." },  // C153 arg=0
    { UINT64_C(0x0000009900000001), "Position deviates from path: Detected by the Shoulder joint." },  // C153 arg=1
    { UINT64_C(0x0000009900000002), "Position deviates from path: Detected by the Elbow joint." },  // C153 arg=2
    { UINT64_C(0x0000009900000003), "Position deviates from path: Detected by the Wrist 1 joint." },  // C153 arg=3
    { UINT64_C(0x0000009900000004), "Position deviates from path: Detected by the Wrist 2 joint." },  // C153 arg=4
    { UINT64_C(0x0000009900000005), "Position deviates from path: Detected by the Wrist 3 joint." },  // C153 arg=5
    { UINT64_C(0x0000009AFFFFFFFF), "Position in singularity" },  // C154 no arg
    { UINT64_C(0x0000009BFFFFFFFF), "Robot cannot maintain its position, check if payload is correct" },  // C155 no arg
    { UINT64_C(0x0000009CFFFFFFFF), "Wrong payload or mounting detected, or something is pushing the robot when entering Freedrive mode" },  // C156 no arg
    { UINT64_C(0x0000009D00000000), "Collision detected by joint: Detected by the Base joint." },  // C157 arg=0
    { UINT64_C(0x0000009D00000001), "Collision detected by joint: Detected by the Shoulder joint." },  // C157 arg=1
    { UINT64_C(0x0000009D00000002), "Collision detected by joint: Detected by the Elbow joint." },  // C157 arg=2
    { UINT64_C(0x0000009D00000003), "Collision detected by joint: Detected by the Wrist 1 joint." },  // C157 arg=3
    { UINT64_C(0x0000009D00000004), "Collision detected by joint: Detected by the Wrist 2 joint." },  // C157 arg=4
    { UINT64_C(0x0000009D00000005), "Collision detected by joint: Detected by the Wrist 3 joint." },  // C157 arg=5
    { UINT64_C(0x0000009E00000000), "Collision detected by joint: Base. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=0
    { UINT64_C(0x0000009E00000001), "Collision detected by joint: Shoulder. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=1
    { UINT64_C(0x0000009E00000002), "Collision detected by joint: Elbow. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=2
    { UINT64_C(0x0000009E00000003), "Collision detected by joint: Wrist 1. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=3
    { UINT64_C(0x0000009E00000004), "Collision detected by joint: Wrist 2. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=4
    { UINT64_C(0x0000009E00000005), "Collision detected by joint: Wrist 3. The user specified payload is 0kg, please make sure this is correct." },  // C158 arg=5
    { UINT64_C(0x0000009F00000000), "Position deviates from path: Base. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=0
    { UINT64_C(0x0000009F00000001), "Position deviates from path: Shoulder. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=1
    { UINT64_C(0x0000009F00000002), "Position deviates from path: Elbow. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=2
    { UINT64_C(0x0000009F00000003), "Position deviates from path: Wrist 1. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=3
    { UINT64_C(0x0000009F00000004), "Position deviates from path: Wrist 2. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=4
    { UINT64_C(0x0000009F00000005), "Position deviates from path: Wrist 3. The user specified payload is 0kg, please make sure this is correct." },  // C159 arg=5
    { UINT64_C(0x000000A0FFFFFFFF), "The robot was powered off last time due to a joint position disagreement" },  // C160 no arg
    { UINT64_C(0x000000A1FFFFFFFF), "Large movement of the robot detected while it was powered off. The joints were moved while it was powered off, or the encoders do not function" },  // C161 no arg
    { UINT64_C(0x000000A2FFFFFFFF), "The protective stop was likely caused by incorrectly specified payload mass and/or center of gravity." },  // C162 no arg
    { UINT64_C(0x000000A300000000), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Base joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=0
    { UINT64_C(0x000000A300000001), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Shoulder joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=1
    { UINT64_C(0x000000A300000002), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Elbow joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=2
    { UINT64_C(0x000000A300000003), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Wrist 1 joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=3
    { UINT64_C(0x000000A300000004), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Wrist 2 joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=4
    { UINT64_C(0x000000A300000005), "More than 50 Protective Stops are detected on the same joint within 8 hours of operation.: Wrist 3 joint. Something is wrong in the application. Recurring protective stops should be resolved, ignoring it can void warranty." },  // C163 arg=5
    { UINT64_C(0x000000A400000000), "Wrist position close to safety plane limits: Wrist 1 is too close to safety plane" },  // C164 arg=0
    { UINT64_C(0x000000A400000001), "Wrist position close to safety plane limits: Wrist 2 is too close to safety plane" },  // C164 arg=1
    { UINT64_C(0x000000A400000002), "Wrist position close to safety plane limits: Wrist 3 is too close to safety plane" },  // C164 arg=2
    { UINT64_C(0x000000AB00000000), "Issue with blends: A MoveC-Waypoint were skipped due to a blend." },  // C171 arg=0
    { UINT64_C(0x000000AB00000001), "Issue with blends: Blend radius too small in a MoveC" },  // C171 arg=1
    { UINT64_C(0x000000AB00000003), "Issue with blends: A ServoC-Waypoint were skipped due to a blend." },  // C171 arg=3
    { UINT64_C(0x000000AB00000004), "Issue with blends: Overlapping Blends in a MoveJ, a Waypoint was skipped" },  // C171 arg=4
    { UINT64_C(0x000000AB00000005), "Issue with blends: Overlapping Blends in a MoveJ, a Waypoint was skipped" },  // C171 arg=5
    { UINT64_C(0x000000AB00000006), "Issue with blends: Overlapping Blends in a MoveJ, a Waypoint was skipped" },  // C171 arg=6
    { UINT64_C(0x000000AB00000007), "Issue with blends: Overlapping Blends in a MoveJ, a Waypoint was skipped" },  // C171 arg=7
    { UINT64_C(0x000000AB00000009), "Issue with blends: A MoveP-Waypoint were skipped due to a blend." },  // C171 arg=9
    { UINT64_C(0x000000AB0000000A), "Issue with blends: Blend radius too small error in a MoveP" },  // C171 arg=10
    { UINT64_C(0x000000AB0000000B), "Issue with blends: Overlapping Blends in a MoveL, a Waypoint was skipped" },  // C171 arg=11
    { UINT64_C(0x000000AB0000000C), "Issue with blends: Overlapping Blends in a MoveL, a Waypoint was skipped" },  // C171 arg=12
    { UINT64_C(0x000000AB0000000D), "Issue with blends: Overlapping Blends in a MoveL, a Waypoint was skipped" },  // C171 arg=13
    { UINT64_C(0x000000AB0000000E), "Issue with blends: Overlapping Blends in a MoveL, a Waypoint was skipped" },  // C171 arg=14
    { UINT64_C(0x000000ACFFFFFFFF), "Illegal control mode" },  // C172 no arg
    { UINT64_C(0x000000AD00000000), "Robot motion causes too high joint torques: Base." },  // C173 arg=0
    { UINT64_C(0x000000AD00000001), "Robot motion causes too high joint torques: Shoulder." },  // C173 arg=1
    { UINT64_C(0x000000AD00000002), "Robot motion causes too high joint torques: Elbow." },  // C173 arg=2
    { UINT64_C(0x000000AD00000003), "Robot motion causes too high joint torques: Wrist 1." },  // C173 arg=3
    { UINT64_C(0x000000AD00000004), "Robot motion causes too high joint torques: Wrist 2." },  // C173 arg=4
    { UINT64_C(0x000000AD00000005), "Robot motion causes too high joint torques: Wrist 3." },  // C173 arg=5
    { UINT64_C(0x000000AD00000006), "Robot motion causes too high joint torques: Base. Problem identified when executing program line {unsigned}." },  // C173 arg=6
    { UINT64_C(0x000000AD00000007), "Robot motion causes too high joint torques: Shoulder. Problem identified when executing program line {unsigned}." },  // C173 arg=7
    { UINT64_C(0x000000AD00000008), "Robot motion causes too high joint torques: Elbow. Problem identified when executing program line {unsigned}." },  // C173 arg=8
    { UINT64_C(0x000000AD00000009), "Robot motion causes too high joint torques: Wrist 1. Problem identified when executing program line {unsigned}." },  // C173 arg=9
    { UINT64_C(0x000000AD0000000A), "Robot motion causes too high joint torques: Wrist 2. Problem identified when executing program line {unsigned}." },  // C173 arg=10
    { UINT64_C(0x000000AD0000000B), "Robot motion causes too high joint torques: Wrist 3. Problem identified when executing program line {unsigned}." },  // C173 arg=11
    { UINT64_C(0x000000AE00000000), "Robot motion causes too high jump in joint torques: Base." },  // C174 arg=0
    { UINT64_C(0x000000AE00000001), "Robot motion causes too high jump in joint torques: Shoulder." },  // C174 arg=1
    { UINT64_C(0x000000AE00000002), "Robot motion causes too high jump in joint torques: Elbow." },  // C174 arg=2
    { UINT64_C(0x000000AE00000003), "Robot motion causes too high jump in joint torques: Wrist 1." },  // C174 arg=3
    { UINT64_C(0x000000AE00000004), "Robot motion causes too high jump in joint torques: Wrist 2." },  // C174 arg=4
    { UINT64_C(0x000000AE00000005), "Robot motion causes too high jump in joint torques: Wrist 3." },  // C174 arg=5
    { UINT64_C(0x000000AE00000006), "Robot motion causes too high jump in joint torques: Base. Problem identified when executing program line {unsigned}." },  // C174 arg=6
    { UINT64_C(0x000000AE00000007), "Robot motion causes too high jump in joint torques: Shoulder. Problem identified when executing program line {unsigned}." },  // C174 arg=7
    { UINT64_C(0x000000AE00000008), "Robot motion causes too high jump in joint torques: Elbow. Problem identified when executing program line {unsigned}." },  // C174 arg=8
    { UINT64_C(0x000000AE00000009), "Robot motion causes too high jump in joint torques: Wrist 1. Problem identified when executing program line {unsigned}." },  // C174 arg=9
    { UINT64_C(0x000000AE0000000A), "Robot motion causes too high jump in joint torques: Wrist 2. Problem identified when executing program line {unsigned}." },  // C174 arg=10
    { UINT64_C(0x000000AE0000000B), "Robot motion causes too high jump in joint torques: Wrist 3. Problem identified when executing program line {unsigned}." },  // C174 arg=11
    { UINT64_C(0x000000B8FFFFFFFF), "Joint self test not received by controller" },  // C184 no arg
    { UINT64_C(0x000000B900000001), "START_NORMAL_OPERATION is not allowed on selftest firmware" },  // C185 arg=1
    { UINT64_C(0x000000B900000002), "GOTO_BACKDRIVE_COMMAND is not allowed on selftest firmware" },  // C185 arg=2
    { UINT64_C(0x000000BA00000001), "joint_mode == JOINT_RUNNING_MODE is not allowed on selftest firmware" },  // C186 arg=1
    { UINT64_C(0x000000BB00000001), "Temperature sensor test failed: Starting temperature were lower than expected" },  // C187 arg=1
    { UINT64_C(0x000000BB00000002), "Temperature sensor test failed: Starting temperature were higher than expected" },  // C187 arg=2
    { UINT64_C(0x000000BB00000003), "Temperature sensor test failed: Temperature increased less than expected during warm up" },  // C187 arg=3
    { UINT64_C(0x000000BB00000004), "Temperature sensor test failed: Temperature increased more than expected during warm up" },  // C187 arg=4
    { UINT64_C(0x000000BE00000000), "Joint failed during selftest: Motor encoder index mark not found" },  // C190 arg=0
    { UINT64_C(0x000000BE00000001), "Joint failed during selftest: Phases not mounted correctly" },  // C190 arg=1
    { UINT64_C(0x000000BE00000002), "Joint failed during selftest: Motor encoder counting the wrong way" },  // C190 arg=2
    { UINT64_C(0x000000BE00000003), "Joint failed during selftest: Joint encoder counting the wrong way" },  // C190 arg=3
    { UINT64_C(0x000000BE00000004), "Joint failed during selftest: No movement detected while trying to move the motor" },  // C190 arg=4
    { UINT64_C(0x000000BE0000000B), "Joint failed during selftest: Temperature alignment did not warm up to 45 degrees C within 30 minutes" },  // C190 arg=11
    { UINT64_C(0x000000BE0000000C), "Joint failed during selftest: Temperature alignment did not cool down to 45 degrees C within 60 minutes" },  // C190 arg=12
    { UINT64_C(0x000000BF00000001), "Safety system violation: Joint position limit violated" },  // C191 arg=1
    { UINT64_C(0x000000BF00000002), "Safety system violation: Joint speed limit violated" },  // C191 arg=2
    { UINT64_C(0x000000BF00000003), "Safety system violation: TCP speed limit violated" },  // C191 arg=3
    { UINT64_C(0x000000BF00000004), "Safety system violation: TCP position limit violated" },  // C191 arg=4
    { UINT64_C(0x000000BF00000005), "Safety system violation: TCP orientation limit violated" },  // C191 arg=5
    { UINT64_C(0x000000BF00000006), "Safety system violation: Power limit violated" },  // C191 arg=6
    { UINT64_C(0x000000BF00000007), "Safety system violation: Joint torque window violated" },  // C191 arg=7
    { UINT64_C(0x000000BF00000008), "Safety system violation: Joint torque window too large" },  // C191 arg=8
    { UINT64_C(0x000000BF00000009), "Safety system violation: Reduced mode output violation" },  // C191 arg=9
    { UINT64_C(0x000000BF0000000A), "Safety system violation: Safeguard stop output violation" },  // C191 arg=10
    { UINT64_C(0x000000BF0000000B), "Safety system violation: Emergency stop output violation" },  // C191 arg=11
    { UINT64_C(0x000000BF0000000C), "Safety system violation: Momentum limit violation" },  // C191 arg=12
    { UINT64_C(0x000000BF0000000D), "Safety system violation: Robot moving output violation" },  // C191 arg=13
    { UINT64_C(0x000000BF0000000E), "Safety system violation: Robot is not braking in stop mode" },  // C191 arg=14
    { UINT64_C(0x000000BF0000000F), "Safety system violation: Robot is moving in stop mode" },  // C191 arg=15
    { UINT64_C(0x000000BF00000010), "Safety system violation: Robot did not stop in time" },  // C191 arg=16
    { UINT64_C(0x000000BF00000011), "Safety system violation: Received a null vector for TCP orientation" },  // C191 arg=17
    { UINT64_C(0x000000BF00000012), "Safety system violation: Robot not stopping output violation" },  // C191 arg=18
    { UINT64_C(0x000000BF00000013), "Safety system violation: Invalid safety IO configuration" },  // C191 arg=19
    { UINT64_C(0x000000BF00000014), "Safety system violation: Configuration information or limit sets not received" },  // C191 arg=20
    { UINT64_C(0x000000BF00000015), "Safety system violation: The other safety processor detected a violation" },  // C191 arg=21
    { UINT64_C(0x000000BF00000016), "Safety system violation: Received unknown command from Controller" },  // C191 arg=22
    { UINT64_C(0x000000BF00000017), "Safety system violation: Invalid setup of safety limits" },  // C191 arg=23
    { UINT64_C(0x000000BF00000018), "Safety system violation: Reduced Mode Output set, while it should not be" },  // C191 arg=24
    { UINT64_C(0x000000BF00000019), "Safety system violation: Reduced Mode Output not set, while it should be" },  // C191 arg=25
    { UINT64_C(0x000000BF0000001A), "Safety system violation: Not Reduced Mode Output set, while it should not be" },  // C191 arg=26
    { UINT64_C(0x000000BF0000001B), "Safety system violation: Not Reduced Mode Output not set, while it should be" },  // C191 arg=27
    { UINT64_C(0x000000BF0000001C), "Safety system violation: Robot Emergency Stop exceeded maximum stop time" },  // C191 arg=28
    { UINT64_C(0x000000BF0000001D), "Safety system violation: System Emergency Stop exceeded maximum stop time" },  // C191 arg=29
    { UINT64_C(0x000000BF0000001E), "Safety system violation: Safeguard Stop exceeded maximum stop time" },  // C191 arg=30
    { UINT64_C(0x000000BF0000001F), "Safety system violation: Operation mode switch is present while the three position switch is missing" },  // C191 arg=31
    { UINT64_C(0x000000BF00000020), "Safety system violation: Joint speed limit violated - Base" },  // C191 arg=32
    { UINT64_C(0x000000BF00000021), "Safety system violation: Joint speed limit violated - Shoulder" },  // C191 arg=33
    { UINT64_C(0x000000BF00000022), "Safety system violation: Joint speed limit violated - Elbow" },  // C191 arg=34
    { UINT64_C(0x000000BF00000023), "Safety system violation: Joint speed limit violated - Wrist 1" },  // C191 arg=35
    { UINT64_C(0x000000BF00000024), "Safety system violation: Joint speed limit violated - Wrist 2" },  // C191 arg=36
    { UINT64_C(0x000000BF00000025), "Safety system violation: Joint speed limit violated - Wrist 3" },  // C191 arg=37
    { UINT64_C(0x000000C000000001), "Safety system fault: Robot still powered in emergency stop" },  // C192 arg=1
    { UINT64_C(0x000000C000000002), "Safety system fault: Robot emergency stop disagreement" },  // C192 arg=2
    { UINT64_C(0x000000C000000003), "Safety system fault: System emergency stop disagreement" },  // C192 arg=3
    { UINT64_C(0x000000C000000004), "Safety system fault: Safeguard stop disagreement" },  // C192 arg=4
    { UINT64_C(0x000000C000000005), "Safety system fault: Euromap safeguard stop disagreement" },  // C192 arg=5
    { UINT64_C(0x000000C000000006), "Safety system fault: Joint position disagreement" },  // C192 arg=6
    { UINT64_C(0x000000C000000007), "Safety system fault: Joint speed disagreement" },  // C192 arg=7
    { UINT64_C(0x000000C000000008), "Safety system fault: Joint torque disagreement" },  // C192 arg=8
    { UINT64_C(0x000000C000000009), "Safety system fault: TCP speed disagreement" },  // C192 arg=9
    { UINT64_C(0x000000C00000000A), "Safety system fault: TCP position disagreement" },  // C192 arg=10
    { UINT64_C(0x000000C00000000B), "Safety system fault: TCP orientation disagreement" },  // C192 arg=11
    { UINT64_C(0x000000C00000000C), "Safety system fault: Power disagreement" },  // C192 arg=12
    { UINT64_C(0x000000C00000000D), "Safety system fault: Joint torque window disagreement" },  // C192 arg=13
    { UINT64_C(0x000000C00000000E), "Safety system fault: Reduced mode input disagreement" },  // C192 arg=14
    { UINT64_C(0x000000C00000000F), "Safety system fault: Reduced mode output disagreement" },  // C192 arg=15
    { UINT64_C(0x000000C000000010), "Safety system fault: Safety output failed" },  // C192 arg=16
    { UINT64_C(0x000000C000000011), "Safety system fault: Safeguard stop output disagreement" },  // C192 arg=17
    { UINT64_C(0x000000C000000012), "Safety system fault: The other safety processor is in fault" },  // C192 arg=18
    { UINT64_C(0x000000C000000013), "Safety system fault: Emergency stop output disagreement" },  // C192 arg=19
    { UINT64_C(0x000000C000000014), "Safety system fault: SPI output error detected" },  // C192 arg=20
    { UINT64_C(0x000000C000000015), "Safety system fault: Momentum disagreement" },  // C192 arg=21
    { UINT64_C(0x000000C000000016), "Safety system fault: Robot moving output disagreement" },  // C192 arg=22
    { UINT64_C(0x000000C000000017), "Safety system fault: Wrong processor ID" },  // C192 arg=23
    { UINT64_C(0x000000C000000018), "Safety system fault: Wrong processor revision" },  // C192 arg=24
    { UINT64_C(0x000000C000000019), "Safety system fault: Potential brownout detected" },  // C192 arg=25
    { UINT64_C(0x000000C00000001A), "Safety system fault: Emergency stop output disagreement" },  // C192 arg=26
    { UINT64_C(0x000000C00000001B), "Safety system fault: Safeguard stop output disagreement" },  // C192 arg=27
    { UINT64_C(0x000000C00000001C), "Safety system fault: Robot not stopping output disagreement" },  // C192 arg=28
    { UINT64_C(0x000000C00000001D), "Safety system fault: Safeguard reset input disagreement" },  // C192 arg=29
    { UINT64_C(0x000000C00000001E), "Safety system fault: Safety processor booted up in fault mode" },  // C192 arg=30
    { UINT64_C(0x000000C00000001F), "Safety system fault: Reduced Mode Output disagreement" },  // C192 arg=31
    { UINT64_C(0x000000C000000020), "Safety system fault: Not Reduced Mode Output disagreement" },  // C192 arg=32
    { UINT64_C(0x000000C000000021), "Safety system fault: A timing issue occurred during startup. Please restart to proceed" },  // C192 arg=33
    { UINT64_C(0x000000C000000022), "Safety system fault: User safety config checksum disagreement between uA and GUI" },  // C192 arg=34
    { UINT64_C(0x000000C000000023), "Safety system fault: Robot config checksum disagreement between uA and GUI" },  // C192 arg=35
    { UINT64_C(0x000000C000000024), "Safety system fault: Online RAM test failed" },  // C192 arg=36
    { UINT64_C(0x000000C000000025), "Safety system fault: Not all safety related functionalities are running" },  // C192 arg=37
    { UINT64_C(0x000000C000000026), "Safety system fault: Package too short for CRC calculation" },  // C192 arg=38
    { UINT64_C(0x000000C000000027), "Safety system fault: Three position switch input disagreement" },  // C192 arg=39
    { UINT64_C(0x000000C000000028), "Safety system fault: Operation mode switch input disagreement" },  // C192 arg=40
    { UINT64_C(0x000000C100000000), "One of the nodes is in fault mode: Base Joint" },  // C193 arg=0
    { UINT64_C(0x000000C100000001), "One of the nodes is in fault mode: Shoulder Joint" },  // C193 arg=1
    { UINT64_C(0x000000C100000002), "One of the nodes is in fault mode: Elbow Joint" },  // C193 arg=2
    { UINT64_C(0x000000C100000003), "One of the nodes is in fault mode: Wrist 1 Joint" },  // C193 arg=3
    { UINT64_C(0x000000C100000004), "One of the nodes is in fault mode: Wrist 2 Joint" },  // C193 arg=4
    { UINT64_C(0x000000C100000005), "One of the nodes is in fault mode: Wrist 3 Joint" },  // C193 arg=5
    { UINT64_C(0x000000C100000006), "One of the nodes is in fault mode: Tool" },  // C193 arg=6
    { UINT64_C(0x000000C100000007), "One of the nodes is in fault mode: Screen 1" },  // C193 arg=7
    { UINT64_C(0x000000C100000008), "One of the nodes is in fault mode: Screen 2" },  // C193 arg=8
    { UINT64_C(0x000000C100000009), "One of the nodes is in fault mode: Euromap 1" },  // C193 arg=9
    { UINT64_C(0x000000C10000000A), "One of the nodes is in fault mode: Euromap 2" },  // C193 arg=10
    { UINT64_C(0x000000C200000000), "One of the nodes is not booted or not present: Base Joint" },  // C194 arg=0
    { UINT64_C(0x000000C200000001), "One of the nodes is not booted or not present: Shoulder Joint" },  // C194 arg=1
    { UINT64_C(0x000000C200000002), "One of the nodes is not booted or not present: Elbow Joint" },  // C194 arg=2
    { UINT64_C(0x000000C200000003), "One of the nodes is not booted or not present: Wrist 1 Joint" },  // C194 arg=3
    { UINT64_C(0x000000C200000004), "One of the nodes is not booted or not present: Wrist 2 Joint" },  // C194 arg=4
    { UINT64_C(0x000000C200000005), "One of the nodes is not booted or not present: Wrist 3 Joint" },  // C194 arg=5
    { UINT64_C(0x000000C200000006), "One of the nodes is not booted or not present: Tool" },  // C194 arg=6
    { UINT64_C(0x000000C200000007), "One of the nodes is not booted or not present: Screen 1" },  // C194 arg=7
    { UINT64_C(0x000000C200000008), "One of the nodes is not booted or not present: Screen 2" },  // C194 arg=8
    { UINT64_C(0x000000C200000009), "One of the nodes is not booted or not present: Euromap 1" },  // C194 arg=9
    { UINT64_C(0x000000C20000000A), "One of the nodes is not booted or not present: Euromap 2" },  // C194 arg=10
    { UINT64_C(0x000000C200000080), "One of the nodes is not booted or not present: Base not ready while brake release requested" },  // C194 arg=128
    { UINT64_C(0x000000C200000081), "One of the nodes is not booted or not present: Shoulder not ready while brake release requested" },  // C194 arg=129
    { UINT64_C(0x000000C200000082), "One of the nodes is not booted or not present: Elbow not ready while brake release requested" },  // C194 arg=130
    { UINT64_C(0x000000C200000083), "One of the nodes is not booted or not present: Wrist 1 not ready while brake release requested" },  // C194 arg=131
    { UINT64_C(0x000000C200000084), "One of the nodes is not booted or not present: Wrist 2 not ready while brake release requested" },  // C194 arg=132
    { UINT64_C(0x000000C200000085), "One of the nodes is not booted or not present: Wrist 3 not ready while brake release requested" },  // C194 arg=133
    { UINT64_C(0x000000C200000086), "One of the nodes is not booted or not present: Tool not ready while brake release requested" },  // C194 arg=134
    { UINT64_C(0x000000C300000001), "Conveyor speed too high: for joint speed safety limit" },  // C195 arg=1
    { UINT64_C(0x000000C300000002), "Conveyor speed too high: for TCP speed safety limit" },  // C195 arg=2
    { UINT64_C(0x000000C300000003), "Conveyor speed too high: for momentum safety limit" },  // C195 arg=3
    { UINT64_C(0x000000C4FFFFFFFF), "MoveP speed too high" },  // C196 no arg
    { UINT64_C(0x000000C5FFFFFFFF), "Blend overlap warning" },  // C197 no arg
    { UINT64_C(0x000000C800000001), "Safety Control Board hardware error: Hardware ID is wrong" },  // C200 arg=1
    { UINT64_C(0x000000C800000002), "Safety Control Board hardware error: MCU type is wrong" },  // C200 arg=2
    { UINT64_C(0x000000C800000003), "Safety Control Board hardware error: Part ID is wrong" },  // C200 arg=3
    { UINT64_C(0x000000C800000004), "Safety Control Board hardware error: RAM test failed" },  // C200 arg=4
    { UINT64_C(0x000000C800000005), "Safety Control Board hardware error: Register test failed" },  // C200 arg=5
    { UINT64_C(0x000000C800000006), "Safety Control Board hardware error: pRom Crc test failed" },  // C200 arg=6
    { UINT64_C(0x000000C800000007), "Safety Control Board hardware error: Watchdog reset the processor" },  // C200 arg=7
    { UINT64_C(0x000000C800000008), "Safety Control Board hardware error: OVG signal test not passed" },  // C200 arg=8
    { UINT64_C(0x000000C800000009), "Safety Control Board hardware error: 3V3A power good pin is low" },  // C200 arg=9
    { UINT64_C(0x000000C80000000A), "Safety Control Board hardware error: 3V3B power good pin is low" },  // C200 arg=10
    { UINT64_C(0x000000C80000000B), "Safety Control Board hardware error: 5V power good is low" },  // C200 arg=11
    { UINT64_C(0x000000C80000000C), "Safety Control Board hardware error: 3V3 voltage too low" },  // C200 arg=12
    { UINT64_C(0x000000C80000000D), "Safety Control Board hardware error: 3v3 voltage too high" },  // C200 arg=13
    { UINT64_C(0x000000C80000000E), "Safety Control Board hardware error: 48V input is too low" },  // C200 arg=14
    { UINT64_C(0x000000C80000000F), "Safety Control Board hardware error: 48V input is too high" },  // C200 arg=15
    { UINT64_C(0x000000C800000010), "Safety Control Board hardware error: 24V IO short circuited" },  // C200 arg=16
    { UINT64_C(0x000000C800000011), "Safety Control Board hardware error: PC current is too high" },  // C200 arg=17
    { UINT64_C(0x000000C800000012), "Safety Control Board hardware error: Robot voltage is too low" },  // C200 arg=18
    { UINT64_C(0x000000C800000013), "Safety Control Board hardware error: Robot voltage is too high" },  // C200 arg=19
    { UINT64_C(0x000000C800000014), "Safety Control Board hardware error: 24V IO voltage is too low" },  // C200 arg=20
    { UINT64_C(0x000000C800000015), "Safety Control Board hardware error: 12V voltage is too high" },  // C200 arg=21
    { UINT64_C(0x000000C800000016), "Safety Control Board hardware error: 12V voltage is too low" },  // C200 arg=22
    { UINT64_C(0x000000C800000017), "Safety Control Board hardware error: It took too long to stabilize 24V" },  // C200 arg=23
    { UINT64_C(0x000000C800000018), "Safety Control Board hardware error: It took too long to stabilize 24V IO" },  // C200 arg=24
    { UINT64_C(0x000000C800000019), "Safety Control Board hardware error: 24V voltage is too high" },  // C200 arg=25
    { UINT64_C(0x000000C80000001A), "Safety Control Board hardware error: 24V IO voltage is too high" },  // C200 arg=26
    { UINT64_C(0x000000C900000000), "Setup of Safety Control Board failed: Setup of Safety Control Board failed" },  // C201 arg=0
    { UINT64_C(0x000000C900000001), "Setup of Safety Control Board failed: SCB uA is not responding" },  // C201 arg=1
    { UINT64_C(0x000000C900000002), "Setup of Safety Control Board failed: SCB uB is not responding" },  // C201 arg=2
    { UINT64_C(0x000000C900000003), "Setup of Safety Control Board failed: SCB is not responding" },  // C201 arg=3
    { UINT64_C(0x000000CAFFFFFFFF), "SCE configuration was illegal, after applying tolerances" },  // C202 no arg
    { UINT64_C(0x000000CBFFFFFFFF), "PolyScope detected a mismatch between the shown and (to be) applied safety parameters" },  // C203 no arg
    { UINT64_C(0x000000CC00000001), "Path sanity check failed: Sudden change in target position" },  // C204 arg=1
    { UINT64_C(0x000000CC00000002), "Path sanity check failed: Inconsistency between target position and speed" },  // C204 arg=2
    { UINT64_C(0x000000CC00000003), "Path sanity check failed: Sudden stop" },  // C204 arg=3
    { UINT64_C(0x000000CC00000004), "Path sanity check failed: Robot has not stopped in the allowed reaction and braking time" },  // C204 arg=4
    { UINT64_C(0x000000CC00000005), "Path sanity check failed: Robot program resulted in invalid setpoint" },  // C204 arg=5
    { UINT64_C(0x000000CC00000006), "Path sanity check failed: Blending failed and resulted in an invalid setpoint" },  // C204 arg=6
    { UINT64_C(0x000000CC00000007), "Path sanity check failed: Robot approaching singularity - Acceleration threshold failed" },  // C204 arg=7
    { UINT64_C(0x000000CD00000000), "Target speed does not match target position: Inconsistency between target position and speed" },  // C205 arg=0
    { UINT64_C(0x000000CE00000000), "Sanity check failed: Target joint speed does not match target joint position change - Base" },  // C206 arg=0
    { UINT64_C(0x000000CE00000001), "Sanity check failed: Target joint speed does not match target joint position change - Shoulder" },  // C206 arg=1
    { UINT64_C(0x000000CE00000002), "Sanity check failed: Target joint speed does not match target joint position change - Elbow" },  // C206 arg=2
    { UINT64_C(0x000000CE00000003), "Sanity check failed: Target joint speed does not match target joint position change - Wrist 1" },  // C206 arg=3
    { UINT64_C(0x000000CE00000004), "Sanity check failed: Target joint speed does not match target joint position change - Wrist 2" },  // C206 arg=4
    { UINT64_C(0x000000CE00000005), "Sanity check failed: Target joint speed does not match target joint position change - Wrist 3" },  // C206 arg=5
    { UINT64_C(0x000000CFFFFFFFFF), "Fieldbus input disconnected" },  // C207 no arg
    { UINT64_C(0x000000D0FFFFFFFF), "Debug Assertion failed" },  // C208 no arg
    { UINT64_C(0x000000D1FFFFFFFF), "A protective stop was triggered (for test purposes only)" },  // C209 no arg
    { UINT64_C(0x000000D2FFFFFFFF), "Socket is read-only when the robot is in local (Teach pendant) control" },  // C210 no arg
    { UINT64_C(0x000000D300000000), "Operational mode changed: Disabled" },  // C211 arg=0
    { UINT64_C(0x000000D300000001), "Operational mode changed: Automatic" },  // C211 arg=1
    { UINT64_C(0x000000D300000002), "Operational mode changed: Manual" },  // C211 arg=2
    { UINT64_C(0x000000D400000001), "Name conflict in loaded program: {unsigned} name conflict(s) occurred between feature names and program variables" },  // C212 arg=1
    { UINT64_C(0x000000D5FFFFFFFF), "No Kinematic Calibration found (calibration.conf file is either corrupt or missing)" },  // C213 no arg
    { UINT64_C(0x000000D600000001), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match the Base checksum" },  // C214 arg=1
    { UINT64_C(0x000000D600000002), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match the Shoulder checksum" },  // C214 arg=2
    { UINT64_C(0x000000D600000003), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match the Elbow checksum" },  // C214 arg=3
    { UINT64_C(0x000000D600000004), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match  Wrist 1 checksum" },  // C214 arg=4
    { UINT64_C(0x000000D600000005), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match for Wrist 2 checksum" },  // C214 arg=5
    { UINT64_C(0x000000D600000006), "Kinematic Calibration for the robot does not match the joint(s): The Kinematic Calibration checksum does not match for Wrist 3 checksum" },  // C214 arg=6
    { UINT64_C(0x000000D7FFFFFFFF), "Kinematic Calibration does not match the robot" },  // C215 no arg
    { UINT64_C(0x000000D800000001), "The offset of the joint has changed: Base" },  // C216 arg=1
    { UINT64_C(0x000000D800000002), "The offset of the joint has changed: Shoulder" },  // C216 arg=2
    { UINT64_C(0x000000D800000003), "The offset of the joint has changed: Elbow" },  // C216 arg=3
    { UINT64_C(0x000000D800000004), "The offset of the joint has changed: Wrist 1" },  // C216 arg=4
    { UINT64_C(0x000000D800000005), "The offset of the joint has changed: Wrist 2" },  // C216 arg=5
    { UINT64_C(0x000000D800000006), "The offset of the joint has changed: Wrist 3" },  // C216 arg=6
    { UINT64_C(0x000000D9FFFFFFFF), "White space detected at the beginning of a string token at line {unsigned}" },  // C217 no arg
    { UINT64_C(0x000000DA00000000), "A thread used a lot of time: Main Robot Program." },  // C218 arg=0
    { UINT64_C(0x000000DA00000001), "A thread used a lot of time: Thread: {string}" },  // C218 arg=1
    { UINT64_C(0x000000DB00000001), "Path Offset: Change in offset is too high to meet joint speed safety limit" },  // C219 arg=1
    { UINT64_C(0x000000DB00000002), "Path Offset: Change in offset is too high to meet tool speed safety limit" },  // C219 arg=2
    { UINT64_C(0x000000DB00000003), "Path Offset: Change in offset is too high to meet momentum safety limit" },  // C219 arg=3
    { UINT64_C(0x000000DC00000001), "Kinematic Calibration: Version {unsigned} on the robot arm is not supported" },  // C220 arg=1
    { UINT64_C(0x000000DC00000002), "Kinematic Calibration: Kinematic Calibration file was replaced with file from the arm." },  // C220 arg=2
    { UINT64_C(0x000000DC00000003), "Kinematic Calibration: Kinematic Calibration uploaded to the arm." },  // C220 arg=3
    { UINT64_C(0x000000DC00000004), "Kinematic Calibration: Kinematic Calibration reuploaded to the arm." },  // C220 arg=4
    { UINT64_C(0x000000DD00000000), "GUI Communication: High load, messages dropped" },  // C221 arg=0
    { UINT64_C(0x000000DD00000001), "GUI Communication: Overload" },  // C221 arg=1
    { UINT64_C(0x000000DD00000002), "GUI Communication: A variable is too large to be sent to polyscope." },  // C221 arg=2
    { UINT64_C(0x000000DE00000001), "Frame Tracking: Change in offset is too high to meet joint speed safety limit" },  // C222 arg=1
    { UINT64_C(0x000000DE00000002), "Frame Tracking: Change in offset is too high to meet tool speed safety limit" },  // C222 arg=2
    { UINT64_C(0x000000DE00000003), "Frame Tracking: Change in offset is too high to meet momentum safety limit" },  // C222 arg=3
    { UINT64_C(0x000000DE00000004), "Frame Tracking: Change in offset is too high to meet joint speed safety limit" },  // C222 arg=4
    { UINT64_C(0x000000DE00000005), "Frame Tracking: Change in offset is too high to meet tool speed safety limit" },  // C222 arg=5
    { UINT64_C(0x000000DE00000006), "Frame Tracking: Change in offset is too high to meet momentum safety limit" },  // C222 arg=6
    { UINT64_C(0x000000DFFFFFFFFF), "Flexible EtherNet/IP Fieldbus input (custom instance) disconnected" },  // C223 no arg
    { UINT64_C(0x000000E0FFFFFFFF), "{string}" },  // C224 no arg
    { UINT64_C(0x000000E1FFFFFFFF), "Logic Program '{string}' is not in running state." },  // C225 no arg
    { UINT64_C(0x000000E2FFFFFFFF), "A fault was triggered (for test purposes only)" },  // C226 no arg
    { UINT64_C(0x0000010300000000), "Filesystem-related issue: Critical error" },  // C259 arg=0
    { UINT64_C(0x0000010300000050), "Filesystem-related issue" },  // C259 arg=80
    { UINT64_C(0x0000010300000059), "Filesystem-related issue: Failed to create a file with ID {unsigned}" },  // C259 arg=89
    { UINT64_C(0x0000010400000000), "Brake Release - old procedure: Critical error" },  // C260 arg=0
    { UINT64_C(0x0000010500000000), "Temperature Sensor: Critical error" },  // C261 arg=0
    { UINT64_C(0x0000010500000003), "Temperature Sensor: Temperature changed more than allowed: {float} Celsius" },  // C261 arg=3
    { UINT64_C(0x0000010500000004), "Temperature Sensor: Temperature is too high ({float} degrees Celsius)" },  // C261 arg=4
    { UINT64_C(0x0000010500000005), "Temperature Sensor: Temperature is too low ({float} degrees Celsius)" },  // C261 arg=5
    { UINT64_C(0x0000010500000015), "Temperature Sensor: Safety Control Board temperature is too high: {float} Celsius" },  // C261 arg=21
    { UINT64_C(0x0000010500000016), "Temperature Sensor: Safety Control Board temperature is too low: {float} Celsius" },  // C261 arg=22
    { UINT64_C(0x0000010500000017), "Temperature Sensor: The Energy Removal power sensor temperature on the Power Board is too high: {float} Celsius" },  // C261 arg=23
    { UINT64_C(0x0000010500000018), "Temperature Sensor: The Energy Removal power sensor temperature on the Power Board is too low: {float} Celsius" },  // C261 arg=24
    { UINT64_C(0x0000010500000019), "Temperature Sensor: The Robot power sensor temperature on the Power Board is too high: {float} Celsius" },  // C261 arg=25
    { UINT64_C(0x000001050000001A), "Temperature Sensor: The Robot power sensor temperature on the Power Board is too low: {float} Celsius" },  // C261 arg=26
    { UINT64_C(0x000001050000001D), "Temperature Sensor: Unable to read the TOOL PCBA temperature for {float} ms." },  // C261 arg=29
    { UINT64_C(0x000001050000001E), "Temperature Sensor: Safety Control Board temperature changed more than allowed: {float} Celsius" },  // C261 arg=30
    { UINT64_C(0x000001050000001F), "Temperature Sensor: Energy Removal power sensor temperature on the Power Board changed more than allowed: {float} Celsius" },  // C261 arg=31
    { UINT64_C(0x0000010500000020), "Temperature Sensor: PSU 48V circuit temperature is too high: {float} Celsius" },  // C261 arg=32
    { UINT64_C(0x0000010500000021), "Temperature Sensor: PSU 48V circuit temperature is too low: {float} Celsius" },  // C261 arg=33
    { UINT64_C(0x0000010500000022), "Temperature Sensor: PSU 48V circuit temperature changed more than allowed: {float} Celsius" },  // C261 arg=34
    { UINT64_C(0x0000010500000023), "Temperature Sensor: PSU 24V circuit temperature is too high: {float} Celsius" },  // C261 arg=35
    { UINT64_C(0x0000010500000024), "Temperature Sensor: PSU 24V circuit temperature is too low: {float} Celsius" },  // C261 arg=36
    { UINT64_C(0x0000010500000025), "Temperature Sensor: PSU 24V circuit temperature changed more than allowed: {float} Celsius" },  // C261 arg=37
    { UINT64_C(0x0000010500000026), "Temperature Sensor: PSU PFC circuit temperature is too low: {float} Celsius" },  // C261 arg=38
    { UINT64_C(0x0000010500000027), "Temperature Sensor: PSU PFC circuit temperature is too low: {float} Celsius" },  // C261 arg=39
    { UINT64_C(0x0000010500000028), "Temperature Sensor: PSU PFC circuit temperature changed more than allowed: {float} Celsius" },  // C261 arg=40
    { UINT64_C(0x0000010600000000), "Communication: Critical error" },  // C262 arg=0
    { UINT64_C(0x0000010600000011), "Communication: Failed to communicate with {deviceName} Joint" },  // C262 arg=17
    { UINT64_C(0x0000010600000012), "Communication: Failed to communicate with TOOL" },  // C262 arg=18
    { UINT64_C(0x0000010600000019), "Communication: Unexpected message version received: {unsigned}" },  // C262 arg=25
    { UINT64_C(0x000001060000001B), "Communication: Failed to communicate with the Base Filter Board" },  // C262 arg=27
    { UINT64_C(0x000001060000001C), "Communication: Failed to properly instantiate a multi-subscriber message or special command" },  // C262 arg=28
    { UINT64_C(0x000001060000001D), "Communication: Failed to communicate with the Cable Extender with ID: {hex}" },  // C262 arg=29
    { UINT64_C(0x0000010700000000), "Motor Encoder: Critical error" },  // C263 arg=0
    { UINT64_C(0x0000010700000001), "Motor Encoder: Motor Encoder is unavailable" },  // C263 arg=1
    { UINT64_C(0x0000010700000002), "Motor Encoder: Calibration has been invalidated and can lead to reduced performance." },  // C263 arg=2
    { UINT64_C(0x0000010700000015), "Motor Encoder: Validation of the detected Index Mark ({signed}) failed" },  // C263 arg=21
    { UINT64_C(0x000001070000001A), "Motor Encoder: Failure to log missing Index Mark, index out of range: {unsigned}" },  // C263 arg=26
    { UINT64_C(0x000001070000001B), "Motor Encoder: Failure to log index drift, position out of range: {unsigned}" },  // C263 arg=27
    { UINT64_C(0x0000010700000026), "Motor Encoder: Time sanity check failed, difference is too great: {float}" },  // C263 arg=38
    { UINT64_C(0x0000010700000027), "Motor Encoder: The motor encoder data CRC was invalid" },  // C263 arg=39
    { UINT64_C(0x0000010700000029), "Motor Encoder: The rate of invalid samples is too high. Debt counter = {unsigned}" },  // C263 arg=41
    { UINT64_C(0x0000010800000000), "Task Manager: Critical error" },  // C264 arg=0
    { UINT64_C(0x0000010800000007), "Task Manager: Start of Cycle pulse was required but did not occur after {unsigned}ms." },  // C264 arg=7
    { UINT64_C(0x0000010800000008), "Task Manager: Systick timer and Start of Cycle pulse was misaligned by {float}us." },  // C264 arg=8
    { UINT64_C(0x0000010800000009), "Task Manager: {signed} unexpected (+) or missing (-) Start of Cycle pulses was detected within a 1 second period." },  // C264 arg=9
    { UINT64_C(0x000001080000000A), "Task Manager: {unsigned} invalid Start of Cycle pulses was detected within a 1 second period." },  // C264 arg=10
    { UINT64_C(0x000001080000000B), "Task Manager: {unsigned} Start of Cycle pulses was lost in a row" },  // C264 arg=11
    { UINT64_C(0x0000010900000000), "Joint Encoder: Joint encoder position invalid. Detailed error: {hex}" },  // C265 arg=0
    { UINT64_C(0x0000010900000005), "Joint Encoder: Near operation limits. Status: {hex}" },  // C265 arg=5
    { UINT64_C(0x0000010900000006), "Joint Encoder: Not present. Status: {hex}" },  // C265 arg=6
    { UINT64_C(0x0000010900000029), "Joint Encoder: The read head temperature is outside the allowed range." },  // C265 arg=41
    { UINT64_C(0x000001090000002A), "Joint Encoder: Signal amplitude low." },  // C265 arg=42
    { UINT64_C(0x000001090000002B), "Joint Encoder: Signal amplitude too high." },  // C265 arg=43
    { UINT64_C(0x000001090000002C), "Joint Encoder: Signal decoding below confidence threshold - position decoding might be inaccurate or fail." },  // C265 arg=44
    { UINT64_C(0x000001090000002D), "Joint Encoder: Internal speed data is not valid." },  // C265 arg=45
    { UINT64_C(0x000001090000002E), "Joint Encoder: Encoder acceleration too high." },  // C265 arg=46
    { UINT64_C(0x000001090000002F), "Joint Encoder: Magnetic pattern decoding error" },  // C265 arg=47
    { UINT64_C(0x0000010900000030), "Joint Encoder: Signal lost." },  // C265 arg=48
    { UINT64_C(0x0000010900000031), "Joint Encoder: Signal amplitude too high. External magnetic field is present" },  // C265 arg=49
    { UINT64_C(0x0000010900000032), "Joint Encoder: System error. Malfunction inside the circuitry." },  // C265 arg=50
    { UINT64_C(0x0000010900000033), "Joint Encoder: Power supply voltage out of range." },  // C265 arg=51
    { UINT64_C(0x0000010900000034), "Joint Encoder: System error. Inconsistent calibration data is detected." },  // C265 arg=52
    { UINT64_C(0x0000010900000035), "Joint Encoder: Too many warnings in a row." },  // C265 arg=53
    { UINT64_C(0x000001090000003A), "Joint Encoder: Invalid sample rate too high." },  // C265 arg=58
    { UINT64_C(0x0000010A00000000), "Self-test: Critical error" },  // C266 arg=0
    { UINT64_C(0x0000010B00000000), "Bootloader error: Critical error" },  // C267 arg=0
    { UINT64_C(0x0000010B0000000B), "Bootloader error: Hardware configuration issue" },  // C267 arg=11
    { UINT64_C(0x0000010B0000000D), "Bootloader error: Required firmware file is missing for device ID: {hex}" },  // C267 arg=13
    { UINT64_C(0x0000010B0000000E), "Bootloader error: Device not supported by the firmware file, device ID: {hex}" },  // C267 arg=14
    { UINT64_C(0x0000010B0000000F), "Bootloader error: No firmware to boot from for device ID: {hex}" },  // C267 arg=15
    { UINT64_C(0x0000010C00000004), "Special Command: Reboot command received but the device is not allowed to reboot" },  // C268 arg=4
    { UINT64_C(0x0000010D00000046), "Transceiver - deprecated: Flash device is not supported, JEDEC data for device is: {hex}" },  // C269 arg=70
    { UINT64_C(0x0000010F00000001), "Low level real-time thread: Runtime is too much behind." },  // C271 arg=1
    { UINT64_C(0x0000011000000000), "Missing calibration: Critical error - the calibration is missing" },  // C272 arg=0
    { UINT64_C(0x0000011100000000), "Cross monitoring: Critical error" },  // C273 arg=0
    { UINT64_C(0x0000011100000005), "Cross monitoring: Disagreement on Safety Control Board State" },  // C273 arg=5
    { UINT64_C(0x0000011100000006), "Cross monitoring: Disagreement on Robot State" },  // C273 arg=6
    { UINT64_C(0x0000011100000007), "Cross monitoring: Disagreement on Safety State" },  // C273 arg=7
    { UINT64_C(0x0000011100000008), "Cross monitoring: Disagreement on position" },  // C273 arg=8
    { UINT64_C(0x0000011100000009), "Cross monitoring: Disagreement on velocity" },  // C273 arg=9
    { UINT64_C(0x000001110000000A), "Cross monitoring: Disagreement on current" },  // C273 arg=10
    { UINT64_C(0x000001110000000B), "Cross monitoring: Disagreement on temperature" },  // C273 arg=11
    { UINT64_C(0x000001110000000C), "Cross monitoring: Disagreement on Teach Pendant State" },  // C273 arg=12
    { UINT64_C(0x000001110000000D), "Cross monitoring: Disagreement on Teach Pendant Emergency Stop" },  // C273 arg=13
    { UINT64_C(0x000001110000000E), "Cross monitoring: One processor entered Fault State" },  // C273 arg=14
    { UINT64_C(0x000001110000000F), "Cross monitoring: One processor entered Violation State" },  // C273 arg=15
    { UINT64_C(0x0000011100000010), "Cross monitoring: Joint State disagreement" },  // C273 arg=16
    { UINT64_C(0x0000011100000011), "Cross monitoring: Joint Constant Data CRC disagreement" },  // C273 arg=17
    { UINT64_C(0x0000011100000012), "Cross monitoring: Joint target current disagreement" },  // C273 arg=18
    { UINT64_C(0x0000011100000013), "Cross monitoring: Torque Window disagreement" },  // C273 arg=19
    { UINT64_C(0x0000011100000014), "Cross monitoring: Torque Error disagreement" },  // C273 arg=20
    { UINT64_C(0x0000011100000015), "Cross monitoring: Target Velocity disagreement" },  // C273 arg=21
    { UINT64_C(0x0000011100000016), "Cross monitoring: Target Acceleration disagreement" },  // C273 arg=22
    { UINT64_C(0x0000011100000017), "Cross monitoring: Recovery Mode CRC disagreement" },  // C273 arg=23
    { UINT64_C(0x0000011100000018), "Cross monitoring: Robot Configuration CRC disagreement" },  // C273 arg=24
    { UINT64_C(0x0000011100000019), "Cross monitoring: User Configuration CRC disagreement" },  // C273 arg=25
    { UINT64_C(0x000001110000001A), "Cross monitoring: Maximum stopping time disagreement" },  // C273 arg=26
    { UINT64_C(0x000001110000001B), "Cross monitoring: Stopping Time Torque Overload disagreement" },  // C273 arg=27
    { UINT64_C(0x000001110000001C), "Cross monitoring: Disagreement error on joint {unsigned}" },  // C273 arg=28
    { UINT64_C(0x000001110000001D), "Cross monitoring: Tool speed disagreement" },  // C273 arg=29
    { UINT64_C(0x000001110000001E), "Cross monitoring: Safety Mode Limit disagreement" },  // C273 arg=30
    { UINT64_C(0x000001110000001F), "Cross monitoring: Hand Protection Distance disagreement" },  // C273 arg=31
    { UINT64_C(0x0000011100000020), "Cross monitoring: Elbow Sphere speed disagreement" },  // C273 arg=32
    { UINT64_C(0x0000011100000021), "Cross monitoring: Momentum disagreement" },  // C273 arg=33
    { UINT64_C(0x0000011100000022), "Cross monitoring: Power disagreeement" },  // C273 arg=34
    { UINT64_C(0x0000011100000023), "Cross monitoring: Elbow position disagreement" },  // C273 arg=35
    { UINT64_C(0x0000011100000024), "Cross monitoring: Workpiece Rotation disagreement" },  // C273 arg=36
    { UINT64_C(0x0000011100000025), "Cross monitoring: Disagreement on Workpiece Position" },  // C273 arg=37
    { UINT64_C(0x0000011100000026), "Cross monitoring: Disagreement on motor parameter (R_pp)" },  // C273 arg=38
    { UINT64_C(0x0000011100000027), "Cross monitoring: Disagreement on motor parameter (L_pp)" },  // C273 arg=39
    { UINT64_C(0x0000011100000028), "Cross monitoring: Disagreement on motor parameter (Kb)" },  // C273 arg=40
    { UINT64_C(0x0000011100000029), "Cross monitoring: Disagreement on motor parameter (Kt)" },  // C273 arg=41
    { UINT64_C(0x000001110000002A), "Cross monitoring: Disagreement on motor parameter (T)" },  // C273 arg=42
    { UINT64_C(0x000001110000002B), "Cross monitoring: Disagreement on the Teach Pendant's Three-Position Enabling Device" },  // C273 arg=43
    { UINT64_C(0x000001110000002C), "Cross monitoring: Disagreement on the active status of the Teach Pendant's Three-Position Enabling Device" },  // C273 arg=44
    { UINT64_C(0x000001110000002F), "Cross monitoring: Disagreement on state" },  // C273 arg=47
    { UINT64_C(0x0000011100000030), "Cross monitoring: Disagreement on Injection-Molding-Machine-Interface Emergency Stop input" },  // C273 arg=48
    { UINT64_C(0x0000011100000031), "Cross monitoring: Disagreement on Injection-Molding-Machine-Interface Emergency Stop output" },  // C273 arg=49
    { UINT64_C(0x0000011100000032), "Cross monitoring: Disagreement on Injection-Molding-Machine-Interface Safeguard input" },  // C273 arg=50
    { UINT64_C(0x0000011100000033), "Cross monitoring: Disagreement on Injection-Molding-Machine-Interface type" },  // C273 arg=51
    { UINT64_C(0x0000011100000034), "Cross monitoring: Disagreement on Torque Parameters CRC" },  // C273 arg=52
    { UINT64_C(0x0000011100000035), "Cross monitoring: Target Torque disagreement" },  // C273 arg=53
    { UINT64_C(0x0000011100000036), "Cross monitoring: Disagreement on hardware configuration CRC" },  // C273 arg=54
    { UINT64_C(0x0000011100000037), "Cross monitoring: Disagreement on compensation current" },  // C273 arg=55
    { UINT64_C(0x0000011100000038), "Cross monitoring: Disagreement on external torque target" },  // C273 arg=56
    { UINT64_C(0x0000011100000039), "Cross monitoring: Safety Target Torque disagreement" },  // C273 arg=57
    { UINT64_C(0x000001110000003A), "Cross monitoring: Disagreement on all motors off in arm" },  // C273 arg=58
    { UINT64_C(0x000001110000003D), "Cross monitoring: Disagreement on joint gear temperature" },  // C273 arg=61
    { UINT64_C(0x000001110000003E), "Cross monitoring: Disagreement on joint house temperature" },  // C273 arg=62
    { UINT64_C(0x000001110000003F), "Cross monitoring: Disagreement on the tool safety input" },  // C273 arg=63
    { UINT64_C(0x0000011100000040), "Cross monitoring: One processor entered Critical Fault State" },  // C273 arg=64
    { UINT64_C(0x0000011100000042), "Cross monitoring: Disagreement on linear velocity" },  // C273 arg=66
    { UINT64_C(0x0000011100000043), "Cross monitoring: Disagreement on angular velocity" },  // C273 arg=67
    { UINT64_C(0x0000011100000044), "Cross monitoring: Disagreement on scanner monitoring case for scanner {unsigned}" },  // C273 arg=68
    { UINT64_C(0x0000011100000045), "Cross monitoring: Disagreement on raw current" },  // C273 arg=69
    { UINT64_C(0x0000011100000046), "Cross monitoring: Safety field configuration CRC disagreement" },  // C273 arg=70
    { UINT64_C(0x0000011100000047), "Cross monitoring: Safety stop flags disagreement" },  // C273 arg=71
    { UINT64_C(0x0000011200000001), "Control box fan error: Fan is not running" },  // C274 arg=1
    { UINT64_C(0x0000011200000002), "Control box fan error: Monitoring data timed out" },  // C274 arg=2
    { UINT64_C(0x000001130000000C), "Brake Pin: Attempted to update brakepin state machine before it was initialized." },  // C275 arg=12
    { UINT64_C(0x0000011400000000), "Uart: Critical error" },  // C276 arg=0
    { UINT64_C(0x0000011500000001), "Memory: Failed to allocate memory" },  // C277 arg=1
    { UINT64_C(0x0000011600000000), "Servo: Critical error" },  // C278 arg=0
    { UINT64_C(0x0000011600000012), "Servo: The version of the received control message is not supported. Version received: {unsigned}" },  // C278 arg=18
    { UINT64_C(0x0000011600000014), "Servo: The control-mode state {unsigned} in the received control message is not supported." },  // C278 arg=20
    { UINT64_C(0x0000011700000000), "Flash: Critical error" },  // C279 arg=0
    { UINT64_C(0x0000011700000026), "Flash: Timed out waiting for system voltage to reach {float}V" },  // C279 arg=38
    { UINT64_C(0x0000011800000000), "Real-time error: Critical error" },  // C280 arg=0
    { UINT64_C(0x0000011900000000), "Robot State Machine: Critical error" },  // C281 arg=0
    { UINT64_C(0x0000011900000003), "Robot State Machine: {deviceName} joint entered the Fault State" },  // C281 arg=3
    { UINT64_C(0x0000011900000004), "Robot State Machine: {deviceName} joint entered the Violation State" },  // C281 arg=4
    { UINT64_C(0x0000011900000007), "Robot State Machine: Teach Pendant entered the Fault State" },  // C281 arg=7
    { UINT64_C(0x0000011900000008), "Robot State Machine: Teach Pendant entered the Violation State" },  // C281 arg=8
    { UINT64_C(0x0000011900000009), "Robot State Machine: {deviceName} joint moved too far before robot entered RUNNING State" },  // C281 arg=9
    { UINT64_C(0x000001190000000E), "Robot State Machine: IMMI entered the Fault State" },  // C281 arg=14
    { UINT64_C(0x000001190000000F), "Robot State Machine: IMMI entered the Violation State" },  // C281 arg=15
    { UINT64_C(0x0000011900000010), "Robot State Machine: {deviceName} joint did not reach correct state before timeout was exceeded" },  // C281 arg=16
    { UINT64_C(0x0000011900000011), "Robot State Machine: {deviceName} joint entered the Brake Failure State" },  // C281 arg=17
    { UINT64_C(0x0000011900000012), "Robot State Machine: Not all joints reached parking within the expected time of {unsigned} ms" },  // C281 arg=18
    { UINT64_C(0x0000011900000014), "Robot State Machine: {deviceName} joint entered the Critical Fault State" },  // C281 arg=20
    { UINT64_C(0x0000011900000015), "Robot State Machine: Tool entered the Fault State" },  // C281 arg=21
    { UINT64_C(0x0000011B00000000), "Safety system: Critical error" },  // C283 arg=0
    { UINT64_C(0x0000011B00000001), "Safety system: Robot is not braking when in Stop Mode" },  // C283 arg=1
    { UINT64_C(0x0000011B00000002), "Safety system: Robot is moving when in Stop Mode" },  // C283 arg=2
    { UINT64_C(0x0000011B00000003), "Safety system: Power not removed from the motors while in Emergency Stop" },  // C283 arg=3
    { UINT64_C(0x0000011B00000004), "Safety system: Failed to power on the Robot Arm" },  // C283 arg=4
    { UINT64_C(0x0000011B00000005), "Safety system: Invalid pin-configuration received: {hex}" },  // C283 arg=5
    { UINT64_C(0x0000011B00000006), "Safety system: Trying to reassign pin configuration with configuration {hex}" },  // C283 arg=6
    { UINT64_C(0x0000011B00000007), "Safety system: {deviceName} joint exceeded the speed limit of the safety settings" },  // C283 arg=7
    { UINT64_C(0x0000011B00000008), "Safety system: The System Emergency Stop Output is not active" },  // C283 arg=8
    { UINT64_C(0x0000011B00000009), "Safety system: System Emergency Stop Output disagreement within the safety system" },  // C283 arg=9
    { UINT64_C(0x0000011B0000000A), "Safety system: Robot Emergency Stop Input disagreement within the safety system" },  // C283 arg=10
    { UINT64_C(0x0000011B0000000B), "Safety system: System Emergency Stop Input disagreement within the safety system" },  // C283 arg=11
    { UINT64_C(0x0000011B0000000C), "Safety system: Safeguard Stop Input disagreement within the safety system" },  // C283 arg=12
    { UINT64_C(0x0000011B0000000D), "Safety system: Safeguard Reset Input disagreement within the safety system" },  // C283 arg=13
    { UINT64_C(0x0000011B0000000E), "Safety system: Operation Mode input disagreement within the safety system." },  // C283 arg=14
    { UINT64_C(0x0000011B0000000F), "Safety system: Three-Positional Enabling Device Input disagreement within the safety system" },  // C283 arg=15
    { UINT64_C(0x0000011B00000010), "Safety system: Operation Mode Switch is defined and no Three-Positional Device is defined." },  // C283 arg=16
    { UINT64_C(0x0000011B00000011), "Safety system: Lost {unsigned} Teach Pendant safety packages in a row" },  // C283 arg=17
    { UINT64_C(0x0000011B00000012), "Safety system: Lost too many Joint safety packages in a row. Diagnostic data: {unsigned}" },  // C283 arg=18
    { UINT64_C(0x0000011B00000013), "Safety system: Invalid gravity vector received" },  // C283 arg=19
    { UINT64_C(0x0000011B00000014), "Safety system: Invalid payload mass received" },  // C283 arg=20
    { UINT64_C(0x0000011B00000015), "Safety system: invalid payload center of gravity received" },  // C283 arg=21
    { UINT64_C(0x0000011B00000016), "Safety system: Teach Pendant is connected while it is disabled in robot configuration" },  // C283 arg=22
    { UINT64_C(0x0000011B0000001A), "Safety system: Force limitation: A joint exceeded the torque window by {float}Nm" },  // C283 arg=26
    { UINT64_C(0x0000011B0000001B), "Safety system: Mismatch on Robot Configuration CRC between the safety system and PolyScope" },  // C283 arg=27
    { UINT64_C(0x0000011B0000001C), "Safety system: Mismatch on User Configuration CRC between the safety system and PolyScope" },  // C283 arg=28
    { UINT64_C(0x0000011B00000022), "Safety system: Error while trying to apply safety configuration" },  // C283 arg=34
    { UINT64_C(0x0000011B00000023), "Safety system: Reduced Mode Output disagreement within the safety system" },  // C283 arg=35
    { UINT64_C(0x0000011B00000024), "Safety system: Not Reduced Mode Output disagreement within the safety system" },  // C283 arg=36
    { UINT64_C(0x0000011B00000025), "Safety system: Robot Moving Output disagreement within the safety system" },  // C283 arg=37
    { UINT64_C(0x0000011B00000026), "Safety system: Robot Not Stopping Output disagreement within the safety system" },  // C283 arg=38
    { UINT64_C(0x0000011B00000028), "Safety system: Reduced Mode Input disagreement within the safety systems" },  // C283 arg=40
    { UINT64_C(0x0000011B00000029), "Safety system: TCP Velocity violates limits of maximum stopping time" },  // C283 arg=41
    { UINT64_C(0x0000011B0000002A), "Safety system: TCP Velocity violates limits of maximum stopping distance" },  // C283 arg=42
    { UINT64_C(0x0000011B0000002B), "Safety system: {deviceName} joint moved too quickly toward a Joint position limit" },  // C283 arg=43
    { UINT64_C(0x0000011B0000002C), "Safety system: The tool moved too fast towards an orientation limit" },  // C283 arg=44
    { UINT64_C(0x0000011B0000002D), "Safety system: The Elbow moved too fast towards a safety plane" },  // C283 arg=45
    { UINT64_C(0x0000011B0000002E), "Safety system: The tool moved too fast towards a safety plane" },  // C283 arg=46
    { UINT64_C(0x0000011B0000002F), "Safety system: {deviceName} joint position limit exceeded" },  // C283 arg=47
    { UINT64_C(0x0000011B00000030), "Safety system: Tool position limit exceeded" },  // C283 arg=48
    { UINT64_C(0x0000011B00000031), "Safety system: Tool orientation limit exceeded" },  // C283 arg=49
    { UINT64_C(0x0000011B00000032), "Safety system: Elbow position limit exceeded" },  // C283 arg=50
    { UINT64_C(0x0000011B00000033), "Safety system: Robot moved with a speed of {float} mm/s at the tool. This exceeds the tool speed limit in the safety settings" },  // C283 arg=51
    { UINT64_C(0x0000011B00000034), "Safety system: Robot moved with a speed of {float} mm/s at the elbow. This exceeds the elbow speed limit in the safety settings" },  // C283 arg=52
    { UINT64_C(0x0000011B00000035), "Safety system: Maximum Tool Center Point Speed in Reduced Mode is invalid" },  // C283 arg=53
    { UINT64_C(0x0000011B00000036), "Safety system: Maximum Elbow Speed in Reduced Mode is invalid" },  // C283 arg=54
    { UINT64_C(0x0000011B00000037), "Safety system: Maximum Joint Speed of joint {unsigned} in Reduced Mode is invalid" },  // C283 arg=55
    { UINT64_C(0x0000011B00000038), "Safety system: Maximum Momentum in Reduced Mode is invalid" },  // C283 arg=56
    { UINT64_C(0x0000011B00000039), "Safety system: Maximum stopping time in Reduced Mode is invalid" },  // C283 arg=57
    { UINT64_C(0x0000011B0000003A), "Safety system: Maximum stopping distance in Reduced Mode is invalid" },  // C283 arg=58
    { UINT64_C(0x0000011B0000003B), "Safety system: Reduced Mode Output is not active" },  // C283 arg=59
    { UINT64_C(0x0000011B0000003C), "Safety system: Reduced Mode Output is not inactive" },  // C283 arg=60
    { UINT64_C(0x0000011B0000003D), "Safety system: Not Reduced Mode Output is not active" },  // C283 arg=61
    { UINT64_C(0x0000011B0000003E), "Safety system: Not Reduced Mode Output is not inactive" },  // C283 arg=62
    { UINT64_C(0x0000011B0000003F), "Safety system: Robot is moving while Robot Moving Output is not active" },  // C283 arg=63
    { UINT64_C(0x0000011B00000040), "Safety system: Tool Direction Vector Length for Normal Mode is {float}, not 1.0" },  // C283 arg=64
    { UINT64_C(0x0000011B00000041), "Safety system: Tool Direction Vector Length for Reduced Mode is {float}, not 1.0" },  // C283 arg=65
    { UINT64_C(0x0000011B00000042), "Safety system: Robot Momentum reached {float} kg * m/s, which exceeds the Momentum limit" },  // C283 arg=66
    { UINT64_C(0x0000011B00000043), "Safety system: Robot Power reached {float} W, which exceeds the Power limit" },  // C283 arg=67
    { UINT64_C(0x0000011B00000044), "Safety system: Error caused by {deviceName}" },  // C283 arg=68
    { UINT64_C(0x0000011B00000048), "Safety system: The motor configuration sent by the Control Box is invalid" },  // C283 arg=72
    { UINT64_C(0x0000011B00000049), "Safety system: Safe Home Position Output disagreement within the safety system" },  // C283 arg=73
    { UINT64_C(0x0000011B0000004A), "Safety system: The Safe Home Position Output is active while not allowed" },  // C283 arg=74
    { UINT64_C(0x0000011B00000051), "Safety system: The robot configuration specifies an unsupported joint size {signed}" },  // C283 arg=81
    { UINT64_C(0x0000011B00000052), "Safety system: The connected Teach Pendant type does not match the configuration" },  // C283 arg=82
    { UINT64_C(0x0000011B00000053), "Safety system: The configured Teach Pendant has no Three-Positional Enabling Device" },  // C283 arg=83
    { UINT64_C(0x0000011B00000055), "Safety system: Automatic Safeguard Stop Input disagreement within the safety system" },  // C283 arg=85
    { UINT64_C(0x0000011B00000056), "Safety system: Automatic Safeguard Reset Input disagreement within the safety system" },  // C283 arg=86
    { UINT64_C(0x0000011B00000057), "Safety system: Injection-Molding-Machine-Interface is connected while it is disabled in the robot configuration" },  // C283 arg=87
    { UINT64_C(0x0000011B00000058), "Safety system: Lost {unsigned} Injection-Molding-Machine-Interface safety packages in a row" },  // C283 arg=88
    { UINT64_C(0x0000011B00000059), "Safety system: The connected Injection-Molding-Machine-Interface type does not match the configuration" },  // C283 arg=89
    { UINT64_C(0x0000011B0000005A), "Safety system: Invalid Injection-Molding-Machine-Interface type in the user configuration: {unsigned}" },  // C283 arg=90
    { UINT64_C(0x0000011B0000005B), "Safety system: The Injection-Molding-Machine-Interface System Emergency Stop Output is not active" },  // C283 arg=91
    { UINT64_C(0x0000011B0000005E), "Safety system: Automatic Safeguard Stop input is configured but no Three-Position Enabling device is configured" },  // C283 arg=94
    { UINT64_C(0x0000011B00000061), "Safety system: The payload inertia matrix diagonal sent from the controller must be non-negative" },  // C283 arg=97
    { UINT64_C(0x0000011B00000062), "Safety system: The payload inertia sent from the controller must be within valid range" },  // C283 arg=98
    { UINT64_C(0x0000011B00000063), "Safety system: Received an invalid value {float} as part of the runtime safety configuration" },  // C283 arg=99
    { UINT64_C(0x0000011B00000064), "Safety system: Multiple sources defined for controlling operational mode" },  // C283 arg=100
    { UINT64_C(0x0000011B00000066), "Safety system: The reduced mode state is inactive while not allowed" },  // C283 arg=102
    { UINT64_C(0x0000011B00000067), "Safety system: The reduced mode state is active while not allowed" },  // C283 arg=103
    { UINT64_C(0x0000011B00000068), "Safety system: Robot is moving while the robot moving state is not active" },  // C283 arg=104
    { UINT64_C(0x0000011B00000069), "Safety system: The safe home position state is active while not allowed" },  // C283 arg=105
    { UINT64_C(0x0000011B0000006A), "Safety system: The safeguard stop state is active while not allowed" },  // C283 arg=106
    { UINT64_C(0x0000011B0000006B), "Safety system: The safeguard stop state is inactive while not allowed" },  // C283 arg=107
    { UINT64_C(0x0000011B00000070), "Safety system: The emergency stop by system state is active while not allowed" },  // C283 arg=112
    { UINT64_C(0x0000011B00000071), "Safety system: The emergency stop by system state is inactive while not allowed" },  // C283 arg=113
    { UINT64_C(0x0000011B00000072), "Safety system: The emergency stop by robot state is active while not allowed" },  // C283 arg=114
    { UINT64_C(0x0000011B00000073), "Safety system: The emergency stop by robot state is inactive while not allowed" },  // C283 arg=115
    { UINT64_C(0x0000011B00000074), "Safety system: The Fault state is inactive while not allowed" },  // C283 arg=116
    { UINT64_C(0x0000011B00000075), "Safety system: The fault state is active while not allowed" },  // C283 arg=117
    { UINT64_C(0x0000011B00000076), "Safety system: The violation state is inactive while not allowed" },  // C283 arg=118
    { UINT64_C(0x0000011B00000077), "Safety system: The violation state is active while not allowed" },  // C283 arg=119
    { UINT64_C(0x0000011B00000078), "Safety system: The category 1 stop state is inactive while not allowed" },  // C283 arg=120
    { UINT64_C(0x0000011B00000079), "Safety system: The category 1 stop state is active while not allowed" },  // C283 arg=121
    { UINT64_C(0x0000011B0000007A), "Safety system: The safeguard stop auto state is active while not allowed" },  // C283 arg=122
    { UINT64_C(0x0000011B0000007B), "Safety system: The safeguard stop auto state is inactive while not allowed" },  // C283 arg=123
    { UINT64_C(0x0000011B0000007C), "Safety system: The 3PE stop state is inactive while not allowed" },  // C283 arg=124
    { UINT64_C(0x0000011B0000007D), "Safety system: The 3PE stop state is active while not allowed" },  // C283 arg=125
    { UINT64_C(0x0000011B0000007E), "Safety system: The category 2 stop state is inactive while not allowed" },  // C283 arg=126
    { UINT64_C(0x0000011B0000007F), "Safety system: The category 2 stop state is active while not allowed" },  // C283 arg=127
    { UINT64_C(0x0000011B00000080), "Safety system: The category 0 stop state is inactive while not allowed" },  // C283 arg=128
    { UINT64_C(0x0000011B00000081), "Safety system: The category 0 stop state is active while not allowed" },  // C283 arg=129
    { UINT64_C(0x0000011B00000082), "Safety system: The safety mode limit is incorrect" },  // C283 arg=130
    { UINT64_C(0x0000011B00000083), "Safety system: The operational mode is incorrect" },  // C283 arg=131
    { UINT64_C(0x0000011B00000086), "Safety system: Teach pendants without 3PE not supported" },  // C283 arg=134
    { UINT64_C(0x0000011B00000087), "Safety system: Freedrive safety-input disagreement within the safety system" },  // C283 arg=135
    { UINT64_C(0x0000011B00000088), "Safety system: Brake monitoring not initialized correctly." },  // C283 arg=136
    { UINT64_C(0x0000011B00000089), "Safety system: Invalid number of dynamic safety clients." },  // C283 arg=137
    { UINT64_C(0x0000011B0000008F), "Safety system: Tool safety input is configured but the safety configuration version does not support it" },  // C283 arg=143
    { UINT64_C(0x0000011B00000090), "Safety system: Tool safety inputs are configured but the robot type does not support it" },  // C283 arg=144
    { UINT64_C(0x0000011B00000091), "Safety system: Tool safety inputs are configured but the joint hardware does not support it" },  // C283 arg=145
    { UINT64_C(0x0000011B00000092), "Safety system: Tool safety inputs are configured but the safety input type is not supported" },  // C283 arg=146
    { UINT64_C(0x0000011B00000093), "Safety system: Tried to get the state of A IO {unsigned} which is not a valid IO" },  // C283 arg=147
    { UINT64_C(0x0000011B00000094), "Safety system: Tried to get the state of B IO {unsigned} which is not a valid IO" },  // C283 arg=148
    { UINT64_C(0x0000011B00000095), "Safety system: Tool safety inputs are configured, but the robot configuration does not contain a wrist3 joint" },  // C283 arg=149
    { UINT64_C(0x0000011B00000096), "Safety system: Tool safety inputs are configured, but was outside the limits for too long" },  // C283 arg=150
    { UINT64_C(0x0000011B00000098), "Safety system: Safety plane is configured with IO, but no IO is configured" },  // C283 arg=152
    { UINT64_C(0x0000011B00000099), "Safety system: Safety Plane with IO safety-input disagreement within the safety system" },  // C283 arg=153
    { UINT64_C(0x0000011B0000009A), "Safety system: Safety Plane with IO safety-output disagreement within the safety system" },  // C283 arg=154
    { UINT64_C(0x0000011B0000009B), "Safety system: The Safety Plane Output is not active" },  // C283 arg=155
    { UINT64_C(0x0000011B0000009D), "Safety system: The Safety Plane Output is not inactive" },  // C283 arg=157
    { UINT64_C(0x0000011B000000A4), "Safety system: Three position enabling stop output is in an incorrect state" },  // C283 arg=164
    { UINT64_C(0x0000011B000000A5), "Safety system: Three position enabling stop output is in an incorrect state" },  // C283 arg=165
    { UINT64_C(0x0000011B000000A6), "Safety system: No three position enabling stop output is in an incorrect state" },  // C283 arg=166
    { UINT64_C(0x0000011B000000A7), "Safety system: No three position enabling stop output is in an incorrect state" },  // C283 arg=167
    { UINT64_C(0x0000011B000000A8), "Safety system: Lost {unsigned} Tool safety packages in a row" },  // C283 arg=168
    { UINT64_C(0x0000011B000000A9), "Safety system: Three position enabling stop output disagreement within the safety system" },  // C283 arg=169
    { UINT64_C(0x0000011B000000AA), "Safety system: Not three position enabling stop output disagreement within the safety system" },  // C283 arg=170
    { UINT64_C(0x0000011B000000AB), "Safety system: Tool safety inputs are configured but not supported" },  // C283 arg=171
    { UINT64_C(0x0000011B000000AC), "Safety system: Received an unexpected tool safety message" },  // C283 arg=172
    { UINT64_C(0x0000011B000000AD), "Safety system: No tool is configured but tool safety inputs are configured" },  // C283 arg=173
    { UINT64_C(0x0000011B000000AF), "Safety system: Tool type disagreement within the safety system" },  // C283 arg=175
    { UINT64_C(0x0000011B000000B1), "Safety system: Tool safety inputs are not supported in this software version" },  // C283 arg=177
    { UINT64_C(0x0000011B000000B2), "Safety system: Parking did not begin within the expected time after emergency stop" },  // C283 arg=178
    { UINT64_C(0x0000011B000000B3), "Safety system: The PROFIsafe configuration is obsolete." },  // C283 arg=179
    { UINT64_C(0x0000011B000000B4), "Safety system: The maximum rotation distance in the movement monitor is too large. Max distance: {float} rad." },  // C283 arg=180
    { UINT64_C(0x0000011B000000B5), "Safety system: Mismatch between tool A type and input state" },  // C283 arg=181
    { UINT64_C(0x0000011B000000B6), "Safety system: Mismatch between tool B type and input state" },  // C283 arg=182
    { UINT64_C(0x0000011B000000B9), "Safety system: The minimum payload limit must be zero or greater. Actual value is {float}kg" },  // C283 arg=185
    { UINT64_C(0x0000011B000000BA), "Safety system: The maximum payload must be greater than or equal to the minimum payload" },  // C283 arg=186
    { UINT64_C(0x0000011B000000BD), "Safety system: Center of gravity sphere radius must be zero or greater. Actual value is {float}m" },  // C283 arg=189
    { UINT64_C(0x0000011B000000BE), "Safety system: A payload mass below the minimum payload limit was received. Actual value is {float}kg" },  // C283 arg=190
    { UINT64_C(0x0000011B000000BF), "Safety system: A payload mass above the maximum payload limit was received. Actual value is {float}kg" },  // C283 arg=191
    { UINT64_C(0x0000011B000000C0), "Safety system: A Payload center of gravity outside the allowed limits was received" },  // C283 arg=192
    { UINT64_C(0x0000011B000000C8), "Safety system: Manual mode high speed output disagreement within the safety system" },  // C283 arg=200
    { UINT64_C(0x0000011B000000C9), "Safety system: The manual mode reduced speed configuration must be greater than zero. Actual value is {float}mm/s" },  // C283 arg=201
    { UINT64_C(0x0000011B000000CA), "Safety system: The manual mode reduced speed configuration is above the limit. Actual value is {float}mm/s" },  // C283 arg=202
    { UINT64_C(0x0000011B000000CC), "Safety system: Robot moved with a speed of {float}mm/s at the tool. This exceeds the manual mode reduced speed limit in the safety settings" },  // C283 arg=204
    { UINT64_C(0x0000011B000000CD), "Safety system: Robot moved with a speed of {float}mm/s at the elbow. This exceeds the manual mode reduced speed limit in the safety settings" },  // C283 arg=205
    { UINT64_C(0x0000011B000000CE), "Safety system: Manual mode high speed output is in an incorrect state" },  // C283 arg=206
    { UINT64_C(0x0000011B000000CF), "Safety system: Manual mode high speed output is in an incorrect state" },  // C283 arg=207
    { UINT64_C(0x0000011B000000D0), "Safety system: Parking did not begin within the expected time after safety stop" },  // C283 arg=208
    { UINT64_C(0x0000011B000000D2), "Safety system: Wrist position limit exceeded" },  // C283 arg=210
    { UINT64_C(0x0000011B000000D3), "Safety system: Scanner Front Input disagreement within the safety system" },  // C283 arg=211
    { UINT64_C(0x0000011B000000D4), "Safety system: Scanner Rear Input disagreement within the safety system" },  // C283 arg=212
    { UINT64_C(0x0000011B000000D5), "Safety system: The freedrive speed configuration must be greater than zero. Actual value is {float}mm/s" },  // C283 arg=213
    { UINT64_C(0x0000011B000000D6), "Safety system: Robot moved with a speed of {float}mm/s at the tool. This exceeds the freedrive speed limit in the safety settings" },  // C283 arg=214
    { UINT64_C(0x0000011B000000D7), "Safety system: Robot moved with a speed of {float}mm/s at the elbow. This exceeds the freedrive speed limit in the safety settings" },  // C283 arg=215
    { UINT64_C(0x0000011B000000D8), "Safety system: The Wrist moved too fast towards a safety plane" },  // C283 arg=216
    { UINT64_C(0x0000011B000000DC), "Safety system: Invalid monitoring case {unsigned} was requested for the front scanner" },  // C283 arg=220
    { UINT64_C(0x0000011B000000DD), "Safety system: Invalid monitoring case {unsigned} was requested for the rear scanner" },  // C283 arg=221
    { UINT64_C(0x0000011B000000DE), "Safety system: The linear velocity of {float} m/s is outside the limits of the active scanner monitoring case" },  // C283 arg=222
    { UINT64_C(0x0000011B000000DF), "Safety system: The angular velocity of {float} rad/s is outside the limits of the active scanner monitoring case" },  // C283 arg=223
    { UINT64_C(0x0000011B000000E6), "Safety system: Mismatch on Safety Field Configuration CRC between the safety system and the configuration" },  // C283 arg=230
    { UINT64_C(0x0000011B000000E7), "Safety system: Audio/visual motion warning violation" },  // C283 arg=231
    { UINT64_C(0x0000011C00000000), "Brake Release: Critical error" },  // C284 arg=0
    { UINT64_C(0x0000011C00000015), "Brake Release: Brake release count reached limit" },  // C284 arg=21
    { UINT64_C(0x0000011C00000016), "Brake Release: Brake release count is close to the limit, remaining brake releases: {unsigned}" },  // C284 arg=22
    { UINT64_C(0x0000011D00000000), "Joint Keep-Alive System: Critical error" },  // C285 arg=0
    { UINT64_C(0x0000011D0000000A), "Joint Keep-Alive System: Lost {unsigned} Keep-Alive System message(s) in a row from Safety Control Board-uPA" },  // C285 arg=10
    { UINT64_C(0x0000011D0000000B), "Joint Keep-Alive System: Lost {unsigned} Keep-Alive System message(s) in a row from Safety Control Board-uPB" },  // C285 arg=11
    { UINT64_C(0x0000011E00000002), "Motor Controller: PWM margin too small, ticks left: {signed}" },  // C286 arg=2
    { UINT64_C(0x0000011E00000004), "Motor Controller: PWM is not zero when in power off" },  // C286 arg=4
    { UINT64_C(0x0000011F00000000), "Saved files: Critical error" },  // C287 arg=0
    { UINT64_C(0x0000011F00000001), "Saved files: The file requested (id {unsigned}) is not saved or its loading failed" },  // C287 arg=1
    { UINT64_C(0x0000012100000001), "Tool Connector: Too high current detected on Digital Output: {unsigned} high side" },  // C289 arg=1
    { UINT64_C(0x0000012100000002), "Tool Connector: Too high sink current detected on Digital Output: {unsigned} low side" },  // C289 arg=2
    { UINT64_C(0x0000012100000004), "Tool Connector: 10 second Average tool IO Current of {float} A is outside of the allowed range." },  // C289 arg=4
    { UINT64_C(0x0000012100000005), "Tool Connector: Unable to remove tool Digital Output fault." },  // C289 arg=5
    { UINT64_C(0x0000012100000006), "Tool Connector: Current of {float} A on the tool connector supply pins is outside of the allowed range." },  // C289 arg=6
    { UINT64_C(0x0000012100000007), "Tool Connector: Current of {float} A on the Digital Output pins is outside of the allowed range." },  // C289 arg=7
    { UINT64_C(0x0000012100000008), "Tool Connector: Current of {float} A on the ground pin is outside of the allowed range." },  // C289 arg=8
    { UINT64_C(0x0000012100000009), "Tool Connector: Current of {float} A on the POWER pin is outside of the allowed range." },  // C289 arg=9
    { UINT64_C(0x000001210000000A), "Tool Connector: The M8 digital output sinking current of {float} A exceeded the allowed single pin limit." },  // C289 arg=10
    { UINT64_C(0x000001210000000B), "Tool Connector: The M8 digital output sinking current of {float} A exceeded the allowed dual pin limit." },  // C289 arg=11
    { UINT64_C(0x000001210000000C), "Tool Connector: The M8 digital output 0 sourcing current of {float} A exceeded the allowed limit." },  // C289 arg=12
    { UINT64_C(0x000001210000000D), "Tool Connector: The M8 digital output 1 sourcing current of {float} A exceeded the allowed limit." },  // C289 arg=13
    { UINT64_C(0x000001210000000E), "Tool Connector: The zero current of the M8-power current sensor exceeds maximum allowed offset. The measured offset was {float}A" },  // C289 arg=14
    { UINT64_C(0x000001210000000F), "Tool Connector: The momentary M8-power current of {float}A exceeded the allowed limit." },  // C289 arg=15
    { UINT64_C(0x0000012100000010), "Tool Connector: The M8-power current exceeded the the allowed limit of {float}A for more than 2 seconds during the last 10 seconds" },  // C289 arg=16
    { UINT64_C(0x0000012100000011), "Tool Connector: The zero current of the IO M8 power current sensor exceeds maximum allowed offset. The measured offset was {float}A" },  // C289 arg=17
    { UINT64_C(0x0000012100000012), "Tool Connector: The zero current of the IO M8 digital output current sensor exceeds maximum allowed offset. The measured offset was {float}A" },  // C289 arg=18
    { UINT64_C(0x0000012400000000), "Online RAM test: Critical error" },  // C292 arg=0
    { UINT64_C(0x0000012600000000), "ADC: Critical error" },  // C294 arg=0
    { UINT64_C(0x000001260000000C), "ADC: The single ended ADC calibration timed out" },  // C294 arg=12
    { UINT64_C(0x0000012600000010), "ADC: The joint failed during recalibration of the ADC offset" },  // C294 arg=16
    { UINT64_C(0x0000012600000013), "ADC: The motor moves too much during ADC offset sampling, the motor position has a standard deviation of {float} [rad] during the sampling" },  // C294 arg=19
    { UINT64_C(0x0000012700000000), "PCB: Wrong PCB type ({hex})" },  // C295 arg=0
    { UINT64_C(0x0000012800000000), "Start up check: Critical error" },  // C296 arg=0
    { UINT64_C(0x0000012800000001), "Start up check: SCB IO failed to power on" },  // C296 arg=1
    { UINT64_C(0x0000012800000002), "Start up check: One or more Motor phases is short circuited to ground. Diagnostic data: {hex}" },  // C296 arg=2
    { UINT64_C(0x0000012800000003), "Start up check: Motor Indication Signal does not work. Diagnostic data: {hex}" },  // C296 arg=3
    { UINT64_C(0x0000012800000004), "Start up check: Phase 1 is not connected. Diagnostic data: {hex}" },  // C296 arg=4
    { UINT64_C(0x0000012800000005), "Start up check: Phase 2 is not connected. Diagnostic data: {hex}" },  // C296 arg=5
    { UINT64_C(0x0000012800000006), "Start up check: Phase 3 is not connected. Diagnostic data: {hex}" },  // C296 arg=6
    { UINT64_C(0x0000012800000007), "Start up check: Motor test results were invalid. Diagnostic data: {hex}" },  // C296 arg=7
    { UINT64_C(0x0000012800000009), "Start up check: Robot Voltage was present during self-diagnostics" },  // C296 arg=9
    { UINT64_C(0x000001280000000A), "Start up check: Time out during self-diagnostics" },  // C296 arg=10
    { UINT64_C(0x000001280000000B), "Start up check: Data was received while trying to disable communication" },  // C296 arg=11
    { UINT64_C(0x000001280000000C), "Start up check: Sequence number did not match expected sequence" },  // C296 arg=12
    { UINT64_C(0x000001280000000F), "Start up check: Interval between messages did not match expectations" },  // C296 arg=15
    { UINT64_C(0x0000012800000017), "Start up check: Cross-monitoring data was invalid for too long while booting" },  // C296 arg=23
    { UINT64_C(0x0000012800000019), "Start up check: Diagnostics module was initialized with a NULL pointer, for a required callback function" },  // C296 arg=25
    { UINT64_C(0x0000012800000021), "Start up check: Timeout while waiting for system to be ready to go to idle" },  // C296 arg=33
    { UINT64_C(0x0000012800000024), "Start up check: The device initialization has failed." },  // C296 arg=36
    { UINT64_C(0x0000012800000025), "Start up check: Robot Safe Torque Off Failed during self-diagnostics" },  // C296 arg=37
    { UINT64_C(0x000001280000002C), "Start up check: 24V voltage is not operating at an acceptable level, the actual value is {float}V" },  // C296 arg=44
    { UINT64_C(0x0000012900000000), "Joint validation: Critical error" },  // C297 arg=0
    { UINT64_C(0x000001290000000B), "Joint validation: The Robot arm does not match the Control Box" },  // C297 arg=11
    { UINT64_C(0x000001290000000D), "Joint validation: Joint type recevied from joint {unsigned} is invalid" },  // C297 arg=13
    { UINT64_C(0x000001290000000E), "Joint validation: Joint type received from joint {unsigned} is invalid." },  // C297 arg=14
    { UINT64_C(0x000001290000000F), "Joint validation: The type detected by the joint was {unsigned}." },  // C297 arg=15
    { UINT64_C(0x0000012900000010), "Joint validation: The expected joint type specified in the robot configuration was {unsigned}." },  // C297 arg=16
    { UINT64_C(0x0000012900000011), "Joint validation: The safety processors on joint {unsigned} disagree on the joint type." },  // C297 arg=17
    { UINT64_C(0x0000012900000012), "Joint validation: Joint type detected by processor A: {unsigned}." },  // C297 arg=18
    { UINT64_C(0x0000012900000013), "Joint validation: Joint type detected by processor B: {unsigned}." },  // C297 arg=19
    { UINT64_C(0x0000012A00000000), "Hand protection: Tool is too close to the lower arm: {float_2_4} meter." },  // C298 arg=0
    { UINT64_C(0x0000012B00000000), "Tool communication: Communication error detected" },  // C299 arg=0
    { UINT64_C(0x0000012B00000003), "Tool communication: RX framing error" },  // C299 arg=3
    { UINT64_C(0x0000012B00000004), "Tool communication: RX Parity error" },  // C299 arg=4
    { UINT64_C(0x0000012D00000000), "Safety message monitor: Critical error" },  // C301 arg=0
    { UINT64_C(0x0000012E00000001), "Tool Configuration: Invalid Robot Type {unsigned}" },  // C302 arg=1
    { UINT64_C(0x0000012F00000000), "System status: Critical error" },  // C303 arg=0
    { UINT64_C(0x0000012F00000006), "System status: Reset caused by independent watchdog" },  // C303 arg=6
    { UINT64_C(0x0000013000000000), "Self monitoring: Critical error" },  // C304 arg=0
    { UINT64_C(0x0000013000000003), "Self monitoring: Close to the gearbox shear limit. Encoders disagree {float} [rad] on the Joint position" },  // C304 arg=3
    { UINT64_C(0x0000013000000004), "Self monitoring: Either the encoder was inappropriately mounted, or the gearbox is loose or broken. Difference between the encoders is {float} [rad]." },  // C304 arg=4
    { UINT64_C(0x0000013000000006), "Self monitoring: Motor phase {unsigned}'s resistance is too high." },  // C304 arg=6
    { UINT64_C(0x0000013100000000), "Robot Power Control: Critical error" },  // C305 arg=0
    { UINT64_C(0x0000013100000001), "Robot Power Control: Power supply voltage too low" },  // C305 arg=1
    { UINT64_C(0x0000013100000002), "Robot Power Control: Robot cable not connected" },  // C305 arg=2
    { UINT64_C(0x0000013100000003), "Robot Power Control: Short circuit in Robot detected or the wrong Robot is connected to the Control Box." },  // C305 arg=3
    { UINT64_C(0x0000013100000004), "Robot Power Control: Robot voltage rising slower than expected" },  // C305 arg=4
    { UINT64_C(0x0000013100000006), "Robot Power Control: Power supply voltage too high: {float} V" },  // C305 arg=6
    { UINT64_C(0x0000013100000008), "Robot Power Control: The Robot Voltage is too high ({float})V when powering on the Robot" },  // C305 arg=8
    { UINT64_C(0x0000013100000009), "Robot Power Control: The Power State was not OFF ({unsigned}) when trying to power on the Robot" },  // C305 arg=9
    { UINT64_C(0x000001310000000B), "Robot Power Control: The power to the robot arm was not removed fast enough after violation" },  // C305 arg=11
    { UINT64_C(0x000001310000000C), "Robot Power Control: Unexpected energy eater type" },  // C305 arg=12
    { UINT64_C(0x0000013200000000), "Joint: Critical error" },  // C306 arg=0
    { UINT64_C(0x0000013200000001), "Joint: Not stopping fast enough" },  // C306 arg=1
    { UINT64_C(0x0000013200000002), "Joint: Velocity failed to pass sanity check" },  // C306 arg=2
    { UINT64_C(0x0000013200000003), "Joint: Acceleration failed to pass sanity check" },  // C306 arg=3
    { UINT64_C(0x0000013200000009), "Joint: Joint moved more than allowable limit" },  // C306 arg=9
    { UINT64_C(0x000001320000000F), "Joint: Joint moved too far while it should be stationary" },  // C306 arg=15
    { UINT64_C(0x0000013200000010), "Joint: A timeout occured while in violation parking" },  // C306 arg=16
    { UINT64_C(0x0000013200000011), "Joint: The joint parking procedure violated max allowed execution time of {float}[ms]." },  // C306 arg=17
    { UINT64_C(0x0000013200000013), "Joint: Illegal state transition, the joint entered parking while motorcontrol was disabled." },  // C306 arg=19
    { UINT64_C(0x0000013200000014), "Joint: Trying to power off from violation while motor control is enabled." },  // C306 arg=20
    { UINT64_C(0x0000013200000015), "Joint: Unexpected joint type ({hex})" },  // C306 arg=21
    { UINT64_C(0x0000013200000016), "Joint: Trying to enter idle brake failure while motor control is enabled." },  // C306 arg=22
    { UINT64_C(0x0000013200000018), "Joint: Motor unsupported by the Critical Fault brake handler" },  // C306 arg=24
    { UINT64_C(0x0000013500000000), "Keep-Alive: Critical error" },  // C309 arg=0
    { UINT64_C(0x0000013500000001), "Keep-Alive: Message with wrong sequence received from SCB-uPA." },  // C309 arg=1
    { UINT64_C(0x0000013500000004), "Keep-Alive: Message with wrong sequence received from SCB-uPB." },  // C309 arg=4
    { UINT64_C(0x000001350000000A), "Keep-Alive: Lost {unsigned} Keep-Alive message(s) in a row from Safety Control Board-uPA" },  // C309 arg=10
    { UINT64_C(0x000001350000000B), "Keep-Alive: Lost {unsigned} Keep-Alive message(s) in a row from Safety Control Board-uPB" },  // C309 arg=11
    { UINT64_C(0x000001350000000E), "Keep-Alive: Invalid command received in Keep-Alive message from SCB-uPA." },  // C309 arg=14
    { UINT64_C(0x000001350000000F), "Keep-Alive: Invalid command received in Keep-Alive message from SCB-uPB." },  // C309 arg=15
    { UINT64_C(0x0000013600000000), "Joint Temperature Manager: Critical error" },  // C310 arg=0
    { UINT64_C(0x0000013800000000), "Data validation: Critical error" },  // C312 arg=0
    { UINT64_C(0x0000013A00000000), "SPI IO: Critical error" },  // C314 arg=0
    { UINT64_C(0x0000013A00000005), "SPI IO: Expected OSSD pulse were not detected on CO{unsigned}" },  // C314 arg=5
    { UINT64_C(0x0000013A00000006), "SPI IO: An unexpected OSSD pulse was detected on CO{unsigned}" },  // C314 arg=6
    { UINT64_C(0x0000013A00000007), "SPI IO: The IO voltage is missing or below threshold" },  // C314 arg=7
    { UINT64_C(0x0000013A00000008), "SPI IO: The configurable safety outputs were not low when expected" },  // C314 arg=8
    { UINT64_C(0x0000013A0000000B), "SPI IO: Configurable IO circuit asserted its alert signal" },  // C314 arg=11
    { UINT64_C(0x0000013A0000000C), "SPI IO: Could not allocate RX and/or TX buffer" },  // C314 arg=12
    { UINT64_C(0x0000013A0000000D), "SPI IO: Configurable IO circuit reports invalid CRC or clock count error" },  // C314 arg=13
    { UINT64_C(0x0000013A00000012), "SPI IO: Timed out while waiting for conversion to complete" },  // C314 arg=18
    { UINT64_C(0x0000013A00000013), "SPI IO: Timed out while waiting for a transaction to complete" },  // C314 arg=19
    { UINT64_C(0x0000013B00000000), "Watchdog: Self-test failed" },  // C315 arg=0
    { UINT64_C(0x0000013B00000001), "Watchdog: Verification found an error at index {unsigned}" },  // C315 arg=1
    { UINT64_C(0x0000013B00000002), "Watchdog: Checked in at {float_1_3} ms which is outside the permitted window." },  // C315 arg=2
    { UINT64_C(0x0000013B00000005), "Watchdog: Keeper module ran out of space." },  // C315 arg=5
    { UINT64_C(0x0000013B00000006), "Watchdog: Trying to register checkpoint with uninitialized keeper." },  // C315 arg=6
    { UINT64_C(0x0000013B00000007), "Watchdog: Keeper failed to initialize, as it was already initialized." },  // C315 arg=7
    { UINT64_C(0x0000013B0000001C), "Watchdog: The watchdog configuration could not be applied" },  // C315 arg=28
    { UINT64_C(0x0000013C00000000), "MCU: Unknown ID" },  // C316 arg=0
    { UINT64_C(0x0000013C00000001), "MCU: This version of the firmware is obsolete and needs to be updated" },  // C316 arg=1
    { UINT64_C(0x0000013C00000009), "MCU: The device is not whitelisted" },  // C316 arg=9
    { UINT64_C(0x0000014800000002), "Transceiver miscellaneous: Flash device is not supported, JEDEC data for device is: {hex}" },  // C328 arg=2
    { UINT64_C(0x0000014900000003), "AXI STREAM: Channel {unsigned} not configured" },  // C329 arg=3
    { UINT64_C(0x0000014900000004), "AXI STREAM: Write failed, {unsigned} messages dropped" },  // C329 arg=4
    { UINT64_C(0x0000014A00000001), "IMMI IO: Injection-Molding-Machine-Interface E-Stop output readback does not match produced value: {hex}" },  // C330 arg=1
    { UINT64_C(0x0000014A00000002), "IMMI IO: Injection-Molding-Machine-Interface Moulding Area Free output readback does not match produced value: {hex}" },  // C330 arg=2
    { UINT64_C(0x0000014A00000003), "IMMI IO: Injection-Molding-Machine-Interface 24V IO voltage outside acceptable range" },  // C330 arg=3
    { UINT64_C(0x0000014A00000004), "IMMI IO: Injection-Molding-Machine-Interface 48V voltages outside acceptable range" },  // C330 arg=4
    { UINT64_C(0x0000014B00000000), "Friction model: Critical error" },  // C331 arg=0
    { UINT64_C(0x0000014B00000001), "Friction model: The velocity of the friction model is outside the limits," },  // C331 arg=1
    { UINT64_C(0x0000014B00000002), "Friction model: The current of the friction model is outside the limits" },  // C331 arg=2
    { UINT64_C(0x0000014B00000003), "Friction model: The temperature of the friction model is outside the limits" },  // C331 arg=3
    { UINT64_C(0x0000014B00000004), "Friction model: The element count of the friction model is wrong" },  // C331 arg=4
    { UINT64_C(0x0000014B00000005), "Friction model: The calibration file for the friction model validation was not found" },  // C331 arg=5
    { UINT64_C(0x0000014B00000006), "Friction model: The limit data for the friction model was not found" },  // C331 arg=6
    { UINT64_C(0x0000014C00000000), "Servo configuration: Critical error" },  // C332 arg=0
    { UINT64_C(0x0000014D00000000), "File message: Critical error" },  // C333 arg=0
    { UINT64_C(0x0000014EFFFFFFFF), "Robot deviated from constrained axes while in Constrained Freedrive." },  // C334 no arg
    { UINT64_C(0x0000015100000000), "Control parameters: Critical error" },  // C337 arg=0
    { UINT64_C(0x0000015200000000), "PROFIsafe: Critical error" },  // C338 arg=0
    { UINT64_C(0x0000015200000006), "PROFIsafe: A PROFIsafe message was received, but there is no valid configuration" },  // C338 arg=6
    { UINT64_C(0x0000015200000007), "PROFIsafe: A PROFIsafe message was received and PROFIsafe was disabled." },  // C338 arg=7
    { UINT64_C(0x0000015200000008), "PROFIsafe: The robot rejected the PROFIsafe F-Parameter set" },  // C338 arg=8
    { UINT64_C(0x000001520000000B), "PROFIsafe: An iPar CRC mismatch has been detected" },  // C338 arg=11
    { UINT64_C(0x000001520000000C), "PROFIsafe: An error occured during PROFIsafe communication" },  // C338 arg=12
    { UINT64_C(0x000001520000000D), "PROFIsafe: The PROFIsafe watchdog timed out" },  // C338 arg=13
    { UINT64_C(0x000001520000000E), "PROFIsafe: The PROFInet provider status is bad" },  // C338 arg=14
    { UINT64_C(0x0000015200000011), "PROFIsafe: Destination address mismatch" },  // C338 arg=17
    { UINT64_C(0x0000015200000012), "PROFIsafe: Destination address invalid" },  // C338 arg=18
    { UINT64_C(0x0000015200000013), "PROFIsafe: Source address mismatch" },  // C338 arg=19
    { UINT64_C(0x0000015200000014), "PROFIsafe: Source address invalid" },  // C338 arg=20
    { UINT64_C(0x0000015200000015), "PROFIsafe: Wrong PROFIsafe submodule selected" },  // C338 arg=21
    { UINT64_C(0x0000015300000000), "Cross communication: Critical error" },  // C339 arg=0
    { UINT64_C(0x0000015400000000), "Energy Monitoring: Idle power consumption too high" },  // C340 arg=0
    { UINT64_C(0x0000015400000001), "Energy Monitoring: Energy surplus shutdown" },  // C340 arg=1
    { UINT64_C(0x0000015400000002), "Energy Monitoring: Energy burst period exceeded, average power: {float}W" },  // C340 arg=2
    { UINT64_C(0x0000015400000003), "Energy Monitoring: The Energy Removal Device peak power exceeded" },  // C340 arg=3
    { UINT64_C(0x0000015400000004), "Energy Monitoring: The Energy Removal Device drew too much power over a period" },  // C340 arg=4
    { UINT64_C(0x0000015500000000), "Motor encoder: Critical error" },  // C341 arg=0
    { UINT64_C(0x0000015500000009), "Motor encoder: The scheduled motor encoder SPI package transfer did not complete before the deadline." },  // C341 arg=9
    { UINT64_C(0x000001550000000A), "Motor encoder: CRC error in CH0 transfer, frame: {hex}" },  // C341 arg=10
    { UINT64_C(0x0000015500000016), "Motor encoder: Detailed status error: Signal clipping, strong external magnetic field is present." },  // C341 arg=22
    { UINT64_C(0x0000015500000017), "Motor encoder: Detailed status warning: Signal amplitude too high. The read head is too close to the ring." },  // C341 arg=23
    { UINT64_C(0x0000015500000018), "Motor encoder: Detailed status error: ASIC was reset, external RFI glitch caused encoder to reset." },  // C341 arg=24
    { UINT64_C(0x0000015500000019), "Motor encoder: Detailed status error: ASIC synchronization lost, external RFI glitch caused malfunction." },  // C341 arg=25
    { UINT64_C(0x000001550000001A), "Motor encoder: Detailed status error: Encoder not configured properly." },  // C341 arg=26
    { UINT64_C(0x000001550000001B), "Motor encoder: Detailed status warning: Signal amplitude low. The distance between the read head and the ring is too large." },  // C341 arg=27
    { UINT64_C(0x000001550000001C), "Motor encoder: Detailed status error: Signal lost. The read head is out of alignment with the ring or the ring is damaged." },  // C341 arg=28
    { UINT64_C(0x000001550000001E), "Motor encoder: Detailed status error: Power supply error. The read head power supply voltage is out of specified range." },  // C341 arg=30
    { UINT64_C(0x000001550000001F), "Motor encoder: Detailed status error: System error. Malfunction inside the circuitry or inconsistent calibration data is detected." },  // C341 arg=31
    { UINT64_C(0x0000015500000020), "Motor encoder: Detailed status error: Magnetic pattern error. A stray magnetic field is present or metal particles are present between the read head and the ring or radial positioning between the read head and the ring is out of tolerances." },  // C341 arg=32
    { UINT64_C(0x0000015500000021), "Motor encoder: Detailed status error: Acceleration to high." },  // C341 arg=33
    { UINT64_C(0x0000015500000041), "Motor encoder: Latest parsed motor encoder position was too old." },  // C341 arg=65
    { UINT64_C(0x0000015500000056), "Motor encoder: An error occured while reading the encoder firmware version." },  // C341 arg=86
    { UINT64_C(0x0000015500000057), "Motor encoder: The encoder firmware version is not valid." },  // C341 arg=87
    { UINT64_C(0x0000015500000058), "Motor encoder: A reading of the register {hex} was attempted which is not valid for the present encoder" },  // C341 arg=88
    { UINT64_C(0x0000015500000059), "Motor encoder: A write of the register {hex} was attempted which is not valid for the present encoder" },  // C341 arg=89
    { UINT64_C(0x000001550000005A), "Motor encoder: Tried to access register {hex}, which is not valid for the present encoder" },  // C341 arg=90
    { UINT64_C(0x000001550000005C), "Motor encoder: The encoder reported that it has not performed a self-calibration" },  // C341 arg=92
    { UINT64_C(0x000001550000005F), "Motor encoder: Detailed status error: System error. Malfunction inside the circuitry." },  // C341 arg=95
    { UINT64_C(0x0000015500000060), "Motor encoder: Detailed status error: System error. Inconsistent calibration data detected." },  // C341 arg=96
    { UINT64_C(0x0000015600000000), "Motor Parameters: Module initialization error" },  // C342 arg=0
    { UINT64_C(0x0000015600000001), "Motor Parameters: Motor datasheet unavailable" },  // C342 arg=1
    { UINT64_C(0x0000015600000002), "Motor Parameters: Loading of the motor parameters file failed." },  // C342 arg=2
    { UINT64_C(0x0000015700000000), "Joint Configuration: Critical error" },  // C343 arg=0
    { UINT64_C(0x0000015700000001), "Joint Configuration: Failed to access the joint general datasheet, as the datasheet has not been loaded" },  // C343 arg=1
    { UINT64_C(0x0000015700000002), "Joint Configuration: Failed to access the joint gear datasheet, as the datasheet has not been loaded" },  // C343 arg=2
    { UINT64_C(0x0000015700000003), "Joint Configuration: Failed to access the joint motor datasheet, as the datasheet has not been loaded" },  // C343 arg=3
    { UINT64_C(0x0000015700000004), "Joint Configuration: Failed to access the joint PCBA datasheet, as the datasheet has not been loaded" },  // C343 arg=4
    { UINT64_C(0x0000015700000005), "Joint Configuration: Failed to load the joint general datasheet" },  // C343 arg=5
    { UINT64_C(0x0000015700000006), "Joint Configuration: Failed to load the joint gear datasheet" },  // C343 arg=6
    { UINT64_C(0x0000015700000007), "Joint Configuration: Failed to load the joint motor datasheet" },  // C343 arg=7
    { UINT64_C(0x0000015700000008), "Joint Configuration: Failed to load the joint PCBA datasheet" },  // C343 arg=8
    { UINT64_C(0x0000015700000018), "Joint Configuration: Unsupported joint type" },  // C343 arg=24
    { UINT64_C(0x0000015700000019), "Joint Configuration: Unsupported motor type" },  // C343 arg=25
    { UINT64_C(0x0000015700000021), "Joint Configuration: The joint have the gear type {unsigned} installed, which is not allowed" },  // C343 arg=33
    { UINT64_C(0x0000015700000022), "Joint Configuration: The joint have the motor type {unsigned} installed, which is not allowed" },  // C343 arg=34
    { UINT64_C(0x0000015700000029), "Joint Configuration: The motor type could not be read from the hardware configuration" },  // C343 arg=41
    { UINT64_C(0x000001570000002A), "Joint Configuration: Failed to access the joint type, as the joint type is still undetermined" },  // C343 arg=42
    { UINT64_C(0x000001570000002B), "Joint Configuration: The gear type could not be read from the hardware configuration" },  // C343 arg=43
    { UINT64_C(0x000001570000002E), "Joint Configuration: Failed to determine original joint type due to invalid PCB type {unsigned}" },  // C343 arg=46
    { UINT64_C(0x000001570000002F), "Joint Configuration: Failed to determine original joint type due to missing motor type information" },  // C343 arg=47
    { UINT64_C(0x0000015700000030), "Joint Configuration: Failed to determine joint type" },  // C343 arg=48
    { UINT64_C(0x0000015700000032), "Joint Configuration: The loaded firmware is too old to work with the calibration data stored in the joint" },  // C343 arg=50
    { UINT64_C(0x0000015700000033), "Joint Configuration: The loaded firmware is too old to fully utilize the calibration data stored in the joint" },  // C343 arg=51
    { UINT64_C(0x0000015B00000004), "Network Map: More robot cable extenders detected than are supported." },  // C347 arg=4
    { UINT64_C(0x0000015B00000005), "Network Map: The far end of the robot cable extender was not detected." },  // C347 arg=5
    { UINT64_C(0x0000015C00000001), "External Axes Bus: Receive error" },  // C348 arg=1
    { UINT64_C(0x0000015C00000002), "External Axes Bus: Send error" },  // C348 arg=2
    { UINT64_C(0x0000015C00000003), "External Axes Bus: Target joints calculation failed" },  // C348 arg=3
    { UINT64_C(0x0000015C00000004), "External Axes Bus: An enabled external axis at index {unsigned} is no longer receiving drive status updates" },  // C348 arg=4
    { UINT64_C(0x0000015D00000000), "Base light: Length check error, incorrect length detected for base light ring: {unsigned}" },  // C349 arg=0
    { UINT64_C(0x0000015D00000002), "Base light: Current for the red color was {float} A, which is outside the allowed range" },  // C349 arg=2
    { UINT64_C(0x0000015D00000003), "Base light: Current for the green color was {float} A, which is outside the allowed range" },  // C349 arg=3
    { UINT64_C(0x0000015D00000004), "Base light: Current for the blue color was {float} A, which is outside the allowed range" },  // C349 arg=4
    { UINT64_C(0x0000015D00000005), "Base light: Current for the yellow color was {float} A, which is outside the allowed range" },  // C349 arg=5
    { UINT64_C(0x0000015E00000000), "Failed brake system: Joint position drifted more than allowed" },  // C350 arg=0
    { UINT64_C(0x0000015E00000001), "Failed brake system: State transition is not permitted with a failed brake system" },  // C350 arg=1
    { UINT64_C(0x0000015E00000003), "Failed brake system: Failure detected in brake system" },  // C350 arg=3
    { UINT64_C(0x0000015F00000000), "Robot configuration: Unexpected tool detected" },  // C351 arg=0
    { UINT64_C(0x0000015F00000001), "Robot configuration: Unexpected {deviceName} joint detected" },  // C351 arg=1
    { UINT64_C(0x0000015F00000002), "Robot configuration: Unexpected base light detected" },  // C351 arg=2
    { UINT64_C(0x0000015F00000003), "Robot configuration: Robot cable extender detected, but not configured." },  // C351 arg=3
    { UINT64_C(0x0000015F00000004), "Robot configuration: Robot cable extender detected, but configuration is corrupted." },  // C351 arg=4
    { UINT64_C(0x0000015F00000005), "Robot configuration: Robot cable extender configured, but not detected." },  // C351 arg=5
    { UINT64_C(0x0000015F00000006), "Robot configuration: Unsupported cable type for robot cable extender configured." },  // C351 arg=6
    { UINT64_C(0x0000015F00000007), "Robot configuration: Detected cable type for robot cable extender is not supported." },  // C351 arg=7
    { UINT64_C(0x0000015F00000008), "Robot configuration: Detected cable type for robot cable extender is incompatible with configured type." },  // C351 arg=8
    { UINT64_C(0x0000015F00000009), "Robot configuration: Detected cable type for robot cable extender does not match configured type." },  // C351 arg=9
    { UINT64_C(0x0000015F0000000A), "Robot configuration: Detected cable type is {unsigned}" },  // C351 arg=10
    { UINT64_C(0x0000015F0000000B), "Robot configuration: Configured cable type is {unsigned}" },  // C351 arg=11
    { UINT64_C(0x0000015F0000000C), "Robot configuration: Unsupported robot for robot cable extender." },  // C351 arg=12
    { UINT64_C(0x0000016000000000), "Backdrive: Parking timed out" },  // C352 arg=0
    { UINT64_C(0x0000016000000001), "Backdrive: The joint moved more than allowed during the parking procedure" },  // C352 arg=1
    { UINT64_C(0x0000016000000002), "Backdrive: The joint moved more than allowed during the parking procedure" },  // C352 arg=2
    { UINT64_C(0x0000016000000003), "Backdrive: The brake solenoid was not engaged for the parking procedure. Solenoid voltage: {float}" },  // C352 arg=3
    { UINT64_C(0x0000016100000001), "IMU: IMU B selected gyro resolution not supported: {unsigned}" },  // C353 arg=1
    { UINT64_C(0x0000016100000002), "IMU: IMU B gyro verification failed: {unsigned}" },  // C353 arg=2
    { UINT64_C(0x0000016100000003), "IMU: IMU B selected acceleration resolution not supported: {unsigned}" },  // C353 arg=3
    { UINT64_C(0x0000016100000004), "IMU: IMU B accelerometer verification failed: {unsigned}" },  // C353 arg=4
    { UINT64_C(0x0000016100000005), "IMU: Timeout occurred while configuring IMU A" },  // C353 arg=5
    { UINT64_C(0x0000016100000006), "IMU: Timeout occurred while configuring IMU B" },  // C353 arg=6
    { UINT64_C(0x0000016100000007), "IMU: Unable to detect IMU A type" },  // C353 arg=7
    { UINT64_C(0x0000016100000008), "IMU: Unable to detect IMU B type" },  // C353 arg=8
    { UINT64_C(0x0000016100000009), "IMU: The ISM330IS IMU data transfer failed" },  // C353 arg=9
    { UINT64_C(0x000001610000000A), "IMU: The ICM42688 IMU data transfer failed" },  // C353 arg=10
    { UINT64_C(0x000001610000000B), "IMU: The IMU data transfer failed for IMU A" },  // C353 arg=11
    { UINT64_C(0x000001610000000C), "IMU: The IMU data transfer failed for IMU B" },  // C353 arg=12
    { UINT64_C(0x0000016200000001), "Initialization Error: The SCB has entered the fault state" },  // C354 arg=1
    { UINT64_C(0x0000016200000002), "Initialization Error: The SCB has entered the violation state" },  // C354 arg=2
    { UINT64_C(0x0000016200000003), "Initialization Error: The SCB did not reach the correct state" },  // C354 arg=3
    { UINT64_C(0x0000016200000004), "Initialization Error: Failed to send the safety configuration" },  // C354 arg=4
    { UINT64_C(0x0000016200000005), "Initialization Error: Error during boot status validation" },  // C354 arg=5
    { UINT64_C(0x0000016200000006), "Initialization Error: Powering down the robot arm took too long" },  // C354 arg=6
    { UINT64_C(0x0000016200000007), "Initialization Error: Failed to close communication devices" },  // C354 arg=7
    { UINT64_C(0x0000016200000008), "Initialization Error: Failed to open communication devices" },  // C354 arg=8
    { UINT64_C(0x0000016200000009), "Initialization Error: Failed to hotplug FPGA devices" },  // C354 arg=9
    { UINT64_C(0x000001620000000A), "Initialization Error: Failed to unload FPGA drivers" },  // C354 arg=10
    { UINT64_C(0x000001620000000B), "Initialization Error: Failed to load FPGA drivers" },  // C354 arg=11
    { UINT64_C(0x000001620000000C), "Initialization Error: The Teach Pendant does not respond" },  // C354 arg=12
    { UINT64_C(0x000001620000000D), "Initialization Error: The Injection-Molding-Machine-Interface does not respond" },  // C354 arg=13
    { UINT64_C(0x000001620000000E), "Initialization Error: The Injection-Molding-Machine-Interface_FPGA does not respond" },  // C354 arg=14
    { UINT64_C(0x000001620000000F), "Initialization Error: The SCB does not respond" },  // C354 arg=15
    { UINT64_C(0x0000016200000010), "Initialization Error: Failed to start Xillybus device scanning" },  // C354 arg=16
    { UINT64_C(0x0000016200000011), "Initialization Error: Failed to read the safety configuration" },  // C354 arg=17
    { UINT64_C(0x0000016200000012), "Initialization Error: Communication initialization failed" },  // C354 arg=18
    { UINT64_C(0x0000016300000001), "Safety API: Unknown safety system message, id {unsigned}" },  // C355 arg=1
    { UINT64_C(0x0000016300000002), "Safety API: Unknown safety system version, id {unsigned}" },  // C355 arg=2
    { UINT64_C(0x0000016300000004), "Safety API: Violation caused by safety API client" },  // C355 arg=4
    { UINT64_C(0x0000016300000005), "Safety API: Safeguard reset time is out of range, received value {unsigned}ms" },  // C355 arg=5
    { UINT64_C(0x0000016300000006), "Safety API: Retransmission of configuration is not allowed" },  // C355 arg=6
    { UINT64_C(0x0000016300000007), "Safety API: Provided an invalid configuration" },  // C355 arg=7
    { UINT64_C(0x0000016300000008), "Safety API: The execution time is greater than the period for supervisor with ID {unsigned}" },  // C355 arg=8
    { UINT64_C(0x0000016300000009), "Safety API: The superviser max execution time is {unsigned}ms" },  // C355 arg=9
    { UINT64_C(0x000001630000000A), "Safety API: The max superviser period is {unsigned}ms" },  // C355 arg=10
    { UINT64_C(0x000001630000000B), "Safety API: Restart period is outside allowed range for supervisor with ID {unsigned}" },  // C355 arg=11
    { UINT64_C(0x000001630000000C), "Safety API: The applied value is {unsigned}" },  // C355 arg=12
    { UINT64_C(0x000001630000000D), "Safety API: The upper limit is {unsigned}" },  // C355 arg=13
    { UINT64_C(0x000001630000000E), "Safety API: The lower limit is {unsigned}" },  // C355 arg=14
    { UINT64_C(0x000001630000000F), "Safety API: Supervisor period is outside allowed range for supervisor with ID {unsigned}" },  // C355 arg=15
    { UINT64_C(0x0000016300000010), "Safety API: Supervisor max execution time is outside allowed range for supervisor with ID {unsigned}" },  // C355 arg=16
    { UINT64_C(0x0000016300000011), "Safety API: Hashes are identical for supervisor with ID {unsigned}" },  // C355 arg=17
    { UINT64_C(0x0000016300000012), "Safety API: Seeds are identical for supervisor with ID {unsigned}" },  // C355 arg=18
    { UINT64_C(0x0000016300000013), "Safety API: Supervisor with ID {unsigned} is faster than the limitset supervisor" },  // C355 arg=19
    { UINT64_C(0x0000016300000014), "Safety API: The limitset supervisor period is {unsigned}ms" },  // C355 arg=20
    { UINT64_C(0x0000016300000015), "Safety API: The actual supervisor period is {unsigned}ms" },  // C355 arg=21
    { UINT64_C(0x0000016300000016), "Safety API: The limitset supervisor is not configured" },  // C355 arg=22
    { UINT64_C(0x0000016300000017), "Safety API: Timeout violation caused by supervisor with ID {unsigned}" },  // C355 arg=23
    { UINT64_C(0x0000016300000018), "Safety API: Time passed {unsigned}ms" },  // C355 arg=24
    { UINT64_C(0x0000016300000019), "Safety API: Validation of hash failed for supervisor with ID {unsigned}" },  // C355 arg=25
    { UINT64_C(0x000001630000001A), "Safety API: Actual seed {hex}" },  // C355 arg=26
    { UINT64_C(0x000001630000001B), "Safety API: Expected hash {hex}" },  // C355 arg=27
    { UINT64_C(0x000001630000001C), "Safety API: Actual hash {hex}" },  // C355 arg=28
    { UINT64_C(0x000001630000001D), "Safety API: Alternation of seed failed for supervisor with ID {unsigned}" },  // C355 arg=29
    { UINT64_C(0x000001630000001E), "Safety API: Incompatible Safety URCap" },  // C355 arg=30
    { UINT64_C(0x000001630000001F), "Safety API: Failed UUID seed validation" },  // C355 arg=31
    { UINT64_C(0x0000016300000020), "Safety API: Invalid tool speed limit, value was {float}" },  // C355 arg=32
    { UINT64_C(0x0000016300000021), "Safety API: Invalid stopping distance limit, value was {float}" },  // C355 arg=33
    { UINT64_C(0x0000016300000022), "Safety API: The received hash does not match the hash from the installation" },  // C355 arg=34
    { UINT64_C(0x0000016300000023), "Safety API: The identifier is incorrect" },  // C355 arg=35
    { UINT64_C(0x0000016300000024), "Safety API: Validation of crc failed for message ID {unsigned}" },  // C355 arg=36
    { UINT64_C(0x0000016500000000), "Too high static load on {deviceName}: Payload CoG is outside reach of robot" },  // C357 arg=0
    { UINT64_C(0x0000016500000001), "Too high static load on {deviceName}: Reorientation of tool causes load on joint to be too high" },  // C357 arg=1
    { UINT64_C(0x0000016700000000), "Report system: Missing violation handler" },  // C359 arg=0
    { UINT64_C(0x0000016700000001), "Report system: Missing critical fault handler" },  // C359 arg=1
    { UINT64_C(0x0000016900000000), "Gear torsion: Excessive gear torsion registered" },  // C361 arg=0
    { UINT64_C(0x0000016900000001), "Gear torsion: State transition is not permitted following an excessive gear torsion" },  // C361 arg=1
    { UINT64_C(0x0000016A00000001), "State Machine: Unexpected state {unsigned} encountered" },  // C362 arg=1
    { UINT64_C(0x0000016A00000002), "State Machine: Registration of state {unsigned} have failed" },  // C362 arg=2
    { UINT64_C(0x0000016B00000000), "Tool Accelerometer: Unexpected identifier {hex} read from the accelerometer" },  // C363 arg=0
    { UINT64_C(0x0000016B00000001), "Tool Accelerometer: The configuration that was read back from the accelerometer does not match the expected configuration" },  // C363 arg=1
    { UINT64_C(0x0000016B00000002), "Tool Accelerometer: Failed to write to a configuration register with data {hex}" },  // C363 arg=2
    { UINT64_C(0x0000016C00000000), "Robot validation: Robot type mismatch between configuration and arm" },  // C364 arg=0
    { UINT64_C(0x0000016C00000001), "Robot validation: Robot type disagreement in the joints" },  // C364 arg=1
    { UINT64_C(0x0000016C00000002), "Robot validation: The robot type stored in the joints is {unsigned}" },  // C364 arg=2
    { UINT64_C(0x0000016C00000003), "Robot validation: The robot type in the configuration is {unsigned}" },  // C364 arg=3
    { UINT64_C(0x0000016D00000000), "Teach Pendant RS485 communication: Critical error" },  // C365 arg=0
    { UINT64_C(0x0000016E00000000), "Tool panel interface: The panel interface output voltage of {float}V is too high while the voltage was not turned on" },  // C366 arg=0
    { UINT64_C(0x0000016E00000001), "Tool panel interface: The panel interface output voltage of {float}V is lower than allowed" },  // C366 arg=1
    { UINT64_C(0x0000016E00000002), "Tool panel interface: The panel interface output voltage of {float}V is higher than allowed" },  // C366 arg=2
    { UINT64_C(0x0000016E00000003), "Tool panel interface: A fault event was detected on supply to the panel interface" },  // C366 arg=3
    { UINT64_C(0x0000016E00000004), "Tool panel interface: An over current fault event was detected on panel interface digital outputs" },  // C366 arg=4
    { UINT64_C(0x0000016F00000000), "Power Control Board: Failed to initialize the robot power sensor" },  // C367 arg=0
    { UINT64_C(0x0000016F00000001), "Power Control Board: Failed to initialize the ERD power sensor" },  // C367 arg=1
    { UINT64_C(0x0000016F00000002), "Power Control Board: Failed to initialize the voltage monitor sensor" },  // C367 arg=2
    { UINT64_C(0x0000016F00000003), "Power Control Board: Failed to read data from the robot power sensor" },  // C367 arg=3
    { UINT64_C(0x0000016F00000004), "Power Control Board: Failed to read data from the ERD power sensor" },  // C367 arg=4
    { UINT64_C(0x0000016F00000005), "Power Control Board: Failed to read data from the voltage monitor sensor" },  // C367 arg=5
    { UINT64_C(0x0000016F00000006), "Power Control Board: Power Control Board revision invalid" },  // C367 arg=6
    { UINT64_C(0x0000016F00000007), "Power Control Board: Power Control Board revision {unsigned} is not supported" },  // C367 arg=7
    { UINT64_C(0x0000016F00000009), "Power Control Board: The voltage monitor chip is not responding" },  // C367 arg=9
    { UINT64_C(0x0000017000000000), "Power Supply Unit: Failed to communicate with power supply unit" },  // C368 arg=0
    { UINT64_C(0x0000017000000005), "Power Supply Unit: Firmware version is invalid ({hex})" },  // C368 arg=5
    { UINT64_C(0x0000017000000006), "Power Supply Unit: Hardware version is invalid ({hex})" },  // C368 arg=6
    { UINT64_C(0x0000017000000009), "Power Supply Unit: General error (status: {hex})" },  // C368 arg=9
    { UINT64_C(0x000001700000000A), "Power Supply Unit: Thermal protection active" },  // C368 arg=10
    { UINT64_C(0x000001700000000B), "Power Supply Unit: 48V constant current trigger active" },  // C368 arg=11
    { UINT64_C(0x000001700000000C), "Power Supply Unit: 24V protection active" },  // C368 arg=12
    { UINT64_C(0x000001700000000D), "Power Supply Unit: 48V protection active" },  // C368 arg=13
    { UINT64_C(0x000001700000000E), "Power Supply Unit: Input protection active" },  // C368 arg=14
    { UINT64_C(0x000001700000000F), "Power Supply Unit: 12V fault active" },  // C368 arg=15
    { UINT64_C(0x0000017000000010), "Power Supply Unit: Communication fault active" },  // C368 arg=16
    { UINT64_C(0x0000017200000000), "Ethernet MDIO: When communicating with a an ethernet PHY, the command has timed out. command: {hex}" },  // C370 arg=0
    { UINT64_C(0x0000017300000000), "End of Arm Ethernet: Unable to read status from the End of Arm Ethernet" },  // C371 arg=0
    { UINT64_C(0x0000017300000003), "End of Arm Ethernet: Unable to establish link to the Control Box" },  // C371 arg=3
    { UINT64_C(0x0000017300000004), "End of Arm Ethernet: Unable to establish link to the Tool" },  // C371 arg=4
    { UINT64_C(0x0000017300000005), "End of Arm Ethernet: Validation of PHY device with ID: {hex} failed" },  // C371 arg=5
    { UINT64_C(0x0000017500000000), "Power Interface Board: Failed to initialize the monitor sensors" },  // C373 arg=0
    { UINT64_C(0x0000017500000001), "Power Interface Board: Failed to initialize the battery interface sensor" },  // C373 arg=1
    { UINT64_C(0x0000017500000002), "Power Interface Board: Failed to read data from the battery interface sensor" },  // C373 arg=2
    { UINT64_C(0x0000017500000003), "Power Interface Board: Failed to read data from the monitor sensors" },  // C373 arg=3
    { UINT64_C(0x0000017500000004), "Power Interface Board: Power Interface Board revision invalid" },  // C373 arg=4
    { UINT64_C(0x0000017500000005), "Power Interface Board: Power Interface Board revision {unsigned} is not supported" },  // C373 arg=5
    { UINT64_C(0x0000017700000000), "Motor encoder: Critical error" },  // C375 arg=0
    { UINT64_C(0x0000017900000000), "Charging system: The motors are active while charging is enabled" },  // C377 arg=0
    { UINT64_C(0x00000190FFFFFFFF), "Elbow position close to safety plane limits" },  // C400 no arg
    { UINT64_C(0x00000191FFFFFFFF), "Exceeding user safety settings for stopping time" },  // C401 no arg
    { UINT64_C(0x00000192FFFFFFFF), "Exceeding user safety settings for stopping distance" },  // C402 no arg
    { UINT64_C(0x00000193FFFFFFFF), "Danger of clamping between the lower arm and tool flange of the robot" },  // C403 no arg
    { UINT64_C(0x0000019400000000), "Unexpected behavior: Runtime sends data too often" },  // C404 arg=0
    { UINT64_C(0x0000019400000001), "Unexpected behavior: Runtime tries to receive data too often" },  // C404 arg=1
    { UINT64_C(0x000001C200000000), "Force-Torque sensor: Sensor data invalid" },  // C450 arg=0
    { UINT64_C(0x000001C200000001), "Force-Torque sensor: Sensor can not be used, therefore it is disabled" },  // C450 arg=1
    { UINT64_C(0x000001C200000004), "Force-Torque sensor: Force-Torque sensor is expected, but it cannot be detected" },  // C450 arg=4
    { UINT64_C(0x000001C200000005), "Force-Torque sensor: Force-Torque sensor is detected but not calibrated" },  // C450 arg=5
    { UINT64_C(0x000001C300000005), "Tool Power Sensor: Failed to initialize the M8 GND sensor chip" },  // C451 arg=5
    { UINT64_C(0x000001C300000006), "Tool Power Sensor: Failed to initialize the M8 digital output low side sensor chip" },  // C451 arg=6
    { UINT64_C(0x000001C300000008), "Tool Power Sensor: Too many failing M8 GND sensor readings in a row" },  // C451 arg=8
    { UINT64_C(0x000001C300000009), "Tool Power Sensor: Too many failing M8 digital output low side sensor readings in a row" },  // C451 arg=9
    { UINT64_C(0x000001C30000000B), "Tool Power Sensor: Failed to initialize the tool power sensor chip" },  // C451 arg=11
    { UINT64_C(0x000001C30000000D), "Tool Power Sensor: Too many failing power sensor readings in a row" },  // C451 arg=13
    { UINT64_C(0x000001C500000002), "SCB IMU: Unable to validate the IMU type, received type {hex}" },  // C453 arg=2
    { UINT64_C(0x000001C500000003), "SCB IMU: Unable to initialize the IMU" },  // C453 arg=3
    { UINT64_C(0x000001C500000004), "SCB IMU: Timeout while waiting for new IMU data" },  // C453 arg=4
    { UINT64_C(0x000001C500000005), "SCB IMU: Failed to start SPI transfer for the IMU" },  // C453 arg=5
    { UINT64_C(0x000001F400000013), "Self-test step: Awaiting acceptance started" },  // C500 arg=19
    { UINT64_C(0x0000020E00000000), "Node ID: Index/node id {unsigned} is not within the range for a joint." },  // C526 arg=0
    { UINT64_C(0x0000020E00000001), "Node ID: Node {unsigned} is not a joint." },  // C526 arg=1
    { UINT64_C(0x000002C600000000), "ROM Test: Critical error" },  // C710 arg=0
    { UINT64_C(0x000002D000000006), "LVD (low voltage detection): The VCCINT voltage of {float} is too high" },  // C720 arg=6
    { UINT64_C(0x000002D000000007), "LVD (low voltage detection): The VCCINT voltage of {float} is too low" },  // C720 arg=7
    { UINT64_C(0x000002D000000008), "LVD (low voltage detection): The VCCAUX voltage of {float} is too high" },  // C720 arg=8
    { UINT64_C(0x000002D000000009), "LVD (low voltage detection): The VCCAUX voltage of {float} is too low" },  // C720 arg=9
    { UINT64_C(0x000002D00000000A), "LVD (low voltage detection): The VCCBRAM voltage of {float} is too high" },  // C720 arg=10
    { UINT64_C(0x000002D00000000B), "LVD (low voltage detection): The VCCBRAM voltage of {float} is too low" },  // C720 arg=11
    { UINT64_C(0x000002E400000000), "Hardware monitoring: Critical error" },  // C740 arg=0
    { UINT64_C(0x000002E40000000A), "Hardware monitoring: 24V IO voltage is outside of the allowed range: {float}V" },  // C740 arg=10
    { UINT64_C(0x000002E40000000B), "Hardware monitoring: 48V voltage is outside of the allowed range: {float}" },  // C740 arg=11
    { UINT64_C(0x000002E40000000F), "Hardware monitoring: The {float}A current draw of the robot is outside the allowed range." },  // C740 arg=15
    { UINT64_C(0x000002E400000014), "Hardware monitoring: 24V IO voltage is outside of the allowed range: {float}V" },  // C740 arg=20
    { UINT64_C(0x000002E400000015), "Hardware monitoring: 24V IO current is outside of the allowed range: {float}A" },  // C740 arg=21
    { UINT64_C(0x000002E400000018), "Hardware monitoring: The left Three-Position Enabling button is inconsistent" },  // C740 arg=24
    { UINT64_C(0x000002E400000019), "Hardware monitoring: The right Three-Position Enabling button is inconsistent" },  // C740 arg=25
    { UINT64_C(0x000002E40000001C), "Hardware monitoring: uA's 3V3voltage ADC ref is outside of the allowed range: {float}" },  // C740 arg=28
    { UINT64_C(0x000002E40000001D), "Hardware monitoring: The solenoid driver curcuit encountered an error condition" },  // C740 arg=29
    { UINT64_C(0x000002E40000001E), "Hardware monitoring: The brake solenoid could not be detected" },  // C740 arg=30
    { UINT64_C(0x000002E40000001F), "Hardware monitoring: The robot current offset limit check has failed" },  // C740 arg=31
    { UINT64_C(0x000002E400000020), "Hardware monitoring: The robot current offset integrity check has failed" },  // C740 arg=32
    { UINT64_C(0x000002E400000021), "Hardware monitoring: 48V voltage is below the minimum allowed limit: {float}" },  // C740 arg=33
    { UINT64_C(0x000002E400000022), "Hardware monitoring: 48V voltage is above the maximum allowed limit: {float}" },  // C740 arg=34
    { UINT64_C(0x000002E400000024), "Hardware monitoring: The M8 IO current offset measurement of {float} A is outside of the allowed range." },  // C740 arg=36
    { UINT64_C(0x000002E400000026), "Hardware monitoring: 48V Safety Control Board voltage is below the minimum allowed limit: {float}" },  // C740 arg=38
    { UINT64_C(0x000002E400000027), "Hardware monitoring: 48V Safety Control Board voltage is above the maximum allowed limit: {float}" },  // C740 arg=39
    { UINT64_C(0x000002E400000028), "Hardware monitoring: 5V1 voltage is outside of the allowed range: {float}" },  // C740 arg=40
    { UINT64_C(0x000002E400000029), "Hardware monitoring: 12VA voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=41
    { UINT64_C(0x000002E40000002A), "Hardware monitoring: 5VA voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=42
    { UINT64_C(0x000002E40000002B), "Hardware monitoring: 3V3A voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=43
    { UINT64_C(0x000002E40000002C), "Hardware monitoring: 2V5 reference voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=44
    { UINT64_C(0x000002E40000002D), "Hardware monitoring: PSU 48V voltage is outside of the allowed range: {float} V" },  // C740 arg=45
    { UINT64_C(0x000002E40000002E), "Hardware monitoring: PSU 48V current is outside of the allowed range: {float} A" },  // C740 arg=46
    { UINT64_C(0x000002E40000002F), "Hardware monitoring: PSU 24V voltage is outside of the allowed range: {float} V" },  // C740 arg=47
    { UINT64_C(0x000002E400000030), "Hardware monitoring: The 24V current reported by the PSU is outside of the allowed range: {float} A" },  // C740 arg=48
    { UINT64_C(0x000002E400000031), "Hardware monitoring: PSU 12V voltage is outside of the allowed range: {float} V" },  // C740 arg=49
    { UINT64_C(0x000002E400000032), "Hardware monitoring: PSU AC voltage is outside of the allowed range: {float} V" },  // C740 arg=50
    { UINT64_C(0x000002E400000033), "Hardware monitoring: PSU AC current is outside of the allowed range: {float} A" },  // C740 arg=51
    { UINT64_C(0x000002E400000034), "Hardware monitoring: PSU AC power is outside of the allowed range: {float} W" },  // C740 arg=52
    { UINT64_C(0x000002E400000035), "Hardware monitoring: PSU temperature is outside of the allowed range: {float} degrees C" },  // C740 arg=53
    { UINT64_C(0x000002E400000036), "Hardware monitoring: The PCB type: {unsigned} is unsupported by the SW Version." },  // C740 arg=54
    { UINT64_C(0x000002E400000037), "Hardware monitoring: Vin on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=55
    { UINT64_C(0x000002E400000038), "Hardware monitoring: 5V on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=56
    { UINT64_C(0x000002E400000039), "Hardware monitoring: 5VA on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=57
    { UINT64_C(0x000002E40000003A), "Hardware monitoring: 3V3A on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=58
    { UINT64_C(0x000002E40000003B), "Hardware monitoring: 2V5 reference on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=59
    { UINT64_C(0x000002E40000003C), "Hardware monitoring: 2V5 ISO reference on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=60
    { UINT64_C(0x000002E40000003D), "Hardware monitoring: 24V ISO on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=61
    { UINT64_C(0x000002E40000003E), "Hardware monitoring: Battery interface voltage on the Power Interface Board is outside of the allowed range: {float} V" },  // C740 arg=62
    { UINT64_C(0x000002E40000003F), "Hardware monitoring: 12VB voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=63
    { UINT64_C(0x000002E400000040), "Hardware monitoring: 5VB voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=64
    { UINT64_C(0x000002E400000041), "Hardware monitoring: 3V3B voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=65
    { UINT64_C(0x000002E400000042), "Hardware monitoring: 48V voltage on the Power Control Board is outside of the allowed range: {float}" },  // C740 arg=66
    { UINT64_C(0x000002E500000002), "Hardware general information: The hardware is not supported by this software version" },  // C741 arg=2
    { UINT64_C(0x000002E600000001), "Control Box temperature: The temperature of {signed} °C is close to the limit" },  // C742 arg=1
    { UINT64_C(0x000002E600000002), "Control Box temperature: The temperature of {signed} °C is above the limit" },  // C742 arg=2
    { UINT64_C(0x000002E700000000), "External Axes: Velocity limit for the external axis at index {unsigned} was exceeded" },  // C743 arg=0
    { UINT64_C(0x000002E700000001), "External Axes: The servo drive for the external axis at index {unsigned} has entered a fault state" },  // C743 arg=1
    { UINT64_C(0x000002E700000002), "External Axes: Freedrive is incompatible with external axis motion" },  // C743 arg=2
    { UINT64_C(0x000002E700000003), "External Axes: The servo drive for the external axis at index {unsigned} has become disabled" },  // C743 arg=3
    { UINT64_C(0x000002E700000004), "External Axes: External axis homing is incompatible with other motion" },  // C743 arg=4
    { UINT64_C(0x000002E700000005), "External Axes: Only one external axis servo drive homing operation at a time" },  // C743 arg=5
    { UINT64_C(0x000002E700000006), "External Axes: Motion detected within multiple external axis groups" },  // C743 arg=6
    { UINT64_C(0x000002E700000007), "External Axes: The position of external axis at index {unsigned} is close to joint limit" },  // C743 arg=7
    { UINT64_C(0x000002E700000008), "External Axes: The position of external axis at index {unsigned} is close to upper joint limit" },  // C743 arg=8
    { UINT64_C(0x000002E700000009), "External Axes: The position of external axis at index {unsigned} is close to lower joint limit" },  // C743 arg=9
    { UINT64_C(0x000002E70000000A), "External Axes: The position of external axis at index {unsigned} was commanded to move without the axis enabled." },  // C743 arg=10
    { UINT64_C(0x000002EA00000000), "Cable Extender: The house temperature is too low: {float} Celsius" },  // C746 arg=0
    { UINT64_C(0x000002EA00000001), "Cable Extender: The house temperature is too high: {float} Celsius" },  // C746 arg=1
    { UINT64_C(0x000002EA00000002), "Cable Extender: The energy eater temperature is too low: {float} Celsius" },  // C746 arg=2
    { UINT64_C(0x000002EA00000003), "Cable Extender: The energy eater temperature is too high: {float} Celsius" },  // C746 arg=3
    { UINT64_C(0x000002EA00000004), "Cable Extender: Cable type mismatch between near and far cable extenders" },  // C746 arg=4
    { UINT64_C(0x000002EA00000005), "Cable Extender: The energy eater regulation is inactive" },  // C746 arg=5
    { UINT64_C(0x000002EA00000007), "Cable Extender: The connected cable was not recognized" },  // C746 arg=7
    { UINT64_C(0x000002EC00000000), "Security: Library error: {unsigned}" },  // C748 arg=0
    { UINT64_C(0x000002EC00000003), "Security: Elevation aborted (bootloader is not capable)" },  // C748 arg=3
    { UINT64_C(0x000002EC00000004), "Security: Elevation aborted (key revision {unsigned} is not valid)" },  // C748 arg=4
    { UINT64_C(0x000002EC00000005), "Security: Elevation aborted (key revision {unsigned} is not supported with a non-red bootloader)" },  // C748 arg=5
    { UINT64_C(0x000002ED00000001), "Digital output: Excessive load" },  // C749 arg=1
    { UINT64_C(0x000002ED00000008), "Digital output: Internal hardware fault" },  // C749 arg=8
    { UINT64_C(0x000002ED0000001B), "Digital output: Failed to start communication with digital output device {unsigned}" },  // C749 arg=27
    { UINT64_C(0x000002ED0000001C), "Digital output: Unexpected response size from digital output device {unsigned}" },  // C749 arg=28
    { UINT64_C(0x000002ED0000001D), "Digital output: Invalid CRC from digital output device {unsigned}" },  // C749 arg=29
    { UINT64_C(0x000002ED0000001E), "Digital output: Could not update digital output state" },  // C749 arg=30
    { UINT64_C(0x000002ED0000001F), "Digital output: Failed to update digital output state" },  // C749 arg=31
    { UINT64_C(0x000002ED00000020), "Digital output: Unexpected transfer size {unsigned} when updating digital output" },  // C749 arg=32
  };
  // clang-format on
  return error_code_map;
}

}  // namespace primary_interface
}  // namespace urcl

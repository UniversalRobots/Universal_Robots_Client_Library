#!/usr/bin/env python3
# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2026 Universal Robots A/S
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# -- END LICENSE BLOCK ------------------------------------------------


"""Small helper script to start the tool communication interface."""

import subprocess
import logging
import argparse
import socket
import os

def get_args():
    # Arguments to configure socat
    arg = argparse.ArgumentParser(description="Starts socat to create a PTY symlink for the UR tool communication interface.")
    arg.add_argument("--robot-ip", default="192.168.56.101", help="Robot IP.")
    arg.add_argument("--tcp-port", type=int, default=54321, help="TCP Port.")
    arg.add_argument("--device-name", default="/tmp/ttyUR", help="PTY symlink device name.")
    return arg.parse_args()


def check_tcp(ip, port, timeout=5.0):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False

def main():
    RED = "\033[31m"
    RESET = "\033[0m"

    args = get_args()
    logging.basicConfig(level=logging.INFO, format="[%(levelname)s] %(message)s")

    # Get parameters from arguments
    robot_ip = args.robot_ip
    logging.info("Robot IP: " + robot_ip)
    tcp_port = args.tcp_port
    logging.info("TCP Port: " + str(tcp_port))
    local_device = args.device_name

    # Check IP and port reachability
    if not check_tcp(robot_ip, tcp_port):
        logging.error(
            f"{RED}Cannot reach {robot_ip}:{tcp_port}.\n"
            "Check that the IP address and port are correct.\n"
            "If so, ensure that the robot is powered on, reachable on the network, "
            f"and that external_control is running.{RESET}"
        )
        logging.info("Exiting tool communication script.")
        return

    # Check if the device_name is a directory
    if os.path.isdir(local_device):

        logging.error(
            f"{RED}'{local_device}' exists and is a directory.\n"
            "Socat needs a file path to create a PTY symlink, but it cannot replace a directory.\n"
            "Fix:\n"
            "  - Remove the directory.\n"
            f"  - Use a different device name, e.g. '--device-name /tmp/ttyUR0'. {RESET}"
        )
        logging.info("Exiting tool communication script.")
        return

    # Configure socat command
    socat_config = [
        "pty",
        f"link={local_device}",
        "raw",
        "ignoreeof",
        "waitslave",
    ]

    socat_command = [
        "socat",
        ",".join(socat_config),
        f"tcp:{robot_ip}:{tcp_port}",
    ]

    logging.info(f"Configuring PTY symlink at '{local_device}'")

    # Start socat
    try:
        logging.info("Starting socat with following command:\n" + " ".join(socat_command))
        subprocess.call(socat_command)
        logging.info("Socat terminated")

    # Error case when socat is not installed
    except FileNotFoundError:
        logging.error(f"{RED}Socat not found in PATH. Install it (e.g. apt-get install socat). {RESET}")
        logging.info("Exiting tool communication script.")
        return
    
    # Other errors
    except Exception as e:
        logging.error(f"{RED}Unexpected error launching socat: {e} {RESET}")
        logging.info("Exiting tool communication script.")
        return

    return

if __name__ == "__main__":
    main()
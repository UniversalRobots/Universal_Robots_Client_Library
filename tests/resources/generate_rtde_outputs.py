# -- BEGIN LICENSE BLOCK ----------------------------------------------
# Copyright 2025 Universal Robots A/S
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

import pathlib
import pandas as pd
import re
URCL_PATH = pathlib.Path(__file__).parent.parent.parent.resolve()

PKG_PATH = [i for i in pathlib.Path(URCL_PATH.as_posix()).glob("**/data_package.cpp")]

assert len(PKG_PATH) == 1

PKG_PATH = PKG_PATH[0].resolve()

OUTPUT_PATH = pathlib.Path(__file__).parent.resolve().as_posix() + "/exhaustive_rtde_output_recipe.txt"
WEB_OUTPUT_PATH = pathlib.Path(__file__).parent.resolve().as_posix() + "/docs_rtde_output_recipe.txt"

with open(PKG_PATH) as pkg_file:
    save_outputs = False
    outputs = []
    while True:
        line = pkg_file.readline()
        if not line:
            break
        if "// INPUT / OUTPUT" in line:
            save_outputs = True
        if save_outputs:
            if "//" not in line and len(line) > 1:
                outputs.append(line.split('"')[1] + "\n")
        if "// NOT IN OFFICIAL DOCS" in line:
            break

    with open(OUTPUT_PATH, "w") as output_file:
        output_file.writelines(outputs)

# Get outputs from official docs
page = pd.read_html("https://docs.universal-robots.com/tutorials/communication-protocol-tutorials/rtde-guide.html")
table = page[1]
outputs = []
for _, row in table.iterrows():
    name = row["Name"]
    if "X" in name:
        numbers = re.findall(r'\d+', row["Comment"])
        amount = int(numbers[0])
        base_addr = int(numbers[1])
        for i in range(amount):
            outputs.append(name[:-1]+str(base_addr+i) + "\n")
    else:
        outputs.append(name + "\n")

with open(WEB_OUTPUT_PATH, "w") as web_output_file:
    web_output_file.writelines(outputs)

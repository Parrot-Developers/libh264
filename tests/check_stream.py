#!/usr/bin/env python3

##
# Copyright (c) 2016 Parrot Drones SAS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Parrot Drones SAS Company nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT DRONES SAS COMPANY BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
##

import sys
import os
import re
import collections
import json
import subprocess
import time

#===============================================================================
#===============================================================================
class Parser(object):
    def __init__(self, progpath):
        self.progpath = progpath
        self.tracefile = None
        self.data1file = None
        self.data2file = None

    def add_data1(self, kind, name, val):
        self.data1file.write("%s: %s = %s\n" % (kind, name, val))

    def add_data2(self, kind, name, val):
        self.data2file.write("%s: %s = %s\n" % (kind, name, val))

    def parse_stream(self, inpath, data1path, data2path):
        raise NotImplementedError()

#===============================================================================
#===============================================================================
class RefParser(Parser):
    _TRACE_FILE = "trace_dec.txt"

    def __init__(self, progpath):
        Parser.__init__(self, progpath)

    def parse_stream(self, inpath, data1path, data2path):
        sys.stdout.write("RefParser %s\n" % inpath)
        start_time = time.perf_counter()
        subprocess.check_call([self.progpath, "-p", "InputFile=%s" % inpath])
        duration = time.perf_counter() - start_time
        sys.stdout.write("RefParser input parsing duration: %.3f s\n" % duration)

        start_time = time.perf_counter()
        with open(RefParser._TRACE_FILE, "r") as self.tracefile:
            with open(data1path, "w") as self.data1file:
                with open(data2path, "w") as self.data2file:
                    self._parse_trace()
        duration = time.perf_counter() - start_time
        sys.stdout.write("RefParser data extraction duration: %.3f s\n" % duration)

    def _parse_trace(self):
        reNalu = re.compile(r"^Annex B NALU w/ (short|long) startcode, len (\d+), "
                r"forbidden_bit (\d+), "
                r"nal_reference_idc (\d+), "
                r"nal_unit_type (\d+)$")
        reField = re.compile(
                r"^@(\d+)\s*(%s)\s*:\s*(\w+( ?\[\w+\])?)\s*(\d+)\s*\(\s*(-?\d+)\s*\)\s*$" %
                "|".join(["SPS", "PPS", "VUI", "SEI", "SH", "MB"]))
        reCoeff = re.compile(r"^COEFF: [^\(]*\(\d+,\d+,\d+,(\d+)\)=(-?\d+)$")
        reMbAddr = re.compile(r"^MBADDR: (\d+)$")

        _FIELD_MAP = {
            "constrained_set0_flag": "constraint_set0_flag",
            "constrained_set1_flag": "constraint_set1_flag",
            "constrained_set2_flag": "constraint_set2_flag",
            "constrained_set3_flag": "constraint_set3_flag",
            "constrained_set4_flag": "constraint_set4_flag",
            "constrained_set5_flag": "constraint_set5_flag",
            "lossless_qpprime_y_zero_flag": "qpprime_y_zero_transform_bypass_flag",
            "offset_for_ref_frame[i]": "offset_for_ref_frame",
            "num_ref_frames": "max_num_ref_frames",
            "num_reorder_frames": "max_num_reorder_frames",
            "num_ref_idx_override_flag": "num_ref_idx_active_override_flag",
            "ref_pic_list_reordering_flag_l0": "ref_pic_list_modification_flag_l0",
            "ref_pic_list_reordering_flag_l1": "ref_pic_list_modification_flag_l1",
            "luma_weight_flag_l0": "luma_weight_flag",
            "luma_weight_flag_l1": "luma_weight_flag",
            "chroma_weight_flag_l0": "chroma_weight_flag",
            "chroma_weight_flag_l1": "chroma_weight_flag",
            "luma_weight_l0": "luma_weight",
            "luma_weight_l1": "luma_weight",
            "luma_offset_l0": "luma_offset",
            "luma_offset_l1": "luma_offset",
            "chroma_weight_l0": "chroma_weight",
            "chroma_weight_l1": "chroma_weight",
            "chroma_offset_l0": "chroma_offset",
            "chroma_offset_l1": "chroma_offset",
            "adaptive_ref_pic_buffering_flag": "adaptive_ref_pic_marking_mode_flag",
            "modification_of_pic_nums_idc_l0": "modification_of_pic_nums_idc",
            "modification_of_pic_nums_idc_l1": "modification_of_pic_nums_idc",
            "abs_diff_pic_num_minus1_l0": "abs_diff_pic_num_minus1",
            "abs_diff_pic_num_minus1_l1": "abs_diff_pic_num_minus1",
            "long_term_pic_idx_l0": "long_term_pic_num",
            "long_term_pic_idx_l1": "long_term_pic_num",
            "delta_pic_order_cnt[0]": "delta_pic_order_cnt",
            "delta_pic_order_cnt[1]": "delta_pic_order_cnt",
            "run_length_minus1 [i]": "run_length_minus1",
            "top_left [i]": "top_left",
            "bottom_right [i]": "bottom_right",
            "slice_group_id[i]": "slice_group_id",
            "max_long_term_pic_idx_plus1": "max_long_term_frame_idx_plus1",
            "seq_scaling_list_present_flag": "scaling_list_present_flag",
            "pic_scaling_list_present_flag": "scaling_list_present_flag",
            "scaling_list_4x4[i]": "scaling_list_4x4",
            "scaling_list_8x8[i]": "scaling_list_8x8",
            "color_description_present_flag": "colour_description_present_flag",
            "mvd0_l0": "mvd_l0",
            "mvd0_l1": "mvd_l1",
            "mvd1_l0": "mvd_l0",
            "mvd1_l1": "mvd_l1",
            "pcm_sample_chroma_u": "pcm_sample_chroma",
            "pcm_sample_chroma_v": "pcm_sample_chroma",
        }

        for line in self.tracefile:
            line = line.rstrip("\n")

            # Field ?
            match = reField.match(line)
            if match is not None:
                kind = match.group(2)
                name = match.group(3)
                decval = match.group(6)
                name = _FIELD_MAP.get(name, name)
                if kind == "MB" and name == "ref_idx_l0" and decval == "0":
                    pass
                elif kind == "MB" and name == "ref_idx_l1" and decval == "0":
                    pass
                elif kind == "MB" and name == "pcm_alignment_zero_bit":
                    pass
                elif kind == "MB":
                    self.add_data2(kind, name, decval)
                else:
                    self.add_data1(kind, name, decval)
                continue

            # Nalu ?
            match = reNalu.match(line)
            if match is not None:
                self.add_data1("NALU", "forbidden_zero_bit", match.group(3))
                self.add_data1("NALU", "nal_ref_idc", match.group(4))
                self.add_data1("NALU", "nal_unit_type", match.group(5))
                continue

            # Macroblock address ?
            match = reMbAddr.match(line)
            if match is not None:
                val = match.group(1)
                self.add_data2("MB", "mb_addr", val)
                continue

            # Coeff ?
            match = reCoeff.match(line)
            if match is not None:
                idx = match.group(1)
                val = match.group(2)
                self.add_data2("COEFF", idx, val)
                continue

#===============================================================================
#===============================================================================
class MyParser(Parser):
    _TRACE_FILE = "out.txt"

    def __init__(self, progpath):
        Parser.__init__(self, progpath)

    def parse_stream(self, inpath, data1path, data2path):
        start_time = time.perf_counter()
        with open(MyParser._TRACE_FILE, "w") as output:
            sys.stdout.write("MyParser %s\n" % inpath)
            subprocess.check_call([self.progpath, inpath],
                    stdout=output, stderr=subprocess.STDOUT)
        duration = time.perf_counter() - start_time
        sys.stdout.write("MyParser input parsing duration: %.3f s\n" % duration)

        start_time = time.perf_counter()
        with open(MyParser._TRACE_FILE, "r") as self.tracefile:
            with open(data1path, "w") as self.data1file:
                with open(data2path, "w") as self.data2file:
                    self._parse_trace()
        duration = time.perf_counter() - start_time
        sys.stdout.write("MyParser data extraction duration: %.3f s\n" % duration)

    def _parse_trace(self):
        _KIND_MAP = {
            "nalu_header": "NALU",
            "aud": "AUD",
            "sps": "SPS",
            "pps": "PPS",
            "vui": "VUI",
            "sei": "SEI",
            "slice_header": "SH",
            "mb": "MB",
            "residual": "COEFF",
        }
        _KIND_SKIP_LIST = ["AUD"]
        _FIELD_SKIP_LIST = ["payload_type", "payload_size", "mbAddr", "MbaffFrameFlag"]

        reCoeff = re.compile(r"[^\(]+\(\d+,(\d+)\)")

        def _browse(kind, name, jentry):
            if isinstance(jentry, dict):
                for key in jentry:
                    if key in _KIND_MAP:
                        kind = _KIND_MAP[key]
                    _browse(kind, key, jentry[key])
            elif isinstance(jentry, list):
                for item in jentry:
                    _browse(kind, name, item)
            elif kind is not None and name is not None:
                if kind not in _KIND_SKIP_LIST and name not in _FIELD_SKIP_LIST:
                    if kind == "COEFF":
                        name = reCoeff.match(name).group(1)
                    if kind == "MB" and name == "ref_idx_l0" and jentry == 0:
                        pass
                    elif kind == "MB" and name == "ref_idx_l1" and jentry == 0:
                        pass
                    elif kind == "MB" or kind == "COEFF":
                        self.add_data2(kind, name, jentry)
                    else:
                        self.add_data1(kind, name, jentry)

        for line in self.tracefile:
            line = line.rstrip("\n")
            if line.startswith("[E]"):
                self.add_data1("ERR", line, None)
                self.add_data2("ERR", line, None)
            elif line.startswith("{"):
                try:
                    jline = json.loads(line, object_pairs_hook=collections.OrderedDict)
                    _browse(None, None, jline)
                except json.decoder.JSONDecodeError as ex:
                    sys.stderr.write(str(ex) + "\n")
                    sys.stderr.write(line + "\n")
                    self.add_data1("ERR", ex, line)

#===============================================================================
#===============================================================================
def main():
    ref_prog_path = sys.argv[1]
    my_prog_path = sys.argv[2]
    stream_path = sys.argv[3]

    name = os.path.basename(stream_path)
    ref_data1_path = name + ".ref_data1.txt"
    my_data1_path = name + ".my_data1.txt"
    diff_data1_path = name + ".data1.diff"
    ref_data2_path = name + ".ref_data2.txt"
    my_data2_path = name + ".my_data2.txt"
    diff_data2_path = name + ".data2.diff"

    _RUN_LIST = [
        (RefParser, ref_prog_path, ref_data1_path, ref_data2_path),
        (MyParser, my_prog_path, my_data1_path, my_data2_path),
    ]

    for run in _RUN_LIST:
        parser = run[0](os.path.abspath(run[1]))
        parser.parse_stream(stream_path, run[2], run[3])

    with open(diff_data1_path, "w") as output:
        subprocess.call(["diff", "-u", ref_data1_path, my_data1_path],
                stdout=output)
    with open(diff_data2_path, "w") as output:
        subprocess.call(["diff", "-u", ref_data2_path, my_data2_path],
                stdout=output)

#===============================================================================
#===============================================================================
if __name__ == "__main__":
    main()

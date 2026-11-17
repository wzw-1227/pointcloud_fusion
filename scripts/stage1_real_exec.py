#!/usr/bin/env python3
"""Launch prefix: validate calibration before any LIO Eigen::Map accesses."""
import argparse
import os
import sys
from stage1_tools.contracts import validate_hardware

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--config',required=True)
    args,command=parser.parse_known_args()
    try:
        validate_hardware(args.config)
        if not command:raise ValueError('No LIO executable supplied')
    except (ValueError,TypeError,KeyError) as exc:
        print('Hardware startup refused: '+str(exc),file=sys.stderr);sys.exit(1)
    os.execvp(command[0],command)

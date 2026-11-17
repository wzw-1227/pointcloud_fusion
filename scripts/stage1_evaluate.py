#!/usr/bin/env python3
import argparse
from pathlib import Path
from stage1_tools.contracts import load_contract
from stage1_tools.evaluation import evaluate_bag,write_report

def main():
    parser=argparse.ArgumentParser(description='Evaluate a recorded Stage 1 bag without re-fitting the trajectory')
    parser.add_argument('bag');parser.add_argument('--contract',required=True)
    parser.add_argument('--robot',choices=['ugv','uav','both'],default='both')
    parser.add_argument('--scenario',choices=['static','motion','height','coexist','smoke'],default='static')
    parser.add_argument('--environment',choices=['sim','real'],default='sim')
    parser.add_argument('--warmup',type=float,default=15)
    parser.add_argument('--map-review',choices=['pending','pass','fail'],default='pending')
    parser.add_argument('--output')
    args=parser.parse_args()
    contract=load_contract(args.contract,args.robot,args.environment)
    report=evaluate_bag(args.bag,contract,args.robot,args.scenario,args.warmup,args.environment,args.map_review)
    output=args.output or str(Path(args.bag).parent)
    write_report(report,output);print(report['status']+' -> '+output)
    for error in report['errors']:print('FAIL: '+error)
    return 0 if report['numeric_pass'] else 1

if __name__=='__main__':raise SystemExit(main())

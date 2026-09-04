"""Read-only numeric development metric for the bounded experiment ledger."""
import argparse
import json
import math
from pathlib import Path


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,type=Path)
    parser.add_argument('--metric',action='store_true',required=True)
    args=parser.parse_args(argv)
    state=json.loads((args.run/'incumbent.json').read_text())
    value=float(state['development_mean_loss'])
    if not math.isfinite(value) or value < 0:
        raise ValueError('invalid recorded development loss')
    print(value)
    return 0


if __name__=='__main__':
    raise SystemExit(main())

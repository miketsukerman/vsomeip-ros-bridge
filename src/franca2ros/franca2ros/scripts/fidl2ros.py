from optparse import check_builtin
from franca2ros.translator.translator import FIDLTraslator
import argparse

from pathlib import Path

def check_output_dir(output):
    path = Path(output)
    path.mkdir(parents=True, exist_ok=True)

def main():
    parser = argparse.ArgumentParser(description="FIDL to ROS IDL translator")
    parser.add_argument('-f','--fidl', type=str, nargs="+", required=True)
    parser.add_argument('-i','--import', type=str, action="append", dest="import_dirs", metavar="import_dir",  required=False)
    parser.add_argument('-o','--output', type=str, required=True)
    
    args = parser.parse_args()
    
    check_output_dir(args.output)

    for fidl in args.fidl:
        f2r = FIDLTraslator(fidl, args.import_dirs)
    
        f2r.save(args.output)

if __name__ == "__main__":
    main()
#!/usr/bin/env python

import sys
import os

def main():
    args = sys.argv
    if len(args) != 4:
        print "Usage: ", args[0], "include_dir include_prefix common_header_name"
        exit()
    include_dir = args[1]
    include_prefix = args[2]
    common_header_name = args[3]
    headers = os.listdir(include_dir + "/" + include_prefix)
    headers.sort()              # Aesthetic purposes only
    headers = [include_prefix + "/" + header for header in headers if header != common_header_name and '~' not in header and '#' not in header]
    cfile = open(include_dir + "/" + include_prefix + "/" + common_header_name, 'w')
    includes = ["#include <{0}>\n".format(header) for header in headers]
    cfile.write("// Auto-generated using generate_common_header. Do not modify\n")
    cfile.writelines(includes)
    cfile.close()
    

if __name__=='__main__':
    try:
        main()
    except IOError as e:
        print "Failed to generate common header due to IO error: ", e.strerror
        raise
    except:
        print "Unexpected error: ", sys.exc_info()
        raise
    exit(0)




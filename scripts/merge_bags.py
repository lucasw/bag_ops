#!/usr/bin/env python
# from https://www.clearpathrobotics.com/assets/downloads/support/merge_bag.py

import argparse
from fnmatch import fnmatchcase

from rosbag import Bag


def main():
    description = 'Merge one or more bag files with the possibilities of filtering topics.'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('outputbag',
                        help='output bag file with topics merged')
    parser.add_argument('inputbag', nargs='+',
                        help='input bag files')
    parser.add_argument('-v', '--verbose', action="store_true", default=True,
                        help='verbose output')
    help_text = 'string interpreted as a list of topics (wildcards \'*\' and \'?\' allowed'
    help_text += 'to include in the merged bag file'
    parser.add_argument('-t', '--topics', default="*",
                        help=help_text)
    parser.add_argument('-t0', '--t_start', default=None,
                        help="don't output any message before this time")
    parser.add_argument('-t1', '--t_end', default=None,
                        help="don't output any messages after this time")

    args = parser.parse_args()

    if args.t_start is not None:
        t_start = float(args.t_start)
    else:
        t_start = None

    if args.t_end is not None:
        t_end = float(args.t_end)
    else:
        t_end = None

    print(f"start time: {t_start}, stop time: {t_end}")

    topics = args.topics.split(' ')

    total_included_count = 0
    total_skipped_count = 0

    if (args.verbose):
        print("Writing bag file: " + args.outputbag)
        print("Matching topics against patters: '%s'" % ' '.join(topics))

    with Bag(args.outputbag, 'w') as o:
        for ifile in args.inputbag:
            matchedtopics = []
            included_count = 0
            skipped_count = 0
            if (args.verbose):
                print("> Reading bag file: " + ifile)
            with Bag(ifile, 'r') as ib:
                bag_start = ib.get_start_time()
                bag_end = ib.get_end_time()
                if t_start is not None and bag_end < t_start:
                    print(f"bag end {bag_end} < t_start {t_start}, skipping")
                    continue
                if t_end is not None and bag_start > t_end:
                    print(f"bag start {bag_start} > t_end {t_end}, skipping")
                    continue
                for topic, msg, t in ib:
                    if t_start is not None and t.to_sec() < float(t_start):
                        continue
                    if t_end is not None and t.to_sec() > float(t_end):
                        if (args.verbose):
                            print(f"done early with t_end {t_end}")
                        break
                    if any(fnmatchcase(topic, pattern) for pattern in topics):
                        if topic not in matchedtopics:
                            matchedtopics.append(topic)
                            if False:  # if (args.verbose):
                                print("Including matched topic '%s'" % topic)
                        o.write(topic, msg, t)
                        included_count += 1
                    else:
                        skipped_count += 1
            total_included_count += included_count
            total_skipped_count += skipped_count
            if (args.verbose):
                print("< Included %d messages and skipped %d" % (included_count, skipped_count))

    if (args.verbose):
        print("Total: Included %d messages and skipped %d" % (total_included_count, total_skipped_count))


if __name__ == "__main__":
    main()

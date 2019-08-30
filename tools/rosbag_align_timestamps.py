#!/usr/bin/env python

import sys
import argparse
from fnmatch import fnmatchcase

from rosbag import Bag

def main():
    """"
    Adds timestamp of one topic to corresponding message of other topic (i.e. with the same index; assumes 1:1 correspondence)
    """
    parser = argparse.ArgumentParser(
        description='Align timestamps of several topics to that of a specified topic.')
    parser.add_argument('inputbag',
                        help='input bag file')
    parser.add_argument('outputbag',
                        help='output bag file')
    parser.add_argument('correct_timestamp_topic',
                        help='topics with timestamps we wish to align to')
    parser.add_argument('-ta', '--topics_to_align', nargs='+',
                        help='topics to align')

    args = parser.parse_args()

    print(args.topics_to_align)

    count = 0
    correct_timestamps = []
    with Bag(args.outputbag, 'w') as ob:
        for topic, msg, t in Bag(args.inputbag, 'r').read_messages(topics=args.correct_timestamp_topic):
            # if topic == args.correct_timestamp_topic:
            correct_timestamps.append(t)
            print("Reading", topic, t)

        for topic, msg, t in Bag(args.inputbag, 'r').read_messages():
            # print("Writing", topic, t)
            if topic in args.topics_to_align:
                print("Writing", topic, t, " aligning timestamp from", msg.header.stamp, " to", correct_timestamps[count], end='\r')
                msg.header.stamp = correct_timestamps[count]
                ob.write(topic, msg, correct_timestamps[count])
                count += 1
            else:
                print("Copying", topic, t, " as is to output rosbag. ", end='\r')
                ob.write(topic, msg, t)

            # count += 1
            # if count > 30:
            #     break

if __name__ == "__main__":
    main()

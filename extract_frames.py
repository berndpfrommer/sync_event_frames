from rosbags.highlevel import AnyReader
from pathlib import Path
import cv2
import argparse

from rosbags.image import message_to_cvimage

def extract_frames(args):
    count = 0
    with AnyReader([Path(args.bag)]) as reader:
        # topic and msgtype information is available on .connections list
        for connection in reader.connections:
            print(connection.topic, connection.msgtype)
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == args.topic: # topic Name of images
                msg = reader.deserialize(rawdata, connection.msgtype)
                img = message_to_cvimage(msg, 'mono8') # change encoding type if needed
                fn = f"{args.output_dir}/frame%06i.png" % count
                print(f'writing frame to {fn}')
                cv2.imwrite(fn, img)
                count += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='extract frames from bag.')
    parser.add_argument('--bag', '-b', action='store', default=None,
                        required=True, help='name of rosbag')
    parser.add_argument('--topic', '-t', action='store', default="/event_camera/events/image_raw",
                        required=False, help='name of ros topic for events')
    parser.add_argument('--output_dir', '-o', action='store', default=None,
                        required=True, help='name of output directory')

    args = parser.parse_args()
    extract_frames(args)
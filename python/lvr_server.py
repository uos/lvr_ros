#!/usr/bin/env python2

from __future__ import print_function
import time
import argparse
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from os import path, makedirs, unlink, listdir, error


class TriggerFileEventHandler(FileSystemEventHandler):

    def __init__(self, config_name='remote_reconstruction_config.yaml',
                 trigger_name='.start_reconstruction', box_dir='/tmp/clouds_remote'):
        super(TriggerFileEventHandler, self).__init__()

        self.box_dir = box_dir
        self.trigger_found = self.config_found = False
        self.trigger_name = trigger_name
        self.config_name = config_name
        self.is_executing = False

    def delete_all(self):
        for f in listdir(self.box_dir):
            unlink(path.join(self.box_dir, f))

    def on_created(self, event):
        print("New file: %s" % event.src_path)

        if self.is_executing:
            return

        fname = path.basename(event.src_path)
        if fname == self.trigger_name:
            self.trigger_found = True
            print("Found trigger file.")
        if fname == self.config_name:
            self.config_found = True
            print("Found config file.")
        if self.trigger_found and self.config_found:
            self.execute()

    def execute(self):
        self.is_executing = True
        print("Now SLAMming and LVRing ... #kthxbye")


def main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument('-p', '--path', required=False, type=str, help='The directory to watch.',
                           default='/tmp/clouds_remote')
    argparser.add_argument('-d', '--destination', required=False, type=str,
                           help='The directory to watch.')
    args = argparser.parse_args()

    if not path.exists(args.path):
        print("Box directory not found. Creating...")
        try:
            makedirs(args.path)
        except error:
            print('Warning. Directory was created in the meantime. Someone else is interfering.')

    observer = Observer()
    event_handler = TriggerFileEventHandler(box_dir=args.path)
    observer.schedule(event_handler, args.path, recursive=True)
    print('Starting observation.')
    observer.start()

    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        observer.stop()
        print("Cleaning all files.")
        event_handler.delete_all()
        print('Ok bye.')

    observer.join()

    # while True:
    #     waitForTrigger(args.path)
    #     reconstruct(args.path, args.destination)
    #     sendResult(args.destination)


if __name__ == '__main__':
    main()

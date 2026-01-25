#!/usr/bin/env python3
import argparse

from livesync import Folder, sync

parser = argparse.ArgumentParser(description='Sync local code with robot.')
parser.add_argument('robot', help='Robot hostname')

args = parser.parse_args()
folders = [Folder('.', f'{args.robot}:~/feldfreund_devkit_ros')]
sync(*folders)

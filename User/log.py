

import sys
import time
import logging
import datetime
import os.path
import uuid

import serial
import structlog

log = structlog.get_logger()


def parse_line(line):
    # TODO: parse out structured information

    return {}


def open_and_log(dev, baud, timeout):

    metadata = {}

    with serial.Serial(dev, baud, timeout=timeout) as ser:

        log.info("open device")
        # discard old data
        ser.reset_input_buffer()

        while True:
            lines = ser.readlines()
            for data in lines:
                try:
                    line = data.decode('utf-8')
                except UnicodeDecodeError as e:
                    log.warning('decode-error', error=e, data=data)
                event = parse_line(line)
                log.debug('line-received', line=line)

            log.debug('readlines', lines=len(lines))


def parse():
    import argparse
    parser = argparse.ArgumentParser("Open the log of device")

    parser.add_argument('--serial', default='/dev/ttyACM0')
    parser.add_argument('--baudrate', type=int, default=460800)
    parser.add_argument('--timeout', type=float, default=0.01)
    parser.add_argument('--logs', type=str, default='logs')
    parser.add_argument('--retry', type=float, default='2.0', help="Serial retry time")

    return parser.parse_args()


def main():
    args = parse()

    dev = args.serial
    baud = args.baudrate
    timeout = args.timeout
    retry = args.retry

    while True:
        try:
            open_and_log(dev, baud, timeout)
        except serial.serialutil.SerialException as e:
            log.info("serial read failed", error=e, devicepath=dev)
            if retry > 0.0:
                time.sleep(2.0)
            else:
                log.info("quit", reason="retry disabled")        
                break


if __name__ == '__main__':
    main()

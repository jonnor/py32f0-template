

import sys
import time
import logging
import datetime
import os.path
import uuid
import queue
import argparse
import base64

import serial
import structlog
import sounddevice
import numpy

log = structlog.get_logger()

def parse_logfmt_key_values(s : str, delim=' ', kv_delim='=') -> dict[str, str]:
    """
    https://www.brandur.org/logfmt
    """    

    out = {}
    parts = s.split(delim)
    #print('pp', parts, s)
    for tok in parts:
        #print('tt', tok)
        split = tok.split(kv_delim)
        # key may not contain kv_delim - but value might
        key = split[0]
        value = kv_delim.join(split[1:])
        #if tok.endswith(kv_delim):
        #    value += kv_delim

        out[key] = value

    return out

def parse_line(line: str) -> dict:
    """
    Parse a structured log line which has EVENT followed by logfmt key-value pairs

    Example:

    event keya=Aaa key2=23234
    """


    tok = line.strip('\r\n').split(' ')
    event = tok[0]
    values = ' '.join(tok[1:]) + ' foo=bar'
    if event == 'audio-block':
        e = parse_logfmt_key_values(values)
        try:
            e['sequence_no'] = int(e['seq'])
            d = base64.b64decode(e['data'])
            d = numpy.frombuffer(d, dtype='>i2')
            e['samples'] = d
            e['event'] = event

            #log.debug('decode-audio-block', eventname=event, values=values, kvs=e)
            return e
        except Exception as e:
            log.warning('decode-audio-block', error=e)
            return {}

        
    else:
        return {}


class DataReceiver():
    """
    Receive and process data from serial
    
    Audio chunks will be written to the specified audio device.
    Can be an actual output device for playback on speakers/headphone.
    Or a loopback device, which enables recording/processing using a standard program.
    """

    def __init__(self, serial,
            audio_device,
            read_timeout=0.1,
            baudrate=921600,
        ):
        self.queue_capacity = 20
        self.buffer_blocks = self.queue_capacity // 2
        self.samplerate = 44100
        self.blocksize = 64
        self.serial = serial
        self.read_timeout = read_timeout
        self.baudrate = baudrate
        self.audio_device = audio_device
        self.channels = 1
        self.audio_dtype = 'int16'

        self.audio_queue = queue.Queue(maxsize=self.queue_capacity)
        self.output_stream = None
        self.serial_device = None
        
        sounddevice.check_input_settings(device=self.audio_device,
            channels=self.channels, dtype=self.audio_dtype, samplerate=self.samplerate)


    def _audio_device_callback(self, outdata, frames, time, status):
        """
        SoundDevice callback.

        Should fill outdata with audio from self.audio_queue
        """
        try:
            log.debug('device-callback', status=status, time=time)

            assert frames == self.blocksize
            if status.output_underflow:
                log.error('output-underflow', time=time)

            assert not status
            try:
                data = self.audio_queue.get_nowait()
            except queue.Empty as e:
                log.error('queue-underflow', time=time)
                data = numpy.zeros(frames)

            assert len(data) == len(outdata)
            outdata[:] = data

        except Exception as e:
            print(e)
            if self.output_stream:
                self.output_stream.stop()
                self.output_stream = None

    def _add_audio_chunk(self, samples):

        log.info('add-audio-chunk',
            n_samples=len(samples),
        )

        # FIXME: assert the data format on input
        self.audio_queue.put_nowait(samples)

        if self.output_stream is None:
            # we are buffering
            if self.audio_queue.qsize() >= self.buffer_blocks:
                log.info('audio-buffering-completed',
                    queue_length=self.audio_queue.qsize(),
                )
                # have buffered enough, start the stream
                self.output_stream = sounddevice.OutputStream(
                    samplerate=self.samplerate,
                    blocksize=self.blocksize,
                    device=self.audio_device,
                    channels=self.channels,
                    dtype=self.audio_dtype,
                    callback=self._audio_device_callback,
                )
                self.output_stream.start()
                log.info('audio-stream-up',
                    device=self.audio_device,
                )
        else:
            # stream already up, nothing needed
            pass


    def run_forever(self):

        self.serial_device = serial.Serial(self.serial,
            self.baudrate, timeout=self.read_timeout)

        # discard old data
        self.serial_device.reset_input_buffer()
        log.info("open-serial", device=self.serial)

        while True:
            lines = self.serial_device.readlines()
            #log.debug('readlines', lines=len(lines))

            for data in lines:
                try:
                    line = data.decode('utf-8')
                except UnicodeDecodeError as e:
                    log.warning('decode-error', error=e, data=data)

                log.debug('line-received', line=line)

                event = parse_line(line)

                event_type = event.get('event', None)
                if event_type == 'audio-block':
                    self._add_audio_chunk(event['data'])



def int_or_str(text):
    """Helper function for argument parsing."""
    try:
        return int(text)
    except ValueError:
        return text

def parse():

    # Handle list-devices option
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--list-devices', action='store_true',
        help='show list of audio devices and exit')
    args, remaining = parser.parse_known_args()
    if args.list_devices:
        print(sounddevice.query_devices())
        parser.exit(0)

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
        parents=[parser])

    parser.add_argument(
        '--device', type=int_or_str,
        help='output device (numeric ID or substring)')
    parser.add_argument(
        '--blocksize', type=int, default=1024,
        help='block size (default: %(default)s)')
    parser.add_argument(
        '--buffersize', type=int, default=10,
        help='number of blocks used for buffering (default: %(default)s)')

    parser.add_argument('--serial', default='/dev/ttyUSB0')
    parser.add_argument('--baudrate', type=int, default=921600)
    parser.add_argument('--timeout', type=float, default=0.001)
    parser.add_argument('--logs', type=str, default='logs')
    parser.add_argument('--retry', type=float, default='2.0', help="Serial retry time")

    args = parser.parse_args(remaining)
    return args



def main():
    args = parse()

    retry = args.retry

    while True:
        try:
            receiver = DataReceiver(serial=args.serial,
                baudrate=args.baudrate,
                read_timeout=args.timeout,
                audio_device=args.device,
            )
            receiver.run_forever()
        except serial.serialutil.SerialException as e:
            log.info("serial read failed", error=e, devicepath=dev)
            if retry > 0.0:
                time.sleep(2.0)
            else:
                log.info("quit", reason="retry disabled")        
                break


if __name__ == '__main__':
    main()

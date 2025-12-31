#! /usr/bin/env python3
import os

os.environ["OMP_NUM_THREADS"] = "1"
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["VECLIB_MAXIMUM_THREADS"] = "1"
os.environ["NUMEXPR_NUM_THREADS"] = "1"

import time
import h264decoder

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

import numpy as np
import socket
import struct
import cv2
import base64
import random

cv2.setNumThreads(0)

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{message}]'


def dummy(bla):
    pass


class RTSPClient:
    def __init__(self, host, username, password, print_function=print):
        self.host = host
        self.username = username
        self.password = password

        self.url = f"rtsp://{self.host}/axis-media/media.amp?videocodec=h264"

        self.set_rtsp_params()
        self.set_rtp_params()

        self.rtsp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.rtsp_socket.settimeout(5)
        self.rtsp_socket.connect((self.host, 554))

        # self.print_function = dummy # print_function
        self.print_function = print_function

    def set_rtsp_params(self):
        self.rtsp_version = None
        self.status_code = None
        self.status_text = None
        self.sequence_ack = None
        self.date = None
        self.methods_ack = None
        self.basic = False
        self.realm = None
        self.digest = True
        self.realm = None
        self.nonce = None
        self.stale = None
        self.content_type = None
        self.content_base = None
        self.content_length = None
        self.session_id_value = None
        self.session_timeout = None
        self.transport_ack = None
        self.range = None
        self.rtp_info = None
        self.sdp = None
        self.control_url = None
        self.sdp_sps = None
        self.sdp_pps = None
        self.sequence_val = 1
        self.authorized = False
        self.methods = [
            "OPTIONS",
            "DESCRIBE",
            "SETUP",
            "PLAY",
            "TEARDOWN",
        ]

    def set_rtp_params(self):
        self.unpack_rtp_header = struct.Struct('!BBHII').unpack
        self.unpack_extension_header_len = struct.Struct('!H').unpack
        self.unpack_nal_header_in_rtp = struct.Struct('!BB').unpack
        self.unpack_rtcp = struct.Struct('!III').unpack

        self.rtp_count_rtp = 0
        self.previous_rtp_rtp = 0
        self.previous_utc_rtp = 0

        self.rtp_count_rtcp = 0
        self.previous_rtp_rtcp = 0
        self.rtp_timestamp_from_rtcp = 0
        self.ntp_timestamp_from_rtcp = 0
        self.mhz = 0

        self.sequence_number_old = 1
        h264decoder.disable_logging()
        self.decoder = h264decoder.H264Decoder()

    @property
    def sequence(self):
        """Generate sequence string."""
        return f"CSeq: {str(self.sequence_val)}\r\n"

    def generate_digest(self):
        """RFC 2617."""
        from hashlib import md5

        ha1 = f"{self.username}:{self.realm}:{self.password}"
        HA1 = md5(ha1.encode("UTF-8")).hexdigest()
        ha2 = f"{self.methods[self.sequence_val - 1]}:{self.url_for_auth}"
        HA2 = md5(ha2.encode("UTF-8")).hexdigest()
        encrypt_response = f"{HA1}:{self.nonce}:{HA2}"
        response = md5(encrypt_response.encode("UTF-8")).hexdigest()

        digest_auth = "Digest "
        digest_auth += f'username="{self.username}", '
        digest_auth += f'realm="{self.realm}", '
        digest_auth += 'algorithm="MD5", '
        digest_auth += f'nonce="{self.nonce}", '
        digest_auth += f'uri="{self.url_for_auth}", '
        digest_auth += f'response="{response}"'
        return digest_auth

    def generate_basic(self):
        """RFC 2617."""
        from base64 import b64encode

        credentials = f"{self.username}:{self.password}"
        basic_auth = "Basic "
        basic_auth += b64encode(credentials.encode("UTF-8")).decode("UTF-8")
        return basic_auth

    @property
    def authentication(self):
        """Generate authentication string."""
        if self.digest:
            authentication = self.generate_digest()
        elif self.basic:
            authentication = self.generate_basic()
        else:
            return ""
        return f"Authorization: {authentication}\r\n"

    @property
    def user_agent(self):
        """Generate user-agent string."""
        return f"User-Agent: {self.username}\r\n"

    @property
    def session_id(self):
        """Generate session string."""
        if self.session_id_value:
            return f"Session: {self.session_id_value}\r\n"
        return ""

    @property
    def transport(self):
        """Generate transport string."""
        # return f"Transport: RTP/AVP;unicast;client_port={self.rtp_port}-{self.rtcp_port}\r\n"
        return "Transport: RTP/AVP/TCP;unicast;interleaved=0-1\r\n"

    def OPTIONS(self, authenticate=True):
        """Request options device supports."""
        self.url_for_auth = self.url
        message = f"OPTIONS {self.url} RTSP/1.0\r\n"
        message += self.sequence
        message += self.authentication if authenticate else ""
        message += self.user_agent
        message += self.session_id
        message += "\r\n"
        return message

    def DESCRIBE(self):
        """Request description of what services RTSP server make available."""
        self.url_for_auth = self.url
        message = f"DESCRIBE {self.url} RTSP/1.0\r\n"
        message += self.sequence
        message += self.authentication
        message += self.user_agent
        message += "Accept: application/sdp\r\n"
        message += "\r\n"
        return message

    def SETUP(self):
        """Set up stream transport."""
        self.url_for_auth = self.control_url
        message = f"SETUP {self.control_url} RTSP/1.0\r\n"
        message += self.sequence
        message += self.authentication
        message += self.user_agent
        message += self.transport
        message += "\r\n"
        return message

    def PLAY(self):
        """RTSP session is ready to send data."""
        self.url_for_auth = self.url
        message = f"PLAY {self.url} RTSP/1.0\r\n"
        message += self.sequence
        message += self.authentication
        message += self.user_agent
        message += self.session_id
        message += "\r\n"
        return message

    def TEARDOWN(self):
        """Tell device to tear down session."""
        message = f"TEARDOWN {self.url} RTSP/1.0\r\n"
        message += self.sequence
        message += self.authentication
        message += self.user_agent
        message += self.session_id
        message += "\r\n"
        return message

    def update(self, response):
        """Update session information from device response.
        Increment sequence number when starting stream, not when playing.
        If device requires authentication resend previous message with auth.
        """
        # self.print_function(response)
        data = response.splitlines()
        while data:
            line = data.pop(0).decode("utf-8")
            if "RTSP/1.0" in line:
                self.rtsp_version = int(line.split(" ")[0][5])
                self.status_code = int(line.split(" ")[1])
                self.status_text = line.split(" ")[2]
            elif "CSeq:" in line:
                self.sequence_ack = int(line.split(": ")[1])
            elif "Date:" in line:
                self.date = line.split(": ")[1]
            elif "Public:" in line:
                self.methods_ack = line.split(": ")[1].split(", ")
            elif "Server:" in line:
                self.server_text = line.split(": ")[1]
            elif "WWW-Authenticate: Basic" in line:
                self.basic = True
                self.realm = line.split('"')[1]
            elif "WWW-Authenticate: Digest" in line:
                self.digest = True
                self.realm = line.split('"')[1]
                self.nonce = line.split('"')[3]
                self.stale = line.split("stale=")[1] == "TRUE"
            elif "Content-Type:" in line:
                self.content_type = line.split(": ")[1]
            elif "Content-Base:" in line:
                self.content_base = line.split(": ")[1]
            elif "Content-Length:" in line:
                self.content_length = int(line.split(": ")[1])
            elif "Session:" in line:
                self.session_id_value = line.split(": ")[1].split(";")[0]
                if "=" in line:
                    self.session_timeout = int(line.split(": ")[1].split("=")[1])
            elif "Transport" in line:
                self.transport_ack = line.split(": ")[1]
            elif "Range" in line:
                self.range = line.split(": ")[1]
            elif "RTP-Info" in line:
                self.rtp_info = line.split(": ")[1]
            # from here on SPD
            elif "a=control:rtsp" in line:
                self.control_url = line.split(":", 1)[1]
            elif "sprop-parameter-sets" in line:
                sub_params = line.split(';')
                for sub_param in sub_params:
                    key = sub_param.split('=')[0]
                    if key == "sprop-parameter-sets":
                        sps_pps = sub_param[21:].split(',')
                        self.sdp_sps = sps_pps[0]
                        self.sdp_pps = sps_pps[1]
                        nal_sdp_sps = bytes.fromhex("00000001" + base64.b64decode(self.sdp_sps).hex())
                        nal_sdp_pps = bytes.fromhex("00000001" + base64.b64decode(self.sdp_pps).hex())
                        _ = self.decoder.decode(nal_sdp_sps)
                        _ = self.decoder.decode(nal_sdp_pps)
                        break

        if self.status_code == 200:
            self.sequence_val += 1
            self.authorized = True
        elif self.status_code == 401:
            self.authorized = False
        else:
            # If device configuration is correct we should never get here
            self.print_function(f"{self.host} RTSP {self.status_code} {self.status_text}")

    def run_command(self, command_function):
        while True:
            message = bytes(command_function(), 'UTF-8')
            # self.print_function(message)
            self.rtsp_socket.send(message)
            self.update(self.rtsp_socket.recv(2048))
            if self.authorized:
                break

    def start_rtsp(self):
        self.run_command(self.OPTIONS)
        self.run_command(self.DESCRIBE)
        self.run_command(self.SETUP)
        self.run_command(self.PLAY)

    def stop_rtsp(self):
        self.rtsp_socket.send(bytes(self.TEARDOWN(), 'UTF-8'))
        self.rtsp_socket.close()

    @staticmethod
    def parse_interleaved_packet(data_buffer):
        """
        Parse the interleaved RTSP packet to extract RTP or RTCP packets.
        """

        # First byte must be 0x24 (the dollar sign '$')
        if data_buffer[0] != 0x24:
            raise ValueError("Invalid interleaved RTSP packet")

        # Second byte is the channel identifier (0 for RTP, 1 for RTCP in typical cases)
        channel = data_buffer[1]

        # Third and fourth bytes (2-3) are the length of the RTP/RTCP packet
        length = (data_buffer[2] << 8) | data_buffer[3]
        # self.print_function(length)

        # Check if the entire packet has been received
        if len(data_buffer) < 4 + length:
            # Not enough data, return None and keep the buffer intact
            return None, None, data_buffer

        # Extract the actual RTP/RTCP packet
        packet = data_buffer[4:4 + length]

        return channel, packet, data_buffer[4 + length:]

    def process_rtp_packet(self, rtp_packet):
        # RTP header is typically 12 bytes
        rtp_header = rtp_packet[:12]

        # Unpack the RTP header
        header = self.unpack_rtp_header(rtp_header)

        # Decode the fields
        version = (header[0] >> 6) & 0x03
        padding = (header[0] >> 5) & 0x01
        extension = (header[0] >> 4) & 0x01
        csrc_count = header[0] & 0x0F
        marker = (header[1] >> 7) & 0x01
        payload_type = header[1] & 0x7F
        sequence_number = header[2]
        rtp_timestamp = header[3]
        ssrc = header[4]

        # CSRC list size (each CSRC is 4 bytes)
        csrc_list_size = csrc_count * 4

        # Adjust index to the payload start after CSRCs and possible extension headers
        payload_start = 12 + csrc_list_size
        if extension:
            # Unpack extension header (if present)
            extension_header_len = self.unpack_extension_header_len(rtp_packet[payload_start + 2:payload_start + 4])[0]
            payload_start += 4 + extension_header_len * 4

        # Extract the H.264 payload directly
        payload = rtp_packet[payload_start:]

        # Unpack nal header in rtp
        byte1, byte2 = self.unpack_nal_header_in_rtp(payload[:2])

        # Extract NAL type
        nal_type = byte1 & 0b00011111

        # Initialize nal header
        nal_header = b'\x00\x00\x00\x01'

        # Non-IDR slice  P- or B- frames
        if nal_type == 1:
            nal_packet = nal_header + payload

        # SPS packet
        elif nal_type == 7:
            nal_packet = nal_header + payload

        # PPS packet
        elif nal_type == 8:
            nal_packet = nal_header + payload

        # FU-A (Fragmentation Unit)
        elif nal_type == 28:

            # extract first bit
            start_bit = (byte2 & 0b10000000) >> 7

            # start of a new fragment
            if start_bit:
                middle_data = bytes(((byte1 & 0b11100000) + (byte2 & 0b00011111),))
                nal_packet = nal_header + middle_data + payload[2:]

            # continuation of the old fragment
            else:
                nal_packet = payload[2:]

        # NAL type not implemented
        else:
            raise NotImplementedError("NAL type not implemented")

        frame_data_s = self.decoder.decode(nal_packet)
        for frame_data_ in frame_data_s:
            (frame, w, h, ls) = frame_data_
            if frame is not None:
                frame = np.frombuffer(frame, dtype=np.ubyte, count=len(frame))
                frame = frame.reshape((h, ls // 3, 3))
                frame = frame[:, :w, :]
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                if rtp_timestamp < self.previous_rtp_rtp:
                    self.rtp_count_rtp += 1
                self.previous_rtp_rtp = rtp_timestamp

                utc_time_in_sec = (self.ntp_timestamp_from_rtcp + self.mhz *
                                   (rtp_timestamp + self.rtp_count_rtp * pow(2, 32) - self.rtp_timestamp_from_rtcp))
                diff_time = utc_time_in_sec - self.previous_utc_rtp
                current_system_time = time.time()
                if (round(current_system_time - utc_time_in_sec, 3) > 0.05) and self.mhz:
                    self.print_function(
                        f"{self.host:<10} | "
                        f"UTC - PREV_UTC: {diff_time:>3.3f} | "
                        f"Time - UTC: {current_system_time - utc_time_in_sec:>3.3f} |"
                        f"Time: {current_system_time:>15.3f} |"
                        f"RTP: {rtp_timestamp:>15.3f} |"
                        f"RTP_RTCP: {self.rtp_timestamp_from_rtcp:>15.3f}"
                    )
                if random.random() < 0.02:
                    self.print_function(f"  {' ': >6} {self.host:<10} {utc_time_in_sec:>15.3f} {' ': >6} {' ': >6} {' ': >6} {' ': >6} {current_system_time - utc_time_in_sec:>3.3f} {diff_time:>3.3f}")

                self.previous_utc_rtp = utc_time_in_sec

                if self.mhz:
                    self.do_something_with_frame(frame, utc_time_in_sec)

        return rtp_timestamp, payload

    def do_something_with_frame(self, frame, utc_time_in_sec):
        pass

    def process_rtcp_packet(self, data):
        # self.print_function(f"RTCP: {len(data)}")
        msw, lsw, rtp_timestamp_from_rtcp = self.unpack_rtcp(data[8:20])
        ntp_timestamp_from_rtcp = (msw + lsw * 0.23283064365386962890625 / 1000000000) - 2208988800
        # seventy_years_in_seconds = 2208988800

        if self.rtp_timestamp_from_rtcp == 0:
            self.ntp_timestamp_from_rtcp, self.rtp_timestamp_from_rtcp = \
                ntp_timestamp_from_rtcp, rtp_timestamp_from_rtcp
        else:
            if rtp_timestamp_from_rtcp < self.previous_rtp_rtcp:
                self.rtp_count_rtcp += 1
            self.mhz = (self.ntp_timestamp_from_rtcp - ntp_timestamp_from_rtcp) / (
                    self.rtp_timestamp_from_rtcp - (rtp_timestamp_from_rtcp + self.rtp_count_rtcp * pow(2, 32)))
            self.mhz = 0.0001 / 9
        self.print_function(f"MHZ: {self.mhz: >1.20f}")

        self.previous_rtp_rtcp = rtp_timestamp_from_rtcp

    def read_rtsp(self):
        data_buffer = b''

        while True:
            data = self.rtsp_socket.recv(204800)
            if not data:
                break

            data_buffer += data

            while len(data_buffer) > 4:
                channel, packet, data_buffer = self.parse_interleaved_packet(data_buffer)

                if channel is None:
                    break

                if channel == 0:
                    self.process_rtp_packet(packet)
                elif channel == 1:
                    self.process_rtcp_packet(packet)


class RTSPClientOpenCV(RTSPClient):
    def __init__(self, host, username, password):
        super().__init__(host, username, password)

    def do_something_with_frame(self, frame, utc_time_in_sec):
        cv2.imshow('test', frame)
        if cv2.waitKey(1) == ord('q'):
            a = 1/0


class RTSPClientROS(RTSPClient):
    def __init__(self, node: Node, host, username, password):
        super().__init__(host, username, password, print_function=node.get_logger().info)
        self.node = node
        self.set_ros_stuff()

    def set_ros_stuff(self):
        self.compressed_image_pub = self.node.create_publisher(CompressedImage, "image_raw/compressed", 10)

    def do_something_with_frame(self, frame, utc_time_in_sec):
        sec, nsec = rclpy.time.Time(seconds=utc_time_in_sec).seconds_nanoseconds()

        compressed_img_msg = CompressedImage()
        compressed_img_msg.format = "jpeg"
        compressed_img_msg.header.stamp.sec = sec
        compressed_img_msg.header.stamp.nanosec = nsec
        compressed_img_msg.data.frombytes(np.array(cv2.imencode('.jpg', frame)[1]).tobytes())
        self.compressed_image_pub.publish(compressed_img_msg)

        if not rclpy.ok():
            a = 1/0


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('axis_camera_client')

    replay = node.declare_parameter('replay', False).value

    if replay:
        import sys
        sys.exit()
    else:
        sensor_ip = node.declare_parameter('ip', '192.168.5.41').value
        username = node.declare_parameter('username', 'root').value
        password = node.declare_parameter('password', 'Inf_MobileMast_1').value

        rtsp_client = RTSPClientROS(
            node=node,
            host=sensor_ip,
            username=username,
            password=password
        )

        try:
            rtsp_client.start_rtsp()
            rtsp_client.read_rtsp()
        finally:
            rtsp_client.stop_rtsp()


def main_opencv():
    obj = RTSPClientOpenCV('192.168.5.49', 'root', 'Inf_MobileMast_1')
    try:
        obj.start_rtsp()
        obj.read_rtsp()
    finally:
        obj.stop_rtsp()


if __name__ == "__main__":
    main_opencv() 

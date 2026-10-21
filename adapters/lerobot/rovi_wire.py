"""ROVI policy wire protocol: little-endian, length-prefixed frames. Standard library only."""

import hashlib
import struct
import sys
from array import array
from pathlib import Path

MAGIC = 0x50564F52
VERSION = 1
HELLO, REQUEST, RESPONSE, ERROR = 1, 2, 3, 4
ERROR_MALFORMED, ERROR_SHAPE, ERROR_INTERNAL = 1, 2, 3
MAX_PAYLOAD = 16 * 1024 * 1024
HEADER = struct.Struct("<IHHI")
OBSERVATION_FILE_MAGIC = b"ROVIOBS1"

if sys.byteorder != "little":
    raise RuntimeError("rovi_wire assumes a little-endian host")


class WireError(Exception):
    pass


def recv_exact(sock, size):
    buffer = bytearray(size)
    view = memoryview(buffer)
    received = 0
    while received < size:
        count = sock.recv_into(view[received:])
        if count == 0:
            raise WireError("connection closed")
        received += count
    return bytes(buffer)


def read_frame(sock):
    magic, version, kind, length = HEADER.unpack(recv_exact(sock, HEADER.size))
    if magic != MAGIC or version != VERSION or length > MAX_PAYLOAD:
        raise WireError("bad frame header")
    return kind, recv_exact(sock, length)


def write_frame(sock, kind, payload):
    if len(payload) > MAX_PAYLOAD:
        raise WireError("payload too large")
    sock.sendall(HEADER.pack(MAGIC, VERSION, kind, len(payload)))
    sock.sendall(payload)


class Reader:
    def __init__(self, data):
        self._data = memoryview(data)
        self._at = 0

    def take(self, size):
        if size < 0 or self._at + size > len(self._data):
            raise WireError("truncated payload")
        chunk = self._data[self._at:self._at + size]
        self._at += size
        return chunk

    def unpack(self, fmt):
        layout = struct.Struct("<" + fmt)
        return layout.unpack(self.take(layout.size))

    def floats(self, count):
        values = array("f")
        values.frombytes(self.take(4 * count))
        return values

    def text(self):
        (size,) = self.unpack("H")
        return bytes(self.take(size)).decode("utf-8")

    def finish(self):
        if self._at != len(self._data):
            raise WireError("trailing bytes")


def text(value):
    data = value.encode("utf-8")
    if len(data) > 0xFFFF:
        raise WireError("text too long")
    return struct.pack("<H", len(data)) + data


def encode_observation(state, width, height, rgb):
    state = array("f", state)
    if len(rgb) != width * height * 3:
        raise WireError("image size does not match its dimensions")
    return struct.pack("<I", len(state)) + state.tobytes() + struct.pack("<II", width, height) + bytes(rgb)


def decode_observation(reader):
    (state_dim,) = reader.unpack("I")
    state = reader.floats(state_dim)
    width, height = reader.unpack("II")
    rgb = bytes(reader.take(width * height * 3))
    return state, width, height, rgb


def encode_hello(policy_id, adapter_id, steps, dim, state_dim, width, height, period_ns, policy_type, task,
                 robot_type, device, runtime):
    return (policy_id + adapter_id + struct.pack("<IIIIIq", steps, dim, state_dim, width, height, period_ns)
            + text(policy_type) + text(task) + text(robot_type) + text(device) + text(runtime))


def decode_request(payload):
    reader = Reader(payload)
    request_id, observation_ns = reader.unpack("Qq")
    state, width, height, rgb = decode_observation(reader)
    reader.finish()
    return request_id, observation_ns, state, width, height, rgb


def encode_response(request_id, observation_ns, policy_id, seed, inference_ns, handling_ns, steps, dim, actions):
    actions = array("f", actions)
    if len(actions) != steps * dim:
        raise WireError("action count does not match the chunk shape")
    return (struct.pack("<Qq", request_id, observation_ns) + policy_id
            + struct.pack("<QQQII", seed, inference_ns, handling_ns, steps, dim) + actions.tobytes())


def encode_error(request_id, code, message):
    return struct.pack("<QH", request_id, code) + text(message[:1000])


def files_id(paths):
    digest = hashlib.sha256()
    for path in sorted(paths, key=lambda entry: entry.name):
        name = path.name.encode("utf-8")
        digest.update(struct.pack("<H", len(name)) + name + struct.pack("<Q", path.stat().st_size))
        with path.open("rb") as stream:
            while block := stream.read(1 << 20):
                digest.update(block)
    return digest.digest()


def policy_id(directory):
    return files_id(path for path in Path(directory).iterdir() if path.is_file())


def adapter_id():
    return files_id(Path(__file__).resolve().parent.glob("*.py"))


def write_observation_file(path, episode, frame, state, width, height, rgb):
    data = OBSERVATION_FILE_MAGIC + struct.pack("<II", episode, frame) + encode_observation(state, width, height, rgb)
    Path(path).write_bytes(data)


def read_observation_file(path):
    reader = Reader(Path(path).read_bytes())
    if bytes(reader.take(len(OBSERVATION_FILE_MAGIC))) != OBSERVATION_FILE_MAGIC:
        raise WireError("not a ROVI observation file")
    episode, frame = reader.unpack("II")
    observation = decode_observation(reader)
    reader.finish()
    return episode, frame, observation

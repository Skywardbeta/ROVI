"""Serves a LeRobot policy to ROVI over the typed wire protocol, one request at a time."""

import argparse
import hashlib
import os
import platform
import socket
import sys
import time
from pathlib import Path

import rovi_wire as wire


class StubPolicy:
    """Deterministic chunk without PyTorch or LeRobot, for protocol tests."""

    policy_type = "stub"

    def __init__(self, args):
        self.policy_id = hashlib.sha256(b"rovi-stub-policy").digest()
        self.steps, self.dim, self.state_dim = args.steps, args.dim, args.dim
        self.task = args.task
        self.device = "cpu"
        self.runtime = f"stub, python {platform.python_version()}"
        self.delay_s = args.stub_delay_ms / 1000

    def infer(self, state, width, height, rgb, seed):
        time.sleep(self.delay_s)
        return [state[j] + 0.01 * k for k in range(self.steps) for j in range(self.dim)]


class LeRobotPolicy:
    def __init__(self, args):
        # The inference host never downloads or uploads: models come from the local cache only.
        os.environ.setdefault("HF_HUB_OFFLINE", "1")
        os.environ.setdefault("HF_HUB_DISABLE_IMPLICIT_TOKEN", "1")
        import numpy as np
        import torch
        from lerobot.configs.policies import PreTrainedConfig
        from lerobot.policies.factory import get_policy_class, make_pre_post_processors
        from lerobot.policies.utils import prepare_observation_for_inference

        self.np, self.torch, self.prepare = np, torch, prepare_observation_for_inference
        checkpoint = Path(args.checkpoint)
        config = PreTrainedConfig.from_pretrained(checkpoint)
        config.device = args.device
        if config.type == "smolvla":
            config.load_vlm_weights = False
        self.policy = get_policy_class(config.type).from_pretrained(checkpoint, config=config).to(args.device).eval()
        # As lerobot-record: no rename map, so the camera is fed under the checkpoint's own image key.
        self.pre, self.post = make_pre_post_processors(
            config, pretrained_path=str(checkpoint),
            preprocessor_overrides={"device_processor": {"device": args.device},
                                    "rename_observations_processor": {"rename_map": {}}})
        self.image_key = next(key for key in config.input_features if key.startswith("observation.images."))
        self.policy_type = config.type
        self.policy_id = wire.policy_id(checkpoint)
        self.state_dim = config.input_features["observation.state"].shape[0]
        self.dim = config.output_features["action"].shape[0]
        self.steps = config.n_action_steps
        self.device = torch.device(args.device)
        self.task, self.robot_type = args.task, args.robot_type
        from importlib.metadata import version
        self.runtime = (f"lerobot {version('lerobot')}, torch {torch.__version__}, cuda {torch.version.cuda}, "
                        f"python {platform.python_version()}")

    def infer(self, state, width, height, rgb, seed):
        np, torch = self.np, self.torch
        observation = {"observation.state": np.asarray(state, dtype=np.float32),
                       self.image_key: np.frombuffer(rgb, dtype=np.uint8).reshape(height, width, 3)}
        torch.manual_seed(seed)
        with torch.inference_mode():
            batch = self.pre(self.prepare(observation, self.device, self.task, self.robot_type))
            chunk = self.post(self.policy.predict_action_chunk(batch))
        if self.device.type == "cuda":
            torch.cuda.synchronize()
        chunk = chunk.reshape(-1, self.dim)[: self.steps].float().cpu().numpy()
        return chunk.astype(np.float32).ravel().tolist()


def serve(connection, policy, args, hello):
    wire.write_frame(connection, wire.HELLO, hello)
    while True:
        kind, payload = wire.read_frame(connection)
        received = time.monotonic_ns()
        if kind != wire.REQUEST:
            wire.write_frame(connection, wire.ERROR, wire.encode_error(0, wire.ERROR_MALFORMED, "expected a request"))
            return
        try:
            request_id, observation_ns, state, width, height, rgb = wire.decode_request(payload)
        except wire.WireError as error:
            wire.write_frame(connection, wire.ERROR, wire.encode_error(0, wire.ERROR_MALFORMED, str(error)))
            return
        if len(state) != policy.state_dim or (width, height) != (args.image_width, args.image_height):
            wire.write_frame(connection, wire.ERROR, wire.encode_error(
                request_id, wire.ERROR_SHAPE, f"expected state {policy.state_dim} and image {args.image_width}x{args.image_height}"))
            continue
        try:
            start = time.monotonic_ns()
            actions = policy.infer(state, width, height, rgb, request_id)
            inference_ns = time.monotonic_ns() - start
        except Exception as error:  # A failed inference is reported, not fatal to the server.
            wire.write_frame(connection, wire.ERROR, wire.encode_error(request_id, wire.ERROR_INTERNAL, repr(error)))
            continue
        response = wire.encode_response(request_id, observation_ns, policy.policy_id, request_id, inference_ns,
                                        time.monotonic_ns() - received, policy.steps, policy.dim, actions)
        wire.write_frame(connection, wire.RESPONSE, response)
        print(f"request {request_id:#x}: inference {inference_ns / 1e6:.1f} ms", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--checkpoint", help="pretrained_model directory")
    parser.add_argument("--host", required=True, help="address to bind, e.g. the direct-link address")
    parser.add_argument("--port", type=int, default=47010)
    parser.add_argument("--task", required=True)
    parser.add_argument("--robot-type", default="so_follower")
    parser.add_argument("--device", default="cuda")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--image-width", type=int, default=640)
    parser.add_argument("--image-height", type=int, default=480)
    parser.add_argument("--stub", action="store_true")
    parser.add_argument("--steps", type=int, default=50, help="stub only")
    parser.add_argument("--dim", type=int, default=6, help="stub only")
    parser.add_argument("--stub-delay-ms", type=int, default=0, help="stub only: added reply delay")
    parser.add_argument("--port-file", help="write the bound port here, for tests that bind port 0")
    args = parser.parse_args()
    if not args.stub and not args.checkpoint:
        parser.error("--checkpoint is required unless --stub is given")

    policy = StubPolicy(args) if args.stub else LeRobotPolicy(args)
    # The first inference is slow, so it happens before any scheduled request.
    start = time.monotonic_ns()
    policy.infer([0.0] * policy.state_dim, args.image_width, args.image_height,
                 bytes(args.image_width * args.image_height * 3), 0)
    print(f"warm-up inference {(time.monotonic_ns() - start) / 1e6:.1f} ms", flush=True)
    hello = wire.encode_hello(policy.policy_id, wire.adapter_id(), policy.steps, policy.dim, policy.state_dim,
                              args.image_width, args.image_height, 1_000_000_000 // args.fps, policy.policy_type,
                              policy.task, args.robot_type, str(policy.device), policy.runtime)
    with socket.create_server((args.host, args.port), backlog=1) as server:
        port = server.getsockname()[1]
        if args.port_file:
            Path(args.port_file).write_text(f"{port}\n")
        print(f"serving {policy.policy_type} {policy.policy_id.hex()} on {args.host}:{port}", flush=True)
        while True:
            connection, peer = server.accept()
            with connection:
                connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                try:
                    serve(connection, policy, args, hello)
                except (wire.WireError, OSError) as error:
                    print(f"{peer[0]}: {error}", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()

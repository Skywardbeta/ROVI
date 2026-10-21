"""Exports recorded frames of a LeRobot dataset as ROVI observation files."""

import argparse
from pathlib import Path

import numpy as np
import pandas as pd
from lerobot.datasets.lerobot_dataset import LeRobotDataset

import rovi_wire as wire


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", required=True, help="dataset directory")
    parser.add_argument("--repo-id", required=True)
    parser.add_argument("--episode", type=int, required=True)
    parser.add_argument("--frames", required=True, help="comma-separated frame indices within the episode")
    parser.add_argument("--camera", default="observation.images.top")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    dataset = LeRobotDataset(args.repo_id, root=args.root)
    table = pd.read_parquet(Path(args.root) / "data")
    indices = table[table.episode_index == args.episode]["index"].sort_values().to_numpy()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    for frame in (int(value) for value in args.frames.split(",")):
        item = dataset[int(indices[frame])]
        image = (item[args.camera].permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)
        height, width, _ = image.shape
        path = out / f"episode{args.episode:03d}_frame{frame:05d}.rovobs"
        wire.write_observation_file(path, args.episode, frame, item["observation.state"].numpy().tolist(),
                                    width, height, image.tobytes())
        print(path)


if __name__ == "__main__":
    main()

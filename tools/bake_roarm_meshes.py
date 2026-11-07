"""Bake the Waveshare roarm_description M3 binary STLs into compact display meshes."""
import argparse
from pathlib import Path
import struct

import numpy as np


LINKS = ('base_link', 'link1', 'link2', 'link3', 'link4', 'link5', 'gripper_link')
FACET = np.dtype([('normal', '<f4', (3,)), ('points', '<f4', (3, 3)), ('flags', '<u2')])


def simplify(path, cell=2.5):
    data = path.read_bytes()
    count, = struct.unpack_from('<I', data, 80)
    if len(data) != 84 + count * FACET.itemsize:
        raise ValueError(f'Invalid binary STL: {path.name}')
    points = np.frombuffer(data, FACET, offset=84)['points'].reshape(-1, 3)
    _, index = np.unique(np.rint(points / cell).astype(np.int32), axis=0, return_inverse=True)
    index = index.reshape(-1)
    population = np.bincount(index)
    vertices = np.stack([np.bincount(index, weights=column) / population
                         for column in points.T], axis=1)
    faces = index.reshape(-1, 3)
    keys = np.sort(faces, axis=1)
    keep = (keys[:, 0] != keys[:, 1]) & (keys[:, 1] != keys[:, 2])
    _, unique = np.unique(keys[keep], axis=0, return_index=True)
    return np.round(vertices[faces[keep][unique]], 2).astype(np.float32)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stl_folder', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    np.savez_compressed(args.output, **{
        link: simplify(args.stl_folder / f'{link}.stl') for link in LINKS
    })

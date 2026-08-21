#!/usr/bin/env python3
"""Refresh the repository-owned Ravensberg geometry from OpenStreetMap."""

import argparse
import json
import math
from pathlib import Path
import urllib.request
import xml.etree.ElementTree as ET


RELATION_ID = 14671863
SOURCE_URL = (
    f'https://api.openstreetmap.org/api/0.6/relation/{RELATION_ID}/full'
)
EARTH_RADIUS_M = 6371008.8
OUTLINE_TOLERANCE_M = 10.0
SHORELINE_COLLISION_TOLERANCE_M = 30.0
ISLAND_TOLERANCE_M = 3.0
MAX_ISLANDS = 40
MIN_ISLAND_AREA_M2 = 250.0


def _project(latitude, longitude, origin):
    latitude_radians = math.radians(latitude - origin['latitude_deg'])
    longitude_radians = math.radians(longitude - origin['longitude_deg'])
    return (
        EARTH_RADIUS_M
        * longitude_radians
        * math.cos(math.radians(origin['latitude_deg'])),
        EARTH_RADIUS_M * latitude_radians,
    )


def _distance_to_segment(point, first, second):
    delta_x = second[0] - first[0]
    delta_y = second[1] - first[1]
    denominator = delta_x * delta_x + delta_y * delta_y
    if denominator == 0.0:
        return math.dist(point, first)
    ratio = max(0.0, min(1.0, (
        (point[0] - first[0]) * delta_x
        + (point[1] - first[1]) * delta_y
    ) / denominator))
    closest = (first[0] + ratio * delta_x, first[1] + ratio * delta_y)
    return math.dist(point, closest)


def _simplify_open(points, tolerance):
    if len(points) <= 2:
        return points
    distances = [
        _distance_to_segment(point, points[0], points[-1])
        for point in points[1:-1]
    ]
    maximum = max(distances, default=0.0)
    if maximum <= tolerance:
        return [points[0], points[-1]]
    split = distances.index(maximum) + 1
    return (
        _simplify_open(points[:split + 1], tolerance)[:-1]
        + _simplify_open(points[split:], tolerance)
    )


def _simplify_ring(points, tolerance):
    first_index, second_index = max(
        (
            (first, second)
            for first in range(len(points))
            for second in range(first + 1, len(points))
        ),
        key=lambda pair: math.dist(points[pair[0]], points[pair[1]]),
    )
    first_arc = points[first_index:second_index + 1]
    second_arc = points[second_index:] + points[:first_index + 1]
    simplified = (
        _simplify_open(first_arc, tolerance)[:-1]
        + _simplify_open(second_arc, tolerance)[:-1]
    )
    return [[round(x, 1), round(y, 1)] for x, y in simplified]


def _area(points):
    return abs(0.5 * sum(
        point[0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * point[1]
        for index, point in enumerate(points)
    ))


def _read_osm(url):
    request = urllib.request.Request(
        url,
        headers={'User-Agent': 'karaburan-simulation-import/1.0'},
    )
    with urllib.request.urlopen(request, timeout=30) as response:
        return ET.fromstring(response.read())


def _relation_geometry(root, origin):
    nodes = {
        node.attrib['id']: _project(
            float(node.attrib['lat']),
            float(node.attrib['lon']),
            origin,
        )
        for node in root.findall('node')
    }
    ways = {
        way.attrib['id']: [nodes[item.attrib['ref']] for item in way.findall('nd')]
        for way in root.findall('way')
    }
    relation = next(
        item for item in root.findall('relation')
        if int(item.attrib['id']) == RELATION_ID
    )
    outer = None
    inner = []
    for member in relation.findall('member'):
        points = ways[member.attrib['ref']]
        if points[0] == points[-1]:
            points = points[:-1]
        if member.attrib['role'] == 'outer':
            outer = points
        elif member.attrib['role'] == 'inner':
            inner.append((member.attrib['ref'], points))
    if outer is None:
        raise ValueError('Ravensberg relation has no outer way')
    return outer, inner


def update_config(config, root):
    """Replace source-derived geometry and return import statistics."""
    origin = config['coordinates']['origin']
    outer, inner = _relation_geometry(root, origin)
    candidates = sorted(
        (
            (_area(points), way_id, points)
            for way_id, points in inner
            if _area(points) >= MIN_ISLAND_AREA_M2
        ),
        reverse=True,
    )[:MAX_ISLANDS]
    config['source']['derivation'] = (
        'Local equirectangular projection; outer ring simplified to 10 m '
        'visual/navigation tolerance and 30 m collision tolerance; up to '
        'forty inner rings of at least 250 m2, ranked by source area, '
        'simplified to 3 m tolerance.'
    )
    config['water_outline'] = _simplify_ring(outer, OUTLINE_TOLERANCE_M)
    config['shoreline_collision_outline'] = _simplify_ring(
        outer, SHORELINE_COLLISION_TOLERANCE_M)
    config['islands'] = [
        {
            'id': f'island_{index:02d}',
            'osm_way_id': int(way_id),
            'source_area_m2': round(area, 1),
            'points': _simplify_ring(points, ISLAND_TOLERANCE_M),
        }
        for index, (area, way_id, points) in enumerate(candidates, start=1)
    ]
    config['debug'] = {'island_label_count': 12}
    config['world']['island_collision_count'] = 12
    return {
        'source_inner_ring_count': len(inner),
        'retained_island_count': len(candidates),
        'smallest_retained_area_m2': round(candidates[-1][0], 1),
        'outline_point_count': len(config['water_outline']),
        'shoreline_collision_point_count': len(
            config['shoreline_collision_outline']),
        'island_point_count': sum(
            len(island['points']) for island in config['islands']
        ),
    }


def main():
    """Refresh a Ravensberg JSON configuration in place."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('config', type=Path)
    parser.add_argument('--source-url', default=SOURCE_URL)
    arguments = parser.parse_args()
    config = json.loads(arguments.config.read_text(encoding='utf-8'))
    statistics = update_config(config, _read_osm(arguments.source_url))
    arguments.config.write_text(
        json.dumps(config, indent=2) + '\n',
        encoding='utf-8',
    )
    print(json.dumps(statistics, indent=2))


if __name__ == '__main__':
    main()

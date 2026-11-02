#!/usr/bin/env python3
"""Create a separate diagnostic profile without changing the original RViz file."""
from copy import deepcopy
from pathlib import Path
import sys


def configure(profile):
    profile = deepcopy(profile)
    profile['Visualization Manager']['Global Options']['Fixed Frame'] = 'base_link'
    changed = set()

    def visit(value):
        if isinstance(value, dict):
            topic = value.get('Topic')
            if isinstance(topic, dict):
                name = topic.get('Value')
                if name == '/wheelchair/local_costmap/costmap':
                    topic['Value'] = '/local_costmap/costmap'
                    changed.add('costmap')
                elif name == '/wheelchair/local_costmap/published_footprint':
                    topic['Value'] = '/local_costmap/published_footprint'
                    changed.add('footprint')
                elif name in ('/mid360_201/points_filtered', '/mid360_202/points_filtered'):
                    value['Enabled'] = True
                    value['Value'] = True
                    value['Decay Time'] = 0.5
                    changed.add(name)
                elif name == '/odin1/cloud_render':
                    value['Enabled'] = False
                    value['Value'] = False
                    changed.add('odin')
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(profile)
    expected = {'costmap', 'footprint', '/mid360_201/points_filtered',
                '/mid360_202/points_filtered', 'odin'}
    if changed != expected:
        raise ValueError(f'Unexpected source profile: missing={sorted(expected-changed)}')
    return profile


def main():
    import yaml
    if len(sys.argv) != 3:
        raise SystemExit('Usage: create_diagnostic_rviz.py INPUT OUTPUT')
    source, target = map(Path, sys.argv[1:])
    with source.open() as stream:
        result = configure(yaml.safe_load(stream))
    with target.open('x') as stream:
        yaml.safe_dump(result, stream, sort_keys=False, allow_unicode=True)
    print(target)


if __name__ == '__main__':
    main()

"""drift_ndt_none.yaml is drift.yaml without NDT's scan matching unit (swb roadmap 063).

Under the profile's ndt:=none nothing publishes ndt_scan_matcher's diagnostics, so a graph that
judges it keeps autonomous mode unavailable (and check_launch_args refuses it). Everything else
must stay as drift.yaml has it.
"""
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
DIAG = os.path.join(HERE, '..', 'config', 'diagnostics')
NDT_UNIT = '/autoware/localization/scan_matching_status'


def load(name):
    with open(os.path.join(DIAG, name)) as f:
        text = f.read()
    return text, yaml.safe_load(text)


def test_no_ndt_anywhere():
    text, graph = load('drift_ndt_none.yaml')
    # check_launch_args reads the file as text, comments included
    assert 'ndt_scan_matcher' not in text
    assert NDT_UNIT not in [u['path'] for u in graph['units']]


def test_localization_keeps_the_other_checks():
    _, graph = load('drift_ndt_none.yaml')
    loc = next(u for u in graph['units'] if u['path'] == '/autoware/localization')
    links = []

    def walk(item):
        if item.get('type') == 'link':
            links.append(item['link'])
        for sub in item.get('list', []):
            walk(sub)
        if 'item' in item:
            walk(item['item'])
    walk(loc)
    assert links == ['/autoware/localization/state', '/autoware/localization/accuracy',
                     '/autoware/localization/sensor_fusion_status']


def test_everything_else_is_drift_yaml():
    _, base = load('drift.yaml')
    _, none = load('drift_ndt_none.yaml')
    assert none['files'] == base['files']
    strip = {NDT_UNIT, '/autoware/localization'}
    assert [u for u in none['units'] if u['path'] not in strip] == \
        [u for u in base['units'] if u['path'] not in strip]

from __future__ import annotations

import argparse
import os
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable
from xml.sax.saxutils import escape

from ament_index_python.packages import get_package_share_directory


CYCLONEDDS_SCHEMA = (
    'https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/'
    'etc/cyclonedds.xsd'
)
LEADER_INTERNAL_DOMAIN = 10
FOLLOWER_INTERNAL_DOMAIN = 20
SHARED_DOMAIN = 30
DEFAULT_WIFI_INTERFACE = 'wlp3s0'
DEFAULT_LEADER_SVDP_IP = '192.168.0.162'
DEFAULT_FOLLOWER_SVDP_IP = '192.168.0.113'


@dataclass(frozen=True)
class DomainNetwork:
    domain_id: int | str
    interface: str
    allow_multicast: str = 'spdp'
    multicast: str = 'default'
    peers: tuple[str, ...] = ()


def _package_config_path(filename: str) -> str:
    return os.path.join(
        get_package_share_directory('platoon_bringup'),
        'config',
        filename,
    )


def _write_cyclonedds_xml(name: str, domains: Iterable[DomainNetwork]) -> str:
    path = Path(tempfile.gettempdir()) / f'{name}.xml'
    lines = [
        '<?xml version="1.0" encoding="UTF-8" ?>',
        '<CycloneDDS xmlns="https://cdds.io/config"',
        '            xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"',
        f'            xsi:schemaLocation="https://cdds.io/config {CYCLONEDDS_SCHEMA}">',
    ]
    for domain in domains:
        lines.extend(_domain_xml(domain))
    lines.append('</CycloneDDS>')
    path.write_text('\n'.join(lines) + '\n', encoding='utf-8')
    return f'file://{path}'


def _domain_xml(domain: DomainNetwork) -> list[str]:
    domain_id = escape(str(domain.domain_id))
    interface = escape(domain.interface)
    allow_multicast = escape(domain.allow_multicast)
    multicast = escape(domain.multicast)
    lines = [
        f'  <Domain Id="{domain_id}">',
        '    <General>',
        '      <Interfaces>',
        (
            f'        <NetworkInterface name="{interface}" '
            f'priority="default" multicast="{multicast}" />'
        ),
        '      </Interfaces>',
        f'      <AllowMulticast>{allow_multicast}</AllowMulticast>',
        '      <MaxMessageSize>65500B</MaxMessageSize>',
        '    </General>',
        '    <Discovery>',
        '      <EnableTopicDiscoveryEndpoints>true</EnableTopicDiscoveryEndpoints>',
        '      <ParticipantIndex>auto</ParticipantIndex>',
        '      <MaxAutoParticipantIndex>120</MaxAutoParticipantIndex>',
    ]
    if domain.peers:
        lines.append('      <Peers>')
        for peer in domain.peers:
            lines.append(f'        <Peer Address="{escape(peer)}" />')
        lines.append('      </Peers>')
    lines.extend([
        '    </Discovery>',
        '  </Domain>',
    ])
    return lines


def _base_env(domain_id: int | None, cyclone_uri: str) -> dict[str, str]:
    env = os.environ.copy()
    env['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
    env['ROS_LOCALHOST_ONLY'] = '0'
    env['CYCLONEDDS_URI'] = cyclone_uri
    if domain_id is not None:
        env['ROS_DOMAIN_ID'] = str(domain_id)
    else:
        env.pop('ROS_DOMAIN_ID', None)
    return env


def _exec_ros(args: list[str], env: dict[str, str], dry_run: bool) -> None:
    if dry_run:
        print(' '.join(args))
        print(f'ROS_DOMAIN_ID={env.get("ROS_DOMAIN_ID", "<unset>")}')
        print(f'RMW_IMPLEMENTATION={env["RMW_IMPLEMENTATION"]}')
        print(f'CYCLONEDDS_URI={env["CYCLONEDDS_URI"]}')
        return
    os.execvpe(args[0], args, env)


def _internal_parser(role: str, default_domain: int) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=f'Run {role} platoon bringup on a loopback-only internal DDS domain.',
    )
    parser.add_argument('--domain-id', type=int, default=default_domain)
    parser.add_argument('--cyclonedds-uri', default='')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def _bridge_parser(role: str, default_internal_domain: int) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            f'Run the {role} SVDP domain bridge with internal DDS on lo '
            'and V2V DDS on wlp3s0.'
        ),
    )
    parser.add_argument('--internal-domain', type=int, default=default_internal_domain)
    parser.add_argument('--shared-domain', type=int, default=SHARED_DOMAIN)
    parser.add_argument('--wifi-interface', default=DEFAULT_WIFI_INTERFACE)
    parser.add_argument('--leader-ip', default=DEFAULT_LEADER_SVDP_IP)
    parser.add_argument('--follower-ip', default=DEFAULT_FOLLOWER_SVDP_IP)
    parser.add_argument('--config', default='')
    parser.add_argument('--cyclonedds-uri', default='')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def _run_internal(
    role: str,
    default_domain: int,
    launch_file: str,
    argv: list[str] | None,
) -> None:
    parser = _internal_parser(role, default_domain)
    args, launch_args = parser.parse_known_args(argv)
    cyclone_uri = args.cyclonedds_uri or _write_cyclonedds_xml(
        f'svdp_{role}_internal_cyclonedds',
        [DomainNetwork(domain_id='any', interface='lo')],
    )
    env = _base_env(args.domain_id, cyclone_uri)
    command = ['ros2', 'launch', 'platoon_bringup', launch_file] + launch_args
    _exec_ros(command, env, args.dry_run)


def _run_bridge(
    role: str,
    default_internal_domain: int,
    default_config: str,
    argv: list[str] | None,
) -> None:
    parser = _bridge_parser(role, default_internal_domain)
    args, bridge_args = parser.parse_known_args(argv)
    peers = (args.leader_ip, args.follower_ip)
    cyclone_uri = args.cyclonedds_uri or _write_cyclonedds_xml(
        f'svdp_{role}_bridge_cyclonedds',
        [
            DomainNetwork(domain_id=args.internal_domain, interface='lo'),
            DomainNetwork(
                domain_id=args.shared_domain,
                interface=args.wifi_interface,
                peers=peers,
            ),
        ],
    )
    env = _base_env(None, cyclone_uri)
    config = args.config or _package_config_path(default_config)
    command = ['ros2', 'run', 'domain_bridge', 'domain_bridge'] + bridge_args + [config]
    _exec_ros(command, env, args.dry_run)


def leader_internal_main(argv: list[str] | None = None) -> None:
    _run_internal('leader', LEADER_INTERNAL_DOMAIN, 'leader_bringup.launch.py', argv)


def follower_internal_main(argv: list[str] | None = None) -> None:
    _run_internal('follower', FOLLOWER_INTERNAL_DOMAIN, 'follower_bringup.launch.py', argv)


def leader_bridge_main(argv: list[str] | None = None) -> None:
    _run_bridge(
        'leader',
        LEADER_INTERNAL_DOMAIN,
        'svdp_leader_10_to_30.yaml',
        argv,
    )


def follower_bridge_main(argv: list[str] | None = None) -> None:
    _run_bridge(
        'follower',
        FOLLOWER_INTERNAL_DOMAIN,
        'svdp_follower_30_20.yaml',
        argv,
    )

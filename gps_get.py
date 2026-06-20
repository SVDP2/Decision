#!/usr/bin/env python3

import os
import csv
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from pyproj import Transformer

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)


class GpsPathRecorder(Node):
    def __init__(self):
        super().__init__('gps_path_recorder')

        # ===== 설정값 =====
        self.gps_topic = '/f9p/fix'
        self.output_csv = '/home/kai/dc_ws/GP_Decision/config/path_csv/gongD131.csv'

        # 최소 저장 간격 [m]
        # 0.05 = 5cm 이상 이동했을 때만 저장
        self.min_distance_m = 0.05

        # False: 기존 파일 덮어쓰기
        # True : 기존 파일 뒤에 이어쓰기
        self.append_mode = False

        # WGS84 위도/경도 -> UTM Zone 52N
        # longitude 127도 부근, 서울/건국대 인근은 EPSG:32652 사용
        self.transformer = Transformer.from_crs(
            'EPSG:4326',
            'EPSG:32652',
            always_xy=True
        )

        self.last_x = None
        self.last_y = None
        self.saved_count = 0

        self.prepare_csv()

        # ===== QoS 설정 =====
        # /f9p/fix publisher가 BEST_EFFORT 방식이므로 subscriber도 BEST_EFFORT로 맞춤
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            NavSatFix,
            self.gps_topic,
            self.gps_callback,
            qos_profile
        )

        self.get_logger().info('GPS path recorder started')
        self.get_logger().info(f'GPS topic      : {self.gps_topic}')
        self.get_logger().info(f'Output CSV     : {self.output_csv}')
        self.get_logger().info(f'Min distance   : {self.min_distance_m:.3f} m')
        self.get_logger().info(f'Append mode    : {self.append_mode}')
        self.get_logger().info('Press Ctrl+C to stop recording.')

    def prepare_csv(self):
        output_dir = os.path.dirname(self.output_csv)
        os.makedirs(output_dir, exist_ok=True)

        mode = 'a' if self.append_mode else 'w'

        self.csv_file = open(self.output_csv, mode, newline='')
        self.writer = csv.writer(self.csv_file)

        if not self.append_mode:
            self.writer.writerow(['X(E/m)', 'Y(N/m)'])
            self.csv_file.flush()

    def gps_callback(self, msg: NavSatFix):
        # status.status:
        # -1: NO_FIX
        #  0: FIX
        #  1: SBAS_FIX
        #  2: GBAS_FIX / RTK 계열에서 자주 나옴
        if msg.status.status == NavSatStatus.STATUS_NO_FIX:
            self.get_logger().warn('GPS no fix. Skip.')
            return

        lat = msg.latitude
        lon = msg.longitude

        if not self.is_valid_gps(lat, lon):
            self.get_logger().warn(f'Invalid GPS data. lat={lat}, lon={lon}')
            return

        # always_xy=True 이므로 lon, lat 순서로 넣어야 함
        x, y = self.transformer.transform(lon, lat)

        if not self.should_save(x, y):
            return

        self.writer.writerow([f'{x:.15f}', f'{y:.15f}'])
        self.csv_file.flush()

        self.last_x = x
        self.last_y = y
        self.saved_count += 1

        self.get_logger().info(
            f'Saved #{self.saved_count}: '
            f'X={x:.6f}, Y={y:.6f}, '
            f'lat={lat:.8f}, lon={lon:.8f}'
        )

    def is_valid_gps(self, lat, lon):
        if math.isnan(lat) or math.isnan(lon):
            return False

        if lat < -90.0 or lat > 90.0:
            return False

        if lon < -180.0 or lon > 180.0:
            return False

        return True

    def should_save(self, x, y):
        if self.last_x is None or self.last_y is None:
            return True

        dist = math.hypot(x - self.last_x, y - self.last_y)

        if dist < self.min_distance_m:
            return False

        return True

    def close(self):
        if hasattr(self, 'csv_file') and not self.csv_file.closed:
            self.csv_file.flush()
            self.csv_file.close()

        self.get_logger().info(
            f'Finished. Saved {self.saved_count} points to {self.output_csv}'
        )


def main(args=None):
    rclpy.init(args=args)

    node = GpsPathRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
import requests
from rclpy.node import Node


class ControlServerClient:
    """Control Server (FastAPI) HTTP API 클라이언트.

    Fleet Manager 내부 모듈들이 직접 HTTP를 호출하지 않도록,
    Control Server 와의 모든 통신은 이 클래스로 집약한다.
    """

    def __init__(
        self,
        node: Node,
        base_url: str,
        timeout_sec: float = 5.0,
    ) -> None:
        self._node = node
        self._base_url = base_url.rstrip('/')
        self._timeout = timeout_sec

    # ── Zone ──────────────────────────────────────────────────────────

    def fetch_zone_coords(self) -> dict[str, tuple[float, float]]:
        """전체 zone 의 (x, y) 좌표를 zone_name 키로 반환한다.

        실패 시 빈 dict 를 반환하여, 호출자가 기본 좌표를 유지하도록 한다.
        """
        url = f'{self._base_url}/api/fleet/zones'
        try:
            resp = requests.get(
                url,
                params={'zone_type': 'ALL'},
                timeout=self._timeout,
            )
        except requests.exceptions.RequestException as e:
            self._node.get_logger().warn(
                f'[ControlServerClient] zone 좌표 조회 오류: {e}'
            )
            return {}

        if resp.status_code != 200:
            self._node.get_logger().warn(
                f'[ControlServerClient] zone 좌표 조회 실패: HTTP {resp.status_code}'
            )
            return {}

        coords: dict[str, tuple[float, float]] = {}
        for zone in resp.json():
            pose = zone.get('pose') or {}
            if pose.get('x') is None or pose.get('y') is None:
                continue
            coords[zone['zone_name']] = (float(pose['x']), float(pose['y']))

        self._node.get_logger().info(
            f'[ControlServerClient] zone 좌표 조회 완료: {len(coords)}개'
        )
        return coords

#!/usr/bin/env python3
"""
Parking Space Manager Node
- Supabase DB와 연동하여 주차 공간 관리
- 우선순위 기반 주차 위치 할당
- 이중주차 상태 관리
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import json
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from supabase import create_client, Client


@dataclass
class ParkingSpace:
    """주차 공간 데이터 클래스"""
    location_id: str
    zone: str
    floor: str
    x: float
    y: float
    orientation: str  # 'NORTH', 'SOUTH', 'EAST', 'WEST'
    is_occupied: bool = False
    vehicle_type: Optional[str] = None


@dataclass
class AllocationResult:
    """주차 할당 결과"""
    success: bool
    location_id: str
    zone_id: str
    x: float
    y: float
    orientation: str  # 'NORTH', 'SOUTH', 'EAST', 'WEST'
    message: str


class ParkingSpaceManager(Node):
    def __init__(self):
        super().__init__('parking_space_manager')
        
        # ==================== 파라미터 설정 ====================
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')
        self.declare_parameter('table_name', 'parking_locations')
        
        supabase_url = self.get_parameter('supabase_url').value
        supabase_key = self.get_parameter('supabase_key').value
        self.table_name = self.get_parameter('table_name').value
        
        # ==================== Supabase 클라이언트 ====================
        try:
            self.supabase: Client = create_client(supabase_url, supabase_key)
            self.get_logger().info("✅ Supabase 연결 성공")
        except Exception as e:
            self.get_logger().error(f"❌ Supabase 연결 실패: {e}")
            raise
        
        # ==================== ROS2 통신 ====================
        # 차종 정보 구독
        self.vehicle_type_sub = self.create_subscription(
            String,
            '/vehicle_type',
            self.vehicle_type_callback,
            10
        )
        
        # 주차 할당 결과 발행
        self.allocation_pub = self.create_publisher(
            String,
            '/parking_allocation',
            10
        )
        
        # 주차 완료 신호 구독
        self.parking_done_sub = self.create_subscription(
            Bool,
            '/parking_done',
            self.parking_done_callback,
            10
        )
        
        self.current_vehicle_type: Optional[str] = None
        self.current_allocation: Optional[AllocationResult] = None
        self.parking_spaces_cache: List[ParkingSpace] = []
        
        # 우선순위 설정 (zone 번호 순서)
        self.zone_priority = [4, 3, 2, 1]  # 큰 숫자가 높은 우선순위
        
        self.get_logger().info("🚗 Parking Space Manager 시작!")
        
        # 초기 DB 데이터 로드
        self.refresh_parking_data()

    def refresh_parking_data(self):

        try:
            response = self.supabase.table(self.table_name).select("*").execute()
            
            self.parking_spaces_cache = []
            for row in response.data:
                space = ParkingSpace(
                    location_id=row['location_id'],
                    zone=row['zone'],
                    floor=row['floor'],
                    x=float(row['x']),
                    y=float(row['y']),
                    orientation=str(row['orientation']),  # text로 저장
                    is_occupied=row.get('is_occupied', False),
                    vehicle_type=row.get('vehicle_type', None)
                )
                self.parking_spaces_cache.append(space)
            
            self.get_logger().info(f"📊 주차 공간 데이터 로드: {len(self.parking_spaces_cache)}개")
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 조회 실패: {e}")

    def vehicle_type_callback(self, msg: String):
        """차종 정보 수신 → 주차 공간 할당"""
        vehicle_type = msg.data
        self.get_logger().info(f"🚙 차종 감지: {vehicle_type}")
        
        self.current_vehicle_type = vehicle_type
        
        # 주차 공간 할당
        allocation = self.allocate_parking_space(vehicle_type)
        
        if allocation.success:
            self.current_allocation = allocation
            
            # 결과를 JSON으로 발행
            result_json = json.dumps({
                'success': True,
                'location_id': allocation.location_id,
                'zone_id': allocation.zone_id,
                'x': allocation.x,
                'y': allocation.y,
                'orientation': allocation.orientation,
                'message': allocation.message
            })
            
            self.allocation_pub.publish(String(data=result_json))
            self.get_logger().info(f"✅ {allocation.message}")
            
        else:
            # 할당 실패
            result_json = json.dumps({
                'success': False,
                'message': allocation.message
            })
            self.allocation_pub.publish(String(data=result_json))
            self.get_logger().warn(f"⚠️ {allocation.message}")

    def allocate_parking_space(self, vehicle_type: str) -> AllocationResult:
        """
        우선순위 기반 주차 공간 할당
        
        우선순위 규칙:
        1. vehicle_type이 'A', 'B', 'C'이면 해당 zone 우선
        2. X_n_2 > X_n_1 (안쪽 우선)
        3. 숫자가 클수록 우선 (4>3>2>1)
        예: A타입 차량 → A_4_2 > A_4_1 > A_3_2 > ... > B_4_2 > ...
        """
        
        # 1. DB 최신 데이터 가져오기
        self.refresh_parking_data()
        
        # 2. 빈 공간 필터링 (X_n_1, X_n_2 형태만)
        available_spaces = [
            space for space in self.parking_spaces_cache
            if not space.is_occupied and '_' in space.location_id and 
            len(space.location_id.split('_')) == 3  # 예: A_1_1, A_1_2
        ]
        
        # 2-1. vehicle_type에 따라 zone 우선 정렬
        preferred_zone = vehicle_type.upper() if vehicle_type.upper() in ['A', 'B', 'C'] else None
        
        if preferred_zone:
            # 선호 zone 먼저, 나머지는 뒤로
            preferred_spaces = [s for s in available_spaces if s.zone == preferred_zone]
            other_spaces = [s for s in available_spaces if s.zone != preferred_zone]
            available_spaces = preferred_spaces + other_spaces
            
            self.get_logger().info(f"🎯 {vehicle_type} 타입 → {preferred_zone}존 우선 ({len(preferred_spaces)}개 가능)")
        
        if not available_spaces:
            return AllocationResult(
                success=False,
                location_id='',
                zone_id='',
                x=0.0,
                y=0.0,
                orientation='NORTH',
                message='❌ 사용 가능한 주차 공간이 없습니다'
            )
        
        # 3. 우선순위 정렬
        sorted_spaces = self.sort_by_priority(available_spaces)
        
        # 4. 최우선 공간 선택
        selected = sorted_spaces[0]
        
        # 5. zone_id 추출 (예: A_1_2 → A_1)
        parts = selected.location_id.split('_')
        zone_id = f"{parts[0]}_{parts[1]}"
        
        # 6. DB 업데이트 (주차 상태)
        self.update_parking_status(selected.location_id, True, vehicle_type)
        
        return AllocationResult(
            success=True,
            location_id=selected.location_id,
            zone_id=zone_id,
            x=selected.x,
            y=selected.y,
            orientation=selected.orientation,
            message=f'주차 위치 할당: {selected.location_id} (좌표: {selected.x}, {selected.y})'
        )

    def sort_by_priority(self, spaces: List[ParkingSpace]) -> List[ParkingSpace]:
        """
        우선순위에 따라 주차 공간 정렬
        
        정렬 규칙:
        1. 안쪽(_2) > 바깥쪽(_1)
        2. 같은 레벨이면 숫자 큰 순서 (4>3>2>1)
        """
        
        def get_priority_key(space: ParkingSpace) -> Tuple[int, int, int]:
            parts = space.location_id.split('_')
            if len(parts) != 3:
                return (0, 0, 0)
            
            zone_num = int(parts[1])  # 예: A_2_1 → 2
            position = int(parts[2])  # 예: A_2_1 → 1
            
            # 튜플 반환: (위치 우선순위, 존 번호, 위치)
            # 위치 2가 1보다 우선, 존 번호 큰 게 우선
            return (
                -position,  # -2가 -1보다 작으므로 _2가 먼저 옴
                -zone_num,  # -4가 -1보다 작으므로 4가 먼저 옴
                0
            )
        
        return sorted(spaces, key=get_priority_key)

    def update_parking_status(self, location_id: str, is_occupied: bool, vehicle_type: Optional[str] = None):
        """DB에 주차 상태 업데이트"""
        try:
            update_data = {
                'is_occupied': is_occupied,
                'vehicle_type': vehicle_type if is_occupied else None
            }
            
            self.supabase.table(self.table_name).update(update_data).eq(
                'location_id', location_id
            ).execute()
            
            self.get_logger().info(f"📝 DB 업데이트: {location_id} → {'주차됨' if is_occupied else '비어있음'}")
            
        except Exception as e:
            self.get_logger().error(f"❌ DB 업데이트 실패: {e}")


    def parking_done_callback(self, msg: Bool):
        """주차 완료 신호 수신"""
        if msg.data and self.current_allocation:
            self.get_logger().info(
                f"✅ 주차 완료: {self.current_allocation.location_id}"
            )
            # 필요시 추가 로직 (예: 통계 업데이트)
            self.current_allocation = None
            self.current_vehicle_type = None

def main(args=None):
    rclpy.init(args=args)
    
    node = ParkingSpaceManager()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

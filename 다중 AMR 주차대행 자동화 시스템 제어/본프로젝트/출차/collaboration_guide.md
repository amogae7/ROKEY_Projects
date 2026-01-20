

## 1: 매니저로부터 Task 데이터 받기

### 📡 통신 구조
```
[Robot Manager]
      ↓ (ROS2 Topic: /task_command/robot5)
[출차 로봇 (robot5)]
```

### 💾 Task 데이터 형식 (JSON)

**ENTER Task:**
```json
{
  "task_id": "TASK_001",
  "task_type": "ENTER",
  "vehicle_plate": "12가3456",
  "start_location": "ENTRANCE",
  "start_coords": {"x": -2.93, "y": -0.326, "orientation": "SOUTH"},
  "target_location": "B_2_1",
  "target_coords": {"x": 5.0, "y": 10.0, "orientation": "NORTH"},
  "waypoint_location": "B_2",  // 있을 수도, 없을 수도
  "waypoint_coords": {"x": 5.0, "y": 8.0, "orientation": "NORTH"},
  "priority": 1
}
```

**EXIT_SINGLE Task:**
```json
{
  "task_id": "TASK_002",
  "task_type": "EXIT_SINGLE",
  "vehicle_plate": "34나5678",
  "start_location": "A_1_1",
  "start_coords": {"x": 3.0, "y": 5.0, "orientation": "EAST"},
  "target_location": "EXIT_ZONE",
  "target_coords": {"x": -2.95, "y": 4.05, "orientation": "NORTH"},
  "waypoint_location": "A_1",  // 주차 위치면 waypoint 자동 추가됨
  "waypoint_coords": {"x": 3.0, "y": 3.0, "orientation": "EAST"},
  "priority": 2
}
```

**EXIT_DOUBLE Task (Phase 2):**
```json
{
  "task_id": "TASK_003",
  "task_type": "EXIT_DOUBLE",
  "phase": 2,
  "vehicle_plate": "56다7890",
  "start_location": "B_2_1",
  "start_coords": {"x": 5.0, "y": 10.0, "orientation": "NORTH"},
  "target_location": "EXIT_ZONE",
  "target_coords": {"x": -2.95, "y": 4.05, "orientation": "NORTH"},
  "waypoint_location": "B_2",
  "waypoint_coords": {"x": 5.0, "y": 8.0, "orientation": "NORTH"},
  "description": "메인 차량 출차"
}
```

---

### 위치: `__init__()` 함수 내부

```python
def __init__(self):
    super().__init__('robot5_node')
    
    # ==================== 설정 ====================
    self.robot_id = 'robot5'
    
    # ==================== Supabase 연결 ====================
    # (입차팀과 동일)
    self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
    self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')
    
    try:
        url = self.get_parameter('supabase_url').value
        key = self.get_parameter('supabase_key').value
        self.supabase: Client = create_client(url, key)
        self.get_logger().info("✅ Supabase 연결 성공")
    except Exception as e:
        self.get_logger().error(f"❌ Supabase 연결 실패: {e}")
        raise
    
    # ==================== Task 구독 ====================
    # ⭐ 핵심: 매니저로부터 Task 받기
    self.callback_group = ReentrantCallbackGroup()
    
    self.task_sub = self.create_subscription(
        String,
        f'/task_command/{self.robot_id}',  # '/task_command/robot5'
        self.task_callback,                 # Task 수신 시 호출될 함수
        10,
        callback_group=self.callback_group
    )
    
    # ==================== 상태 변수 ====================
    self.is_busy = False
    self.current_task = None
    
    self.get_logger().info("✅ Task 구독 준비 완료: /task_command/robot5")
```

### 📍 위치: 클래스 메서드로 추가

```python
def task_callback(self, msg: String):
    """
    ⭐ 매니저로부터 Task 수신
    
    Args:
        msg: String 타입의 ROS 메시지
             msg.data = JSON 문자열
    """
    # 이미 작업 중이면 무시
    if self.is_busy:
        self.get_logger().warn("⚠️ 이미 작업 중 - Task 무시")
        return
    
    try:
        # JSON 파싱
        task = json.loads(msg.data)
        
        self.get_logger().info("\n" + "🔔" * 30)
        self.get_logger().info(f"Task 수신: {task['task_type']}")
        self.get_logger().info(f"차량: {task['vehicle_plate']}")
        self.get_logger().info("🔔" * 30)
        
        # ⭐ 중요: Task를 받자마자 'assigned' 상태로 변경
        self.mark_task_assigned(task['task_id'])
        
        # Task 실행
        self.execute_task(task)
        
    except Exception as e:
        self.get_logger().error(f"❌ Task 처리 에러: {e}")
        import traceback
        self.get_logger().error(traceback.format_exc())
        self.is_busy = False

def execute_task(self, task: Dict):
    """Task 타입별 실행"""
    self.is_busy = True
    self.current_task = task
    
    task_type = task['task_type']
    
    try:
        if task_type == 'EXIT_SINGLE':
            self.do_exit_single(task)
        elif task_type == 'EXIT_DOUBLE' and task.get('phase') == 2:
            self.do_exit_double_phase2(task)
        else:
            self.get_logger().warn(f"⚠️ 지원하지 않는 Task: {task_type}")
    
    except Exception as e:
        self.get_logger().error(f"❌ Task 실행 에러: {e}")
        import traceback
        self.get_logger().error(traceback.format_exc())
    
    finally:
        self.is_busy = False
        self.current_task = None
```

---

## 🔧 출차팀이 심어야 할 코드 2: DB 업데이트 함수

### 📍 위치: 클래스 메서드로 추가

```python
# ==================== DB 업데이트 ====================

def mark_task_assigned(self, task_id: str):
    """
    ⭐ Task를 받자마자 'assigned' 상태로 변경
    
    이 함수는 task_callback에서 자동으로 호출됩니다.
    출차팀이 Task를 받았다는 것을 Manager에게 알려줍니다.
    
    Args:
        task_id: 'TASK_001' 같은 Task ID
    """
    try:
        self.supabase.table('tasks').update({
            'status': 'assigned'
        }).eq('task_id', task_id).execute()
        
        self.get_logger().info(f"✅ Task 할당 완료: {task_id}")
    
    except Exception as e:
        self.get_logger().error(f"❌ Task 할당 실패: {e}")

def mark_task_done(self, task_id: str):
    """
    ⭐ Task 완료 시 'done' 상태로 변경
    
    이 함수는 작업이 완전히 끝났을 때 호출해야 합니다.
    예: do_exit_single() 함수의 마지막 부분
    
    Args:
        task_id: 'TASK_001' 같은 Task ID
    """
    try:
        self.supabase.table('tasks').update({
            'done': True,
            'status': 'done'
        }).eq('task_id', task_id).execute()
        
        self.get_logger().info(f"✅ Task 완료: {task_id}")
    
    except Exception as e:
        self.get_logger().error(f"❌ Task 완료 실패: {e}")

def handle_navigation_failure(self, task_id: str):
    """
    ⭐ Navigation 실패 시 'failed' 상태로 변경
    
    이 함수는 이동 중 에러가 발생했을 때 호출합니다.
    
    Args:
        task_id: 'TASK_001' 같은 Task ID
    """
    self.get_logger().error("❌ Navigation 실패 - Task 중단")
    
    try:
        self.supabase.table('tasks').update({
            'status': 'failed'
        }).eq('task_id', task_id).execute()
    except Exception as e:
        self.get_logger().error(f"❌ 실패 상태 업데이트 실패: {e}")
    
    self.is_busy = False
```

---

## 🔧 출차팀이 심어야 할 코드 3: Task 실행 예제

### 📍 위치: 클래스 메서드로 추가

```python
def do_exit_single(self, task: Dict):
    """
    ⭐ EXIT_SINGLE Task 실행
    
    주차 위치 → (Waypoint) → 출구
    """
    self.get_logger().info("\n" + "=" * 60)
    self.get_logger().info("🚗 단독 출차 작업 시작")
    self.get_logger().info("=" * 60)
    
    vehicle_plate = task['vehicle_plate']
    start_coords = task['start_coords']
    target_coords = task['target_coords']
    
    # Waypoint 확인
    has_waypoint = 'waypoint_coords' in task
    
    self.get_logger().info(f"   차량: {vehicle_plate}")
    self.get_logger().info(f"   출발: {task['start_location']}")
    
    if has_waypoint:
        self.get_logger().info(f"   경유: {task['waypoint_location']}")
    
    self.get_logger().info(f"   도착: {task['target_location']}")
    
    # 1. Undock
    self.get_logger().info("\n[1/7] Undocking...")
    self.undock()
    
    # 2. 주차 위치로 이동
    self.get_logger().info(f"\n[2/7] 주차 위치로 이동: {task['start_location']}")
    if not self.nav_to_coords(start_coords):
        self.handle_navigation_failure(task['task_id'])
        return
    
    # 3. 차량 Pick
    self.get_logger().info(f"\n[3/7] 차량 Pick: {vehicle_plate}")
    self.perform_pick_action()
    
    # 4. Waypoint로 이동 (있으면)
    if has_waypoint:
        self.get_logger().info(f"\n[4/7] 중간 지점 이동: {task['waypoint_location']}")
        waypoint_coords = task['waypoint_coords']
        if not self.nav_to_coords(waypoint_coords):
            self.handle_navigation_failure(task['task_id'])
            return
    
    # 5. 출구로 이동
    step_num = 5 if has_waypoint else 4
    self.get_logger().info(f"\n[{step_num}/7] 출구로 이동: {task['target_location']}")
    
    if not self.nav_to_coords(target_coords):
        self.handle_navigation_failure(task['task_id'])
        return
    
    # 6. 차량 Place
    step_num += 1
    self.get_logger().info(f"\n[{step_num}/7] 차량 Place")
    self.perform_place_action()
    
    # 7. ⭐ Task 완료 - DB 업데이트!
    step_num += 1
    self.get_logger().info(f"\n[{step_num}/7] Task 완료 처리")
    self.mark_task_done(task['task_id'])  # ⭐ 이 부분이 핵심!
    
    # 8. Dock
    self.get_logger().info("\n[7/7] Docking...")
    self.dock()
    
    self.get_logger().info("\n✅ 출차 작업 완료!")
    self.get_logger().info("=" * 60 + "\n")
```

---

## 📊 전체 흐름 다이어그램

```
┌──────────────────────────────────────────────────────────────────┐
│                      [Robot Manager]                              │
│                                                                    │
│  1. tasks 테이블에서 pending Task 발견                             │
│  2. Task 데이터 + 좌표 변환                                        │
└──────────────────────┬───────────────────────────────────────────┘
                       │
                       │ (ROS2 Topic: /task_command/robot5)
                       │ JSON 데이터 발행
                       ↓
┌──────────────────────────────────────────────────────────────────┐
│                   [출차 로봇 (robot5)]                             │
│                                                                    │
│  3. task_callback() 함수에서 JSON 수신                            │
│  4. mark_task_assigned() → DB 상태: 'assigned' ⭐                 │
│  5. do_exit_single() 실행                                         │
│     - Undock                                                      │
│     - 주차 위치로 이동                                             │
│     - Pick                                                        │
│     - (Waypoint 이동)                                             │
│     - 출구로 이동                                                  │
│     - Place                                                       │
│  6. mark_task_done() → DB 상태: 'done' ⭐                         │
│  7. Dock                                                          │
└──────────────────────────────────────────────────────────────────┘
```

---

## ⚠️ 주의사항

### 1. **상태 변경 타이밍이 중요합니다!**

```python
# ❌ 잘못된 순서
def task_callback(self, msg):
    task = json.loads(msg.data)
    self.execute_task(task)
    self.mark_task_assigned(task['task_id'])  # 너무 늦음!

# ✅ 올바른 순서
def task_callback(self, msg):
    task = json.loads(msg.data)
    self.mark_task_assigned(task['task_id'])  # 받자마자!
    self.execute_task(task)
```

**이유:** Manager가 Realtime으로 모니터링하고 있어서, 빠르게 'assigned'로 변경해야 중복 발행을 막을 수 있습니다.

### 2. **완료 신호는 모든 작업이 끝난 후**

```python
# ✅ 올바른 위치
def do_exit_single(self, task):
    # ... 모든 작업 수행 ...
    self.perform_place_action()
    
    # 작업이 정말 끝났을 때!
    self.mark_task_done(task['task_id'])
    
    self.dock()
```

### 3. **Navigation 실패 시 반드시 처리**

```python
if not self.nav_to_coords(target_coords):
    self.handle_navigation_failure(task['task_id'])
    return  # 더 이상 진행 안 함
```

---

## 🎓 왜 이렇게 해야 하나요?

### 로봇 시스템의 **상태 머신(State Machine)** 패턴입니다

```
        [PENDING]
            ↓ (로봇이 Task 수신)
      [ASSIGNED]
            ↓ (작업 진행 중)
    [IN_PROGRESS]
            ↓ (작업 완료)
        [DONE]
```



## 💡 추가 팁

### Import 문 추가
```python
import json
from typing import Dict
from std_msgs.msg import String
from supabase import create_client, Client
```

### main() 함수에서 Executor 설정
```python
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

def main(args=None):
    rclpy.init(args=args)
    node = Robot5Node()
    
    # MultiThreadedExecutor 사용 (Nav2와 구독 동시 처리)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```


// 1. Firebase Config 및 초기화
const firebaseConfig = {
    apiKey: "AIzaSyARr3MrHNBtxKL7maWr1L4NmENvGu5pn2w",
    authDomain: "rokey-baristar-robot.firebaseapp.com",
    databaseURL: "https://rokey-baristar-robot-default-rtdb.asia-southeast1.firebasedatabase.app",
    projectId: "rokey-baristar-robot",
    storageBucket: "rokey-baristar-robot.firebasestorage.app",
    messagingSenderId: "386678381726",
    appId: "1:386678381726:web:99ee620c138d04062eaf8f"
};
firebase.initializeApp(firebaseConfig);
const database = firebase.database();

// 🚨 경로 설정
const ORDER_REF = database.ref('barista_control/order_command'); 
const STATUS_REF = database.ref('barista_status/current_state'); 

// UI 요소
const brewButton = document.getElementById('brew-button');
const statusMessage = document.getElementById('status-message');
const selectionBoxes = document.querySelectorAll('.selection-box');
// [추가됨] 로봇 상태 배지 요소
const statusBadge = document.getElementById('robot-status-badge');

let selectedBeverage = null;
let selectedTemp = null;
let isRobotBusy = false; // 로봇 상태 전역 변수

// ----------------------------------------------------
// A. 버튼 선택 로직 (UI)
// ----------------------------------------------------

function handleSelection(event) {
    // 로봇이 바쁘면 버튼 선택도 막음
    if (isRobotBusy) return;

    const button = event.target.closest('.order-button');
    if (!button) return;

    const type = button.dataset.type;
    const value = button.dataset.value;
    const container = button.parentElement;

    // UI 업데이트
    container.querySelectorAll('.order-button').forEach(btn => btn.classList.remove('selected'));
    button.classList.add('selected');

    // 변수 업데이트
    if (type === 'beverage') selectedBeverage = value;
    else if (type === 'temp') selectedTemp = value;
    
    checkOrderReady();
}

// 주문 가능 상태 확인
function checkOrderReady() {
    // 로봇이 바쁘면 무조건 비활성화
    if (isRobotBusy) {
        brewButton.disabled = true;
        return;
    }

    if (selectedBeverage && selectedTemp) {
        brewButton.disabled = false;
        brewButton.classList.remove('complete');
        brewButton.innerText = 'BREW COFFEE (주문 시작)';
        statusMessage.innerText = `[${selectedBeverage} - ${selectedTemp}] 주문 준비 완료.`;
    } else {
        brewButton.disabled = true;
        statusMessage.innerText = '음료 종류와 온도를 모두 선택해주세요.';
    }
}

selectionBoxes.forEach(box => {
    box.addEventListener('click', handleSelection);
});


// ----------------------------------------------------
// B. 주문 시작 (Firebase 쓰기)
// ----------------------------------------------------

function startOrder() {
    if (!selectedBeverage || !selectedTemp || isRobotBusy) return;

    // UI 즉시 잠금
    setUiState(true, 'WAITING'); 
    statusMessage.innerText = '로봇에게 주문을 전송하고 있습니다...';

    const orderPayload = {
        beverage: selectedBeverage,
        temp: selectedTemp,
        timestamp: Date.now(),
        command: "START"
    };

    ORDER_REF.set(orderPayload)
        .then(() => {
            console.log("주문 전송 성공");
            statusMessage.innerText = '전송 완료! 로봇 응답 대기 중...';
        })
        .catch((error) => {
            console.error("전송 실패:", error);
            alert("전송 실패. 다시 시도해주세요.");
            setUiState(false, 'IDLE'); // 실패 시 잠금 해제
        });
}

brewButton.addEventListener('click', startOrder);


// ----------------------------------------------------
// C. 로봇 상태 모니터링 (핵심 기능)
// ----------------------------------------------------

function setupStatusMonitoring() {
    STATUS_REF.on('value', (snapshot) => {
        const currentState = snapshot.val(); // IDLE, MAKING, COMPLETE, ERROR 등
        console.log("Current Robot State:", currentState);
        
        updateStatusBadge(currentState);

        if (currentState === "IDLE") {
            // 로봇 대기 중 (주문 가능)
            if (isRobotBusy) {
                // 이전에 바빴다면 이제 해제
                isRobotBusy = false;
                completeOrderSequence(); // 주문이 끝났으면 초기화
            } else {
                // 그냥 처음부터 IDLE인 경우
                isRobotBusy = false;
                checkOrderReady();
            }

        } else if (currentState === "MAKING" || currentState === "BUSY") {
            // 로봇 작업 중 (주문 불가)
            isRobotBusy = true;
            setUiState(true, 'MAKING');
            statusMessage.innerText = '🤖 로봇이 음료를 제조 중입니다. 잠시만 기다려주세요.';

        } else if (currentState === "COMPLETE") {
            // 작업 완료
            isRobotBusy = true; // 아직은 바쁜 상태로 간주 (사람이 픽업해야 함)
            statusMessage.innerText = '✅ 음료가 준비되었습니다! 픽업해주세요.';
            
            // 주문 완료 후 Firebase 명령 지우기 (안전장치)
            ORDER_REF.set(null);
        }
    });
}

// ----------------------------------------------------
// D. 헬퍼 함수들 (UI 제어 및 배지 업데이트)
// ----------------------------------------------------

// 상단 배지 업데이트 함수

function updateStatusBadge(state) {
    if (!statusBadge) return;

    if (state === "IDLE") {
        // 대기 중
        statusBadge.innerText = "ROBOT: IDLE";
        statusBadge.className = "status-badge ready";
        
    } else if (state === "COLLISION") {
        statusBadge.innerText = "ROBOT: COLLISION 🚨"; 
        statusBadge.className = "status-badge error"; 

    } else if (state === "EMERGENCY_STOP") {
        statusBadge.innerText = "ROBOT: STOP ⛔"; 
        statusBadge.className = "status-badge error"; 

    } else if (state === "RECOVERING") { 
        // ▼▼▼ [추가] 복구 중 상태 처리 ▼▼▼
        statusBadge.innerText = "ROBOT: RECOVERING 🔄";
        statusBadge.className = "status-badge recovering"; // 새로운 CSS 클래스 적용

    } else {
        // 작업 중 (MAKING 등)
        statusBadge.innerText = `ROBOT: ${state}`;
        statusBadge.className = "status-badge busy";
    }
}

// 전체 UI 잠금/해제 관리
function setUiState(busy, mode) {
    // 1. 메인 버튼 잠금/해제
    brewButton.disabled = busy;

    // 2. 선택 버튼들 시각적 잠금 처리
    const allOrderButtons = document.querySelectorAll('.order-button');
    allOrderButtons.forEach(btn => {
        if (busy) {
            btn.style.opacity = '0.5';       // 흐리게
            btn.style.pointerEvents = 'none'; // 클릭 불가
        } else {
            btn.style.opacity = '1';         // 선명하게
            btn.style.pointerEvents = 'auto'; // 클릭 가능
        }
    });
}

// [추가] 주문 완료 후 초기화 함수 (위에서 호출하므로 필요함)
function completeOrderSequence() {
    console.log("주문 사이클 완료. 초기화합니다.");
    
    // 선택 변수 초기화
    selectedBeverage = null;
    selectedTemp = null;
    
    // 선택된 버튼 디자인 해제
    document.querySelectorAll('.order-button.selected').forEach(btn => {
        btn.classList.remove('selected');
    });

    // 상태 메시지 및 버튼 상태 갱신
    statusMessage.innerText = '음료 종류와 온도를 선택해주세요.';
    checkOrderReady();
}

// ====================================================
// E. 앱 시작 (가장 중요!)
// ====================================================
// 이 함수를 실행해야 Firebase 감시가 시작됩니다.
setupStatusMonitoring();

// [기존 코드 아래에 추가]

// ----------------------------------------------------
// [추가] 안전 제어 버튼 로직 (STOP & RESET)
// ----------------------------------------------------

const btnStop = document.getElementById('btn-stop');
const btnReset = document.getElementById('btn-reset');

// 1. 작업 정지 (STOP) - 비상용
btnStop.addEventListener('click', () => {
    // 사용자에게 한 번 더 확인 (선택 사항)
    // if (!confirm("로봇을 즉시 정지하시겠습니까?")) return;

    console.log("!!! STOP 명령 전송 !!!");
    statusMessage.innerText = '⛔ 정지 신호 전송 중...';
    
    // 로봇이 즉시 멈추도록 STOP 명령 전송
    ORDER_REF.set({
        command: "STOP",
        timestamp: Date.now()
    });
    
    // UI 강제 잠금 해제 (필요하다면) 또는 에러 상태 표시
    // 여기서는 로봇 상태(STATUS_REF)가 바뀌는 것을 기다리는 게 안전함
});

// 2. 시스템 복구 (RESET/RECOVER) - 에러 발생 시 초기화
btnReset.addEventListener('click', () => {
    console.log(">>> RECOVER 명령 전송");
    statusMessage.innerText = '🔄 시스템 복구/초기화 요청 중...';

    // 로봇에게 복구 명령 전송
    ORDER_REF.set({
        command: "RECOVER",
        timestamp: Date.now()
    });
});

// --- 속도 조절 로직 ---
const speedSlider = document.getElementById('speed-slider');
const speedValue = document.getElementById('speed-value');
// 경로 주의: 파이썬 코드의 SPEED_PATH와 같아야 함
const SPEED_REF = database.ref('barista_control/setting/speed_ratio'); 

// 1. 슬라이더를 움직였을 때 -> Firebase에 저장
speedSlider.addEventListener('input', (e) => {
    const val = e.target.value;
    speedValue.innerText = Math.round(val * 100) + '%';
    SPEED_REF.set(parseFloat(val));
});

// 2. 새로고침 했을 때 -> Firebase에 저장된 값 불러오기 (동기화)
SPEED_REF.on('value', (snapshot) => {
    const val = snapshot.val();
    if (val !== null) {
        speedSlider.value = val;
        speedValue.innerText = Math.round(val * 100) + '%';
    }
});


# LLM 주문 명령(음성 → 구조화 명령) 구현 가이드

갱신: 2026-06-09. Order Task 시나리오 원문은 `docs/4_Task_Scenario.pdf`의 "Order Task" 참고.

이 문서는 "관리자(또는 고객)가 음성으로 주문 명령을 입력하면 STT가 텍스트로 변환하고
LLM이 이를 구조화된 명령으로 바꿔 Fleet Manager에게 전달한다"는 기능을 나중에 구현할 때
참고하기 위한 정리본이다.
아직 코드는 수정하지 않았고, **무엇을 어디에 어떻게 구현해야 하는지**만 정리했다.

> **구현 범위:** Order Task(주문)만 구현한다. Display Task(진열)는 구현 대상에서 제외한다.

---

## 1. 배경 — Order Task에서 LLM의 역할

`docs/4_Task_Scenario.pdf` Order Task 1단계 요약:

- 관리자(또는 고객)가 관리자 브라우저에서 **음성 데이터**를 입력한다.
  예: "수박 두개 식빵 한개 주문해줘"
- Web Service가 음성 데이터를 Cloud AI Server의 LLM Service로 전달한다.
- LLM Service는 먼저 **STT(Speech-to-Text)** 단계로 음성을 텍스트로 변환한다
  (사용 API: `gpt-4o-mini-transcribe`).
- 변환된 텍스트를 파싱해 **상품명 / 수량** 정보가 담긴 구조화된 주문 명령으로 변환한다.
  예: `{상품: 수박, 수량: 2}`, `{상품: 식빵, 수량: 1}`
- 변환된 명령은 Web Service를 거쳐 Fleet Manager로 전달되고, Fleet Manager는 주문 작업을
  생성해 내부 큐에 등록한다.

> **Zone 용어 정리 (헷갈리기 쉬운 부분)**
> `docs/3_System_Architecture.pdf`의 Map Design 기준으로 구역 명칭을 맞춰보면 다음과 같다.
>
> | 시나리오(`4_Task_Scenario.pdf`) 표현 | Map Design(영문) | DB(`db/seed.sql`) | 위치 / 용도 |
> |---|---|---|---|
> | 픽업존 | Pickup Zone / Pickup Slot | `PICKUP_ZONE_1~2` | 맵 우측. **Order Task** 전용 — 고객 주문 상품을 하차·검수하는 곳 |
> | (참고, 별개 구역) | Stock Zone | `STOCK_ZONE` | 맵 좌상단. **Display Task** 전용 — 진열 작업에서 상품을 파지(상차)하는 곳 |
> | (참고, 별개 구역) | Product Zone | `PRODUCT_ZONE_1~6` | 맵 중앙. **Display Task** 전용 — 진열 작업에서 파지한 상품을 배치하는 곳 |
>
> 즉 Order Task에서 주문 상품은 **Pickup Zone(픽업존)**에서 하차·검수된다.
> Stock Zone / Product Zone은 Display Task 전용 구역이라 Order Task 흐름과는 관계가 없다.

즉 "음성 → 구조화 주문 명령" 변환을 담당하는 부분이 STT + LLM이며, 현재 코드베이스에는
이 변환을 끼워 넣을 자리가 이미 stub 형태로 마련되어 있다.

### 음성 입력 시나리오 (구현 완료)

```text
1. 사용자가 고객 UI의 마이크 버튼을 클릭
        |
        v
2. 브라우저가 마이크 권한 요청 (navigator.mediaDevices.getUserMedia)
        |
        v
3. 마이크 활성화 — MediaRecorder로 녹음 시작
        |
        v
4. 사용자가 주문 명령을 말함
   예: "수박 두개 식빵 한개 주문해줘"
        |
        v
5. 무음 감지 — 1초간 음성 입력 없으면 녹음 자동 종료
   (SILENCE_DURATION_MS 상수로 조정 가능, 기본값 1000ms)
        |
        v
6. 녹음된 오디오 blob을 data:audio/webm;base64,... 형식으로 변환
        |
        v
7. POST /api/customer/llm/messages 로 data URL 전송
        |
        v
8. llm_client.py가 data URL 감지 후 _transcribe() 호출
   gpt-4o-mini-transcribe API가 오디오를 텍스트로 변환
   예: "수박 두개 식빵 한개 주문해줘"
        |
        v
9. LLM이 텍스트를 파싱해 구조화된 주문 명령 반환
   예: [{상품: 수박, 수량: 2}, {상품: 식빵, 수량: 1}]
```

---

## 8. 프론트엔드 STT 방식 변경 (2026-06-09 적용)

### 변경 배경

기존 `customer.js`는 브라우저 내장 Web Speech API(`window.SpeechRecognition`)를 사용해 음성을 텍스트로 변환한 뒤 텍스트를 백엔드에 전송했다. 이 방식에서는 `gpt-4o-mini-transcribe`가 전혀 호출되지 않아 백엔드 STT 구현이 동작하지 않는 문제가 있었다.

| 항목 | 변경 전 | 변경 후 |
|---|---|---|
| STT 처리 주체 | 브라우저 Web Speech API (Google 서버) | OpenAI gpt-4o-mini-transcribe |
| 백엔드로 전송하는 데이터 | 변환된 텍스트 문자열 | 오디오 data URL (`data:audio/webm;base64,...`) |
| 녹음 방식 | Web Speech API 자체 처리 | 브라우저 MediaRecorder로 직접 녹음 |
| 무음 감지 | Web Speech API 자동 처리 | RMS 기반 수동 감지 (100ms 간격) |
| 브라우저 제약 | Chrome / Edge 전용 | 브라우저 무관 |
| 비용 | 무료 | OpenAI API 과금 |
| 정확도 | 보통 | 높음 (상품명 오인식 감소) |

### 변경된 파일

`web/app/static/js/customer.js`

### 상세 변경 내용

#### 1. 상태 변수 교체

```javascript
// 변경 전
let voiceRecognition = null;
let voiceListening = false;
let voiceOrderSending = false;
let voiceRecognitionSupported = true;

// 변경 후
let mediaRecorder = null;       // MediaRecorder 인스턴스
let audioChunks = [];           // 녹음 데이터 조각 누적 배열
let voiceListening = false;
let voiceOrderSending = false;
let silenceCheckInterval = null; // 무음 감지 인터벌 ID
let silenceTimer = null;         // 무음 지속 후 녹음 종료 타이머 ID
const SILENCE_THRESHOLD = 0.01;  // RMS 무음 판정 임계값
const SILENCE_DURATION_MS = 1000; // 무음 지속 시간(ms), 이 시간이 지나면 녹음 종료
let voiceRecognitionSupported = true;
```

#### 2. `startVoiceRecognition()` 함수 교체

```javascript
// 변경 전: SpeechRecognition 객체를 생성하고 start() 호출
function startVoiceRecognition() {
  if (!voiceRecognition) {
    voiceRecognition = createVoiceRecognition();
  }
  if (voiceListening) {
    voiceRecognition.stop();
    return;
  }
  voiceRecognition.start();
}

// 변경 후: 녹음 중이면 중단, 아니면 startMediaRecording() 호출
function startVoiceRecognition() {
  if (!window.isSecureContext) { ... }
  if (voiceListening) {
    if (mediaRecorder && mediaRecorder.state === "recording") {
      mediaRecorder.stop();
    }
    return;
  }
  startMediaRecording();
}
```

#### 3. `createVoiceRecognition()` 제거 및 `startMediaRecording()` 신규 추가

기존 Web Speech API 초기화 함수(`createVoiceRecognition`)를 완전히 제거하고 `startMediaRecording` 비동기 함수로 대체했다.

```javascript
async function startMediaRecording() {
  // 1. MediaRecorder 지원 여부 확인
  if (!navigator.mediaDevices || !navigator.mediaDevices.getUserMedia || !window.MediaRecorder) {
    voiceRecognitionSupported = false;
    // 버튼 비활성화 및 에러 메시지 표시
    return;
  }

  // 2. 마이크 권한 요청
  let stream;
  try {
    stream = await navigator.mediaDevices.getUserMedia({ audio: true });
  } catch (err) {
    // NotAllowedError: 권한 거부 / NotFoundError: 마이크 없음 등 처리
    return;
  }

  // 3. UI 상태 업데이트
  voiceListening = true;
  voiceOrderMicButton?.classList.add("is-listening");
  setVoiceOrderFeedback("listening", "듣는 중", "...");
  audioChunks = [];

  // 4. Web Audio API로 무음 감지 설정
  const audioCtx = new AudioContext();
  const analyser = audioCtx.createAnalyser();
  audioCtx.createMediaStreamSource(stream).connect(analyser);
  // 100ms마다 RMS를 계산해 SILENCE_THRESHOLD 미만이면 silenceTimer 시작
  // 음성이 감지되면 silenceTimer 초기화
  silenceCheckInterval = setInterval(() => { /* RMS 체크 */ }, 100);

  // 5. MediaRecorder 설정 및 녹음 시작
  mediaRecorder = new MediaRecorder(stream);
  mediaRecorder.addEventListener("dataavailable", (event) => {
    audioChunks.push(event.data);  // 녹음 데이터 누적
  });
  mediaRecorder.addEventListener("stop", () => {
    // 인터벌/타이머 정리, 스트림 종료, AudioContext 닫기
    // audioChunks를 Blob으로 합쳐 FileReader로 data URL 변환
    // handleVoiceOrderMessage(dataUrl) 호출
  });
  mediaRecorder.start();
}
```

#### 4. `handleVoiceOrderMessage()` 피드백 텍스트 분기 추가

data URL 전체를 화면에 표시하는 것을 방지하기 위해 오디오 여부에 따라 표시 문구를 분기했다.

```javascript
// 변경 전
setVoiceOrderFeedback("running", "처리 중", `"${message}"`);

// 변경 후
const isAudio = message.startsWith("data:audio") || message.startsWith("data:video");
setVoiceOrderFeedback("running", "처리 중", isAudio ? "음성 변환 중..." : `"${message}"`);
```

### 무음 감지 동작 원리

```text
녹음 시작
    |
    v
100ms마다 AnalyserNode에서 RMS(음량) 계산
    |
    +-- RMS >= 0.01 (음성 있음): silenceTimer 초기화
    |
    +-- RMS < 0.01 (무음): 1000ms 타이머 시작
            |
            v
        타이머 만료 전에 음성 감지 시: 타이머 초기화
            |
            v
        타이머 만료 (1초간 계속 무음): mediaRecorder.stop() 호출
```

`SILENCE_THRESHOLD`(기본값 0.01)와 `SILENCE_DURATION_MS`(기본값 1000)는 `customer.js` 상단 상수로 선언되어 있어 조정이 쉽다.

---

## 2. 현재 구조와 호출 흐름

```text
[관리자 브라우저]
  -> POST /api/admin/llm/messages (음성 데이터 포함)   (web/app/routers/llm_router.py)
  -> build_llm_message(message)                         (web/app/services/llm_client.py)  <- 구현 지점
       1. STT: gpt-4o-mini-transcribe API로 음성 -> 텍스트 변환
       2. 변환된 텍스트를 LLM으로 파싱해 구조화된 주문 명령 추출
  -> action == "ORDER" 이면 _create_order_item(parsed)
  -> POST {FLEET_API_BASE_URL}/api/admin/orders          (Fleet Manager API)
```

관련 파일과 역할:

| 파일 | 역할 |
|---|---|
| `web/app/services/llm_client.py` | `build_llm_message()` — STT + LLM 호출 + 파싱 진입점 (현재 stub) |
| `web/app/routers/llm_router.py` | `/api/admin/llm/messages` 엔드포인트. 파싱 결과를 Fleet API로 위임 |
| `web/app/config.py`, `web/.env(.example)` | STT/LLM API 키 / provider / 모델명 같은 환경설정을 추가할 위치 |
| `web/requirements.txt` | STT/LLM SDK(예: `openai`) 의존성을 추가할 위치 |
| `src/.../fleet_manager/fleet_manager/fleet_api_schemas.py` | `OrderCreateIn` 등 Fleet API 입력 스키마 (참고용) |
| `db/seed.sql` | 6종 상품 카탈로그(상품명 ↔ product_id 매핑의 기준 데이터) |

---

## 3. 현재 stub 코드 상태

`build_llm_message`의 현재 반환값 (`web/app/services/llm_client.py:15-25`):

```python
{
    "result": "ok",
    "message": "LLM 명령 파싱은 아직 연결 대기 상태입니다. 담당 모듈에서 구현해주세요.",
    "action": "CHAT",
    "product_id": None,
    "product_name": None,
    "requested_quantity": None,
    "display_policy": None,
    "display_item_id": None,
    "provider": "stub",
}
```

함수 docstring에 "LLM 담당자는 이 함수만 실제 구현으로 교체하면 된다"고 명시되어 있다.
즉 이 함수의 시그니처(`message: str, context: dict | None`)와 호출부(`llm_router.py`)는
그대로 두고, 내부 구현 + 반환 스키마만 실제 동작에 맞게 바꾸면 된다.
stub의 단수 필드(`display_policy`, `display_item_id` 등)는 5절에서 `items` 리스트로
교체할 때 함께 제거한다. Display Task 관련 필드(`display_policy`, `display_item_id`)는
구현 대상이 아니므로 제거해도 무방하다.

### 6종 상품 카탈로그 (`db/seed.sql:42-49` 기준)

| product_id | 상품명 |
|---|---|
| 1 | 수박 |
| 2 | 식빵 |
| 3 | 환타 |
| 4 | 크림빵 |
| 5 | 초코파이 |
| 6 | 생수 |

> product_id는 seed 순서를 근거로 추정한 값이다. 실제 매핑은 하드코딩보다
> Fleet API `GET /api/products`를 호출해 받아온 목록을 쓰는 편이 안전하다
> (DB가 재시드되어 ID가 바뀌어도 코드 수정 없이 동작).

---

## 4. 1차 구현 단계 — 단일 상품 기준

1. `build_llm_message` 안에서 먼저 `gpt-4o-mini-transcribe` API를 호출해 음성 데이터를 텍스트로 변환한다.
2. 변환된 텍스트를 LLM에 전달해 파싱한다. 상품명 → `product_id` 매핑을 위해 카탈로그 정보를 LLM에게 제공한다.
   - 방법 A: 시스템 프롬프트에 6종 상품명·ID를 직접 넣기
   - 방법 B(권장): `GET /api/products`(Fleet API)를 호출해 최신 목록을 가져와 매핑
3. 반환 dict를 기존 stub 스키마에 맞춰 채운다.
   - `action`: `"ORDER"` 또는 `"CHAT"`
   - `product_id`, `product_name`, `quantity`
   - `provider`: 사용한 STT/LLM 모델 이름
4. `web/app/config.py` + `web/.env`(`.env.example`)에 `OPENAI_API_KEY`, `STT_MODEL=gpt-4o-mini-transcribe` 등
   환경변수를 추가한다.
5. `web/requirements.txt`에 `openai` SDK를 추가한다.

---

## 5. 다중 상품 지원

시나리오의 예시 문장 자체가 "수박 두개 식빵 한개 주문해줘", "콜라 2개 생수 1개 주문해줘"처럼
**한 문장에 여러 상품**을 담고 있다.

**Fleet API(llm_router.py 포함) 측 다중 상품 처리는 API 담당자가 이미 구현 완료.**
수정이 필요한 파일은 `web/app/services/llm_client.py` 하나뿐이다.

### 5-1. `web/app/services/llm_client.py` — `build_llm_message()`

단수 필드(`product_id` / `product_name` / `quantity` / `order_item_id`)를 제거하고,
리스트 필드 `items`로 교체한다.

```python
{
    "result": "ok",
    "message": "...",
    "action": "ORDER",
    "items": [
        {"product_id": 1, "product_name": "수박", "quantity": 2},
        {"product_id": 2, "product_name": "식빵", "quantity": 1},
    ],
    "provider": "...",
}
```

- LLM이 한 문장에서 여러 (상품명, 수량) 쌍을 추출하도록 프롬프트를 설계해야 한다
  (예: function calling / structured output으로 "items 배열"을 강제).

### 5-2. 프런트엔드 — 변경 불필요

`web/app/static/js/admin.js:4356`, `:4382-4387`에서 LLM 응답을 처리할 때
`response.result` / `response.message` **문자열만** 사용하고 있고, 개별
`product_id` / `order_item_id` 필드를 직접 파싱하지 않는다. 따라서 백엔드가
통합된 `message` 문자열만 잘 만들어 주면 `admin.html` / `admin.js`는 그대로 둬도 된다.

---

## 6. Fleet API 다중 상품 처리 — 구현 완료 (참고)

다중 상품 관련 Fleet API 측 구현은 API 담당자가 완료했다.
`llm_client.py`에서 `items` 리스트를 올바른 스키마로 반환하면 router와 Fleet API가
그대로 처리한다. 스키마 확인이 필요하면 아래 파일을 참고한다.

- `web/app/routers/llm_router.py` — ORDER 액션 분기 및 `items` 순회 후 Fleet API 호출 로직 (API 담당자 구현 예정)
- `src/.../fleet_manager/fleet_manager/fleet_api_schemas.py` — `OrderCreateIn` 스키마

> **참고:** `llm_router.py`에 기존 DISPLAY 분기(`_create_display_item`)가 있으나
> Display Task는 구현 대상이 아니므로 건드리지 않는다. ORDER 분기만 추가하면 된다.

---

## 7. 구현 체크리스트

- [x] `web/.env`(`.env.example`) + `config.py`에 STT/LLM provider / API 키 / 모델명 환경변수 추가 (`OPENAI_API_KEY`, `STT_MODEL=gpt-4o-mini-transcribe` 등)
- [x] `web/requirements.txt`에 OpenAI SDK(`openai`) 의존성 추가
- [x] `build_llm_message`: `gpt-4o-mini-transcribe`로 음성 → 텍스트 변환 후, 텍스트를 LLM으로 파싱해 다중 상품 `items` 리스트 반환으로 교체
- [x] 상품명 ↔ `product_id` 매핑 로직 (카탈로그 조회 또는 정적 매핑) 구현
- [x] (선택) `web/API_USAGE.md`의 `/api/customer/llm/messages` 예시를 다중 상품 입출력 예시로 갱신
- [x] 프론트엔드(`customer.js`) STT 방식 변경: Web Speech API에서 MediaRecorder + gpt-4o-mini-transcribe 방식으로 교체 (2026-06-09 완료)

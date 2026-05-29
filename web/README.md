# Just Pick It Web Gateway

`web/`는 고객/관리자 화면과 얇은 API 프록시만 담당한다.
DB 접근과 업무 상태 변경은 Fleet Manager 내부의 Fleet API/FleetRepository가 담당한다.

```text
Browser
  -> Web Gateway (:8000)
  -> Fleet API (:8100)
  -> FleetRepository
  -> just_pick_it_db
  -> PostgreSQL
```

## 실행 기준

- Ubuntu 24.04
- ROS 2 Jazzy
- Python 3.12
- FastAPI `0.101.0`
- Pydantic `1.10.14`

## 폴더 역할

```text
web/
├── app/
│   ├── main.py                         # FastAPI app 생성
│   ├── config.py                       # Web Gateway/Fleet API URL 설정
│   ├── routers/
│   │   ├── page_router.py              # HTML page route
│   │   ├── fleet_api_proxy_router.py   # /api/* HTTP/WebSocket proxy
│   │   └── llm_router.py               # 관리자 AI 명령 endpoint
│   ├── services/
│   │   └── llm_client.py               # LLM 담당자 구현 지점
│   ├── static/
│   └── templates/
├── scripts/
│   ├── setup.sh                        # web/.venv 세팅 only
│   └── run.sh                          # Web Gateway 실행 only
├── requirements.txt
└── .env.example
```

## 초기 세팅

전체 워크스페이스를 처음부터 재세팅하려면 루트에서 실행한다.

```bash
cd ~/just_pick_it
./reset_ws.sh
```

`reset_ws.sh`는 다음을 한 번에 수행한다.

```text
- web/.venv 재생성 및 requirements 설치
- PostgreSQL role/database/schema/seed 준비
- rosdep install --from-paths src --ignore-src -r -y
- build/install/log 삭제
- colcon build --symlink-install
```

팀원 환경을 맞출 때는 web venv만 따로 만지지 말고 루트 `./reset_ws.sh`로 전체 재세팅합니다.

## 실행

전체 로컬 stack:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_all.sh
```

Web Gateway만 실행:

```bash
cd ~/just_pick_it
web/scripts/run.sh
```

Web Gateway만 띄우면 화면은 열리지만, Fleet API가 없을 때 `/api/*`는 `503`이 정상이다.

## 접속 URL

```text
Customer UI : http://localhost:8000/customer
Admin UI    : http://localhost:8000/admin
Fleet API   : http://localhost:8100
DB Health   : http://localhost:8000/api/health/db
```

## API 원칙

- 브라우저는 `http://localhost:8000/api/*`만 호출한다.
- Web Gateway는 `/api/*` HTTP 요청과 `/api/*` WebSocket을 Fleet API로 전달한다.
- Web Gateway는 DB session, SQLAlchemy model, DB service를 import하지 않는다.
- LLM route는 Web Gateway에 남긴다. 단, 진열 요청 생성은 Fleet API에 위임한다.

## LLM 담당자 구현 지점

LLM 담당자는 다음 파일만 우선 구현하면 된다.

```text
web/app/services/llm_client.py
```

반환값이 아래처럼 `action="DISPLAY"`이면 Web Gateway가 Fleet API에 진열 요청을 생성한다.

```json
{
  "result": "ok",
  "action": "DISPLAY",
  "product_id": 1,
  "requested_quantity": 3,
  "display_policy": "REQUESTED_QUANTITY"
}
```

그 외 라우터/DB/FleetRepository는 LLM 담당자가 직접 건드리지 않는다.

## 데모 데이터 초기화

테스트 주문/진열/task를 지우고 seed 기준으로 되돌리려면 루트에서 실행한다.

```bash
cd ~/just_pick_it
./reset_demo_data.sh
```

DB schema까지 다시 만들 필요가 있으면 `./reset_ws.sh`를 사용한다.

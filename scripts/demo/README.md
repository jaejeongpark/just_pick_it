# Full Flow Demo

`run_full_flow_demo.sh`는 로컬 데모용 전체 스택을 띄운다.

```text
Fleet Manager API  : http://localhost:8100
Web Gateway        : http://localhost:8000
Fake robot servers : PICKY1/PICKY2/COBOT1/COBOT2 ROS2 action/service/topic
```

## 실행

```bash
./scripts/demo/run_full_flow_demo.sh
```

실행 후 브라우저에서 확인한다.

```text
Customer UI: http://localhost:8000/customer
Admin UI   : http://localhost:8000/admin
```

고객 주문은 Customer UI에서 넣으면 된다. Fleet Manager가 `ORDER_WAIT` 주문을 polling하고, fake robot server가 ROS2 action feedback/result를 반환해 task가 진행된다.

이미 Fleet API가 떠 있으면 스크립트가 먼저 종료하고 새 demo 설정으로 다시 띄운다.

## 설정

git에는 `full_flow_demo.env.example`만 공유한다.
처음 한 번만 이 파일을 `scripts/demo/full_flow_demo.env`로 복사한다.

```bash
cp scripts/demo/full_flow_demo.env.example scripts/demo/full_flow_demo.env
```

그 다음 `scripts/demo/full_flow_demo.env`에서 `DEMO_ROS_DOMAIN_ID`만 자기 환경에 맞게 채운다. (기본 설정값 : 25)

예:

```bash
export DEMO_ROS_DOMAIN_ID=25
```

`scripts/demo/full_flow_demo.env`는 개인 로컬 파일이라 git에 올리지 않는다.
나머지 값은 데모 기본값이므로 평소에는 수정하지 않는다.
fake PICKY 속도나 COBOT 작업 시간을 조절하고 싶을 때만 같은 `.env` 안의 값을 수정해서 쓴다.

기본 배터리 동작:

- 주문은 `UNLOAD`, 진열은 `DISPLAY_PLACE`가 끝날 때 해당 unit의 PICKY 배터리를 30% 차감한다.
- PICKY가 도킹 후 `CHARGING` 상태에 들어가면 5초 뒤 배터리를 100%로 채우고 `STANDBY`로 전환한다.
- Fleet Manager의 `CHARGE` task는 배터리가 30%를 넘으면 완료 조건을 만족한다.
  이미 30% 초과 상태에서 도킹하면 `CHARGE` task가 먼저 `SUCCESS`가 되고, fake battery 값은 몇 초 뒤 100%로 회복될 수 있다.

경로/도크 선택 기준:

- fake robot server는 경로를 만들지 않고 Fleet Manager가 보낸 waypoint만 따라간다.
- `RETURN_HOME`, `DOCK_IN`의 standby zone과 charging dock 선택은 Fleet Manager/TrafficManager 정책을 따른다.
- 따라서 두 PICKY가 같은 standby zone을 경유하거나 특정 dock을 선택하는 현상은 fake server가 아니라 Fleet/Traffic 경로 선택 정책에서 확인해야 한다.

## 진열 요청 curl

상품 목록 확인:

```bash
curl -s http://localhost:8100/api/products | python3 -m json.tool
```

상품 1번을 2개 진열 요청:

```bash
curl -s -X POST http://localhost:8100/api/admin/display-items \
  -H "Content-Type: application/json" \
  -d '{
    "product_id": 1,
    "requested_quantity": 2,
    "display_policy": "REQUESTED_QUANTITY"
  }' | python3 -m json.tool
```

특정 robot unit에 맡기고 싶을 때:

```bash
curl -s -X POST http://localhost:8100/api/admin/display-items \
  -H "Content-Type: application/json" \
  -d '{
    "product_id": 1,
    "requested_quantity": 2,
    "display_policy": "REQUESTED_QUANTITY",
    "assigned_unit_id": 1
  }' | python3 -m json.tool
```

수량을 지정하지 않고 코봇 처리 결과 기준으로 진열하려면:

```bash
curl -s -X POST http://localhost:8100/api/admin/display-items \
  -H "Content-Type: application/json" \
  -d '{
    "product_id": 1,
    "requested_quantity": null,
    "display_policy": "ALL_PROCESSED"
  }' | python3 -m json.tool
```

## 종료

터미널에서 `Ctrl-C`를 누르면 demo script가 Web Gateway와 fake robot servers를 종료한다.

`run_full_flow_demo.sh`가 직접 띄운 Fleet Manager도 함께 종료한다.

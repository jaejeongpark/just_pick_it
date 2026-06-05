# Just Pick It DB

PostgreSQL schema와 초기 seed 데이터를 두는 폴더입니다.

실제 PostgreSQL database 이름은 `just_pick_it`입니다.
`just_pick_it_db`는 SQLAlchemy model/session/service를 담은 Python 패키지 이름입니다.

## Structure

```text
db/
├── schema.sql
├── seed.sql
└── README.md
```

## 자동 세팅

팀원 공통 DB 세팅은 repo root의 재세팅 스크립트가 담당합니다.

```bash
cd ~/just_pick_it
./reset_ws.sh
```

DB schema/seed는 팀원 환경을 맞출 때 `./reset_ws.sh` 기본 실행으로 다시 만듭니다.
부분 초기화 옵션은 팀 공통 절차로 안내하지 않습니다.

## 수동 적용

```bash
psql "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it" -f db/schema.sql
psql "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it" -f db/seed.sql
```

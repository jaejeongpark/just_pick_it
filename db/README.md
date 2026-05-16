# Just Pick It DB

PostgreSQL schema와 초기 seed 데이터를 두는 폴더입니다.

`Data Structure.pdf`를 기준 문서로 보고, 웹 구현 단계에서 필요한 SQL을 `schema.sql`과 `seed.sql`에 채워 넣습니다.

## Structure

```text
db/
├── schema.sql
├── seed.sql
└── README.md
```

## Local Setup

PostgreSQL이 설치되어 있지 않으면 먼저 설치합니다.

```bash
sudo apt update
sudo apt install postgresql postgresql-contrib
```

DB와 계정을 생성합니다.

```bash
sudo -u postgres psql
```

PostgreSQL 콘솔에서:

```sql
CREATE DATABASE just_pick_it;
CREATE USER just_pick_it_user WITH PASSWORD 'just_pick_it_pw';
GRANT ALL PRIVILEGES ON DATABASE just_pick_it TO just_pick_it_user;
\q
```

PostgreSQL 15 이상에서는 `public` schema 생성 권한을 따로 줘야 할 수 있습니다.

```bash
sudo -u postgres psql -d just_pick_it
```

PostgreSQL 콘솔에서:

```sql
ALTER SCHEMA public OWNER TO just_pick_it_user;
GRANT ALL ON SCHEMA public TO just_pick_it_user;
\q
```

schema와 seed를 적용합니다.

```bash
psql "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it" -f db/schema.sql
psql "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it" -f db/seed.sql
```

## Auto Setup

웹 실행 환경까지 한 번에 준비할 때는 루트에서 아래 스크립트를 사용합니다.

```bash
cd ~/just_pick_it
web/scripts/setup.sh
```

데모 DB를 초기 상태로 다시 만들고 싶으면:

```bash
RESET_DB=1 web/scripts/setup.sh
```

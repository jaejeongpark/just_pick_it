#!/usr/bin/env python3
import argparse
import os
import subprocess
import sys
from pathlib import Path

from dotenv import load_dotenv
from fastapi.testclient import TestClient


SCRIPT_PATH = Path(__file__).resolve()
WEB_DIR = SCRIPT_PATH.parents[1]
ROOT_DIR = SCRIPT_PATH.parents[2]
DEFAULT_DB_URL = "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it"

sys.path.insert(0, str(WEB_DIR))
load_dotenv(WEB_DIR / ".env")

from app.main import app  # noqa: E402


EXPECTED_ORDER_TASKS = (
    ("STANDBY_LOAD", "AMR_1"),
    ("SORTING", "SORTING_COBOT"),
    ("LOAD", "SORTING_COBOT"),
    ("STANDBY_UNLOAD", "AMR_1"),
    ("INSPECTION", "INSPECTION_COBOT"),
    ("UNLOAD", "INSPECTION_COBOT"),
)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Smoke test the Control Server <-> Fleet runtime API flow.",
    )
    parser.add_argument(
        "--reset-db",
        action="store_true",
        help="Drop public schema and reapply db/schema.sql + db/seed.sql before running.",
    )
    args = parser.parse_args()

    if args.reset_db:
        reset_database()

    with TestClient(app) as client:
        product = first_available_product(client)
        order = create_order(client, product["product_id"])
        order_id = order["order_id"]

        print_step(f"created order {order['order_no']} with product {product['name']}")

        tasks = get_order_tasks(client, order_id)
        assert_task_plan(tasks)
        report_robot_available(client, "AMR_1")

        for expected_task_type, expected_robot_id in EXPECTED_ORDER_TASKS:
            task = assigned_task_for_robot(client, expected_robot_id, expected_task_type)
            run_task(client, task)
            finish_task(client, task)
            print_step(f"{expected_task_type} completed by {expected_robot_id}")

        final_order = get_order(client, order_id)
        assert_equal(final_order["status"], "PICKUP_READY", "final order status")
        print_step(f"order {final_order['order_no']} is PICKUP_READY")

        llm_response = create_patrol_task(client)
        assert_equal(llm_response["result"], "ok", "LLM patrol result")
        assert_equal(llm_response["action"], "PATROL", "LLM patrol action")
        assert_equal(llm_response["target_zone_name"], "A_ZONE", "LLM patrol zone")
        assert_equal(llm_response["assigned_robot_id"], None, "LLM patrol initial robot")
        report_robot_available(client, "AMR_2")
        patrol_task = assigned_task_for_robot(client, "AMR_2", "PATROL")
        print_step(f"patrol task {patrol_task['task_id']} assigned to {patrol_task['assigned_robot_id']}")

        admin_status = get_admin_status(client)
        assert_true(admin_status["tasks"], "admin status has tasks")
        assert_true(admin_status["products"], "admin status has products")

    print("[smoke] PASS")


def reset_database() -> None:
    db_url = os.getenv("DATABASE_URL") or DEFAULT_DB_URL
    print_step("resetting database")

    run_command(
        [
            "psql",
            db_url,
            "-v",
            "ON_ERROR_STOP=1",
            "-c",
            "DROP SCHEMA public CASCADE; CREATE SCHEMA public;",
        ]
    )
    run_command(["psql", db_url, "-v", "ON_ERROR_STOP=1", "-f", str(ROOT_DIR / "db/schema.sql")])
    run_command(["psql", db_url, "-v", "ON_ERROR_STOP=1", "-f", str(ROOT_DIR / "db/seed.sql")])


def run_command(command: list[str]) -> None:
    subprocess.run(command, cwd=ROOT_DIR, check=True)


def first_available_product(client: TestClient) -> dict:
    response = client.get("/api/products")
    assert_ok(response, "GET /api/products")

    products = response.json()
    for product in products:
        if product["stock_qty"] > 0:
            return product

    raise AssertionError("no product with stock_qty > 0")


def create_order(client: TestClient, product_id: int) -> dict:
    response = client.post(
        "/api/orders",
        json={
            "items": [
                {
                    "product_id": product_id,
                    "quantity": 1,
                }
            ]
        },
    )
    assert_ok(response, "POST /api/orders")
    return response.json()


def get_order(client: TestClient, order_id: int) -> dict:
    response = client.get(f"/api/orders/{order_id}")
    assert_ok(response, f"GET /api/orders/{order_id}")
    return response.json()


def get_order_tasks(client: TestClient, order_id: int) -> list[dict]:
    response = client.get(f"/api/fleet/orders/{order_id}/tasks")
    assert_ok(response, f"GET /api/fleet/orders/{order_id}/tasks")
    return response.json()


def assert_task_plan(tasks: list[dict]) -> None:
    task_types = [task["task_type"] for task in tasks]
    expected_task_types = [task_type for task_type, _robot_id in EXPECTED_ORDER_TASKS]
    assert_equal(task_types, expected_task_types, "order task plan")

    for task in tasks:
        assert_equal(task["status"], "QUEUED", f"{task['task_type']} initial status")
        assert_equal(
            task["assigned_robot_id"],
            initial_robot_for_task(task["task_type"]),
            f"{task['task_type']} initial robot",
        )


def initial_robot_for_task(task_type: str) -> str | None:
    return {
        "SORTING": "SORTING_COBOT",
        "LOAD": "SORTING_COBOT",
        "INSPECTION": "INSPECTION_COBOT",
        "UNLOAD": "INSPECTION_COBOT",
    }.get(task_type)


def report_robot_available(client: TestClient, robot_id: str) -> None:
    response = client.patch(
        f"/api/fleet/robots/{robot_id}",
        json={
            "status": "STANDBY" if robot_id.startswith("AMR_") else "IDLE",
            "current_task_id": None,
            "battery_level": 100 if robot_id.startswith("AMR_") else None,
        },
    )
    assert_ok(response, f"PATCH robot {robot_id} available")


def assigned_task_for_robot(
    client: TestClient,
    robot_id: str,
    expected_task_type: str,
) -> dict:
    response = client.get(
        "/api/fleet/tasks",
        params={
            "robot_id": robot_id,
            "status": "ASSIGNED",
        },
    )
    assert_ok(response, f"GET assigned task for {robot_id}")
    tasks = response.json()

    for task in tasks:
        if task["task_type"] == expected_task_type:
            assert_true(task["target_zone_pose"] or expected_task_type == "UNLOAD", "task target pose")
            return task

    raise AssertionError(f"{robot_id} has no assigned {expected_task_type} task: {tasks}")


def run_task(client: TestClient, task: dict) -> None:
    response = client.patch(
        f"/api/fleet/tasks/{task['task_id']}",
        json={
            "status": "RUNNING",
        },
    )
    assert_ok(response, f"PATCH task {task['task_id']} RUNNING")


def finish_task(client: TestClient, task: dict) -> None:
    response = client.patch(
        f"/api/fleet/tasks/{task['task_id']}",
        json={
            "status": "SUCCESS",
        },
    )
    assert_ok(response, f"PATCH task {task['task_id']} SUCCESS")


def create_patrol_task(client: TestClient) -> dict:
    response = client.post(
        "/api/admin/llm/messages",
        json={
            "message": "A 구역 순찰해줘",
        },
    )
    assert_ok(response, "POST /api/admin/llm/messages")
    return response.json()


def get_admin_status(client: TestClient) -> dict:
    response = client.get("/api/admin/status")
    assert_ok(response, "GET /api/admin/status")
    return response.json()


def assert_ok(response, label: str) -> None:
    if response.status_code >= 400:
        raise AssertionError(f"{label} failed: {response.status_code} {response.text}")


def assert_equal(actual, expected, label: str) -> None:
    if actual != expected:
        raise AssertionError(f"{label}: expected {expected!r}, got {actual!r}")


def assert_true(value, label: str) -> None:
    if not value:
        raise AssertionError(f"{label}: expected truthy value, got {value!r}")


def print_step(message: str) -> None:
    print(f"[smoke] {message}")


if __name__ == "__main__":
    main()

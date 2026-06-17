#!/usr/bin/env python3
"""
Cobot ExecuteTask 액션 서버 디버깅용 가상 클라이언트 GUI.

cobot_state_machine 의 ExecuteTask 액션 서버에 task 를 가상으로 발급하고, feedback 과
result 를 실시간으로 확인한다. 여러 task 를 큐에 넣고 result 수신 시 다음 task 로 자동
진행시켜, 단계별 흐름(다음 task 로 넘어가는지)을 검증할 수 있다.

실행:
  ros2 run picky_cobot_1 task_debug_gui
  (cobot_state_manager 가 떠 있어야 한다. 액션 이름 기본값은 /cobot1/execute_task)

스레드 모델:
  - rclpy executor 는 백그라운드 스레드에서 spin.
  - 액션 콜백(feedback/result)은 executor 스레드에서 실행되므로 tkinter 위젯을 직접
    건드리지 않고 thread-safe 큐에 이벤트만 넣는다.
  - GUI 스레드는 root.after 로 큐를 주기적으로 비우며 위젯을 갱신한다.
"""
import queue
import threading
import tkinter as tk
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from just_pick_it_interfaces.action import ExecuteTask


TASK_TYPES = [
    'SORTING_AND_LOAD',
    'INSPECTION',
    'UNLOAD',
    'DISPLAY_PLACE',
]

PRODUCTS = ['fanta', 'water', 'watermelon', 'bread', 'cream_bread', 'choco_pie']


class TaskDebugGUI:
    def __init__(self, root: tk.Tk, node) -> None:
        self._root = root
        self._node = node
        self._ui_queue: queue.Queue = queue.Queue()

        self._client: ActionClient | None = None
        self._flush_client = None
        self._seed_pub = None
        self._goal_handle = None
        self._auto_run = False
        self._next_task_id = 1
        self._task_queue: list[dict] = []

        root.title('Cobot ExecuteTask Debug Client')
        self._build_widgets()
        self._connect()  # 기본 액션 이름으로 클라이언트 생성

        root.after(100, self._drain_ui_queue)

    # ── 위젯 구성 ────────────────────────────────────────────────────────

    def _build_widgets(self) -> None:
        pad = {'padx': 4, 'pady': 2}

        # 연결.
        conn = ttk.LabelFrame(self._root, text='연결')
        conn.grid(row=0, column=0, columnspan=2, sticky='ew', padx=6, pady=4)
        ttk.Label(conn, text='Action:').grid(row=0, column=0, **pad)
        self._action_var = tk.StringVar(value='/cobot1/execute_task')
        ttk.Entry(conn, textvariable=self._action_var, width=32).grid(row=0, column=1, **pad)
        ttk.Button(conn, text='재연결', command=self._connect).grid(row=0, column=2, **pad)
        self._server_var = tk.StringVar(value='서버 확인 중...')
        ttk.Label(conn, textvariable=self._server_var).grid(row=0, column=3, **pad)

        # Task 빌더.
        builder = ttk.LabelFrame(self._root, text='Task 생성')
        builder.grid(row=1, column=0, sticky='nsew', padx=6, pady=4)

        ttk.Label(builder, text='task_type').grid(row=0, column=0, sticky='e', **pad)
        self._type_var = tk.StringVar(value=TASK_TYPES[0])
        ttk.Combobox(builder, textvariable=self._type_var, values=TASK_TYPES,
                     state='readonly', width=20).grid(row=0, column=1, **pad)

        ttk.Label(builder, text='task_id').grid(row=1, column=0, sticky='e', **pad)
        self._task_id_var = tk.StringVar(value='1')
        ttk.Entry(builder, textvariable=self._task_id_var, width=22).grid(row=1, column=1, **pad)

        ttk.Label(builder, text='product_name').grid(row=2, column=0, sticky='e', **pad)
        self._product_var = tk.StringVar(value='watermelon')
        ttk.Combobox(builder, textvariable=self._product_var, values=PRODUCTS,
                     width=20).grid(row=2, column=1, **pad)

        ttk.Label(builder, text='quantity').grid(row=3, column=0, sticky='e', **pad)
        self._qty_var = tk.StringVar(value='1')
        ttk.Spinbox(builder, from_=0, to=99, textvariable=self._qty_var,
                    width=20).grid(row=3, column=1, **pad)

        ttk.Label(builder, text='order_id').grid(row=4, column=0, sticky='e', **pad)
        self._order_var = tk.StringVar(value='0')
        ttk.Entry(builder, textvariable=self._order_var, width=22).grid(row=4, column=1, **pad)

        ttk.Label(builder, text='display_item_id').grid(row=5, column=0, sticky='e', **pad)
        self._display_var = tk.StringVar(value='0')
        ttk.Entry(builder, textvariable=self._display_var, width=22).grid(row=5, column=1, **pad)

        ttk.Label(builder, text='target_zone_name').grid(row=6, column=0, sticky='e', **pad)
        self._zone_var = tk.StringVar(value='')
        ttk.Entry(builder, textvariable=self._zone_var, width=22).grid(row=6, column=1, **pad)

        btns = ttk.Frame(builder)
        btns.grid(row=7, column=0, columnspan=2, **pad)
        ttk.Button(btns, text='바로 전송', command=self._send_now).grid(row=0, column=0, padx=3)
        ttk.Button(btns, text='큐에 추가', command=self._add_to_queue).grid(row=0, column=1, padx=3)
        ttk.Button(btns, text='취소(cancel)', command=self._cancel_current).grid(row=0, column=2, padx=3)

        # 큐.
        qframe = ttk.LabelFrame(self._root, text='Task 큐 (result 수신 시 자동 진행)')
        qframe.grid(row=1, column=1, sticky='nsew', padx=6, pady=4)
        self._queue_list = tk.Listbox(qframe, width=42, height=10)
        self._queue_list.grid(row=0, column=0, columnspan=3, **pad)
        ttk.Button(qframe, text='다음 전송', command=self._send_next).grid(row=1, column=0, padx=3)
        ttk.Button(qframe, text='자동 실행', command=self._run_all).grid(row=1, column=1, padx=3)
        ttk.Button(qframe, text='자동 중단', command=self._stop_auto).grid(row=1, column=2, padx=3)
        ttk.Button(qframe, text='선택 삭제', command=self._remove_selected).grid(row=2, column=0, padx=3)
        ttk.Button(qframe, text='큐 비우기', command=self._clear_queue).grid(row=2, column=1, padx=3)
        self._stop_on_fail = tk.BooleanVar(value=True)
        ttk.Checkbutton(qframe, text='실패 시 중단', variable=self._stop_on_fail).grid(
            row=2, column=2, padx=3)

        # 진행 상태.
        status = ttk.LabelFrame(self._root, text='현재 상태')
        status.grid(row=2, column=0, columnspan=2, sticky='ew', padx=6, pady=4)
        self._cur_var = tk.StringVar(value='대기 중')
        ttk.Label(status, textvariable=self._cur_var).grid(
            row=0, column=0, columnspan=2, sticky='w', **pad)
        self._progress = ttk.Progressbar(status, length=420, maximum=1.0)
        self._progress.grid(row=1, column=0, columnspan=2, sticky='w', **pad)
        ttk.Button(status, text='바구니 flush(적재 초기화)', command=self._flush_loadout).grid(
            row=0, column=2, sticky='e', **pad)

        # 가상 적재(디버그): SORTING_AND_LOAD 없이 적재 DB 를 주입해 INSPECTION/UNLOAD/
        # DISPLAY_PLACE 단독 테스트. 쉼표로 구분(적재 순서 = 슬롯 0,1,2,3). 빈 값=초기화.
        ttk.Label(status, text='가상 적재(쉼표구분):').grid(row=2, column=0, sticky='e', **pad)
        self._seed_var = tk.StringVar(value='water,water,cream_bread')
        ttk.Entry(status, textvariable=self._seed_var, width=40).grid(
            row=2, column=1, sticky='w', **pad)
        ttk.Button(status, text='적재 주입(seed)', command=self._seed_loadout).grid(
            row=2, column=2, sticky='e', **pad)

        # 로그.
        logf = ttk.LabelFrame(self._root, text='Feedback / Result 로그')
        logf.grid(row=3, column=0, columnspan=2, sticky='nsew', padx=6, pady=4)
        self._log = ScrolledText(logf, width=80, height=16, state='disabled')
        self._log.grid(row=0, column=0, sticky='nsew')
        ttk.Button(logf, text='로그 지우기', command=self._clear_log).grid(row=1, column=0, sticky='e')

        self._root.columnconfigure(0, weight=1)
        self._root.columnconfigure(1, weight=1)
        self._root.rowconfigure(3, weight=1)

    # ── 연결 / 서버 확인 ─────────────────────────────────────────────────

    def _connect(self) -> None:
        action_name = self._action_var.get().strip()
        if self._client is not None:
            self._client.destroy()
        self._client = ActionClient(self._node, ExecuteTask, action_name)
        # flush 서비스는 액션 이름에서 파생: /cobot1/execute_task -> /cobot1/flush_loadout
        flush_name = (action_name.rsplit('/', 1)[0] + '/flush_loadout'
                      if '/' in action_name else 'flush_loadout')
        if self._flush_client is not None:
            self._node.destroy_client(self._flush_client)
        self._flush_client = self._node.create_client(Trigger, flush_name)
        # seed 토픽도 액션 이름에서 파생: /cobot1/execute_task -> /cobot1/seed_loadout
        seed_name = (action_name.rsplit('/', 1)[0] + '/seed_loadout'
                     if '/' in action_name else 'seed_loadout')
        if self._seed_pub is not None:
            self._node.destroy_publisher(self._seed_pub)
        self._seed_pub = self._node.create_publisher(String, seed_name, 10)
        self._server_var.set('서버 확인 중...')
        threading.Thread(target=self._check_server, daemon=True).start()

    def _flush_loadout(self) -> None:
        threading.Thread(target=self._do_flush, daemon=True).start()

    def _do_flush(self) -> None:
        client = self._flush_client
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            self._ui_queue.put(('log', '[flush] 서비스 미연결 — cobot_state_manager 확인'))
            return
        self._ui_queue.put(('log', '[flush] picky 적재 초기화 요청'))
        future = client.call_async(Trigger.Request())
        future.add_done_callback(self._on_flush_response)

    def _on_flush_response(self, future) -> None:
        try:
            resp = future.result()
            self._ui_queue.put(('log', f'[flush] success={resp.success} :: {resp.message}'))
        except Exception as exc:  # noqa: BLE001
            self._ui_queue.put(('log', f'[flush] 오류: {exc}'))

    def _seed_loadout(self) -> None:
        if self._seed_pub is None:
            self._append_log('[seed] 퍼블리셔 미생성 — 재연결 후 다시 시도')
            return
        text = self._seed_var.get().strip()
        msg = String()
        msg.data = text
        self._seed_pub.publish(msg)
        self._append_log(
            f'[seed] 가상 적재 발행: {text or "(빈 값 = 적재 초기화)"} '
            '(cobot_state_manager 로그에서 결과 확인)')

    def _check_server(self) -> None:
        ok = self._client.wait_for_server(timeout_sec=3.0)
        self._ui_queue.put((
            'server',
            f'서버 연결됨: {self._action_var.get()}' if ok else '서버 없음(대기)',
        ))

    # ── Task 빌드/전송 ───────────────────────────────────────────────────

    def _form_to_task(self) -> dict | None:
        try:
            task = {
                'task_id': int(self._task_id_var.get() or 0),
                'task_type': self._type_var.get(),
                'product_name': self._product_var.get(),
                'quantity': int(self._qty_var.get() or 0),
                'order_id': int(self._order_var.get() or 0),
                'display_item_id': int(self._display_var.get() or 0),
                'target_zone_name': self._zone_var.get(),
            }
        except ValueError:
            self._append_log('[오류] 숫자 필드에 잘못된 값이 있습니다')
            return None
        return task

    def _bump_task_id(self) -> None:
        self._next_task_id = max(self._next_task_id, int(self._task_id_var.get() or 0)) + 1
        self._task_id_var.set(str(self._next_task_id))

    def _send_now(self) -> None:
        task = self._form_to_task()
        if task is None:
            return
        self._bump_task_id()
        self._send_task(task)

    def _add_to_queue(self) -> None:
        task = self._form_to_task()
        if task is None:
            return
        self._task_queue.append(task)
        self._queue_list.insert(tk.END, self._task_summary(task))
        self._bump_task_id()

    def _send_next(self) -> None:
        if not self._task_queue:
            self._append_log('[큐] 비어 있음')
            return
        task = self._task_queue.pop(0)
        self._queue_list.delete(0)
        self._send_task(task)

    def _run_all(self) -> None:
        if not self._task_queue:
            self._append_log('[큐] 비어 있음')
            return
        self._auto_run = True
        self._append_log('[자동] 큐 자동 실행 시작')
        self._send_next()

    def _stop_auto(self) -> None:
        self._auto_run = False
        self._append_log('[자동] 자동 진행 중단')

    def _remove_selected(self) -> None:
        sel = self._queue_list.curselection()
        if not sel:
            return
        idx = sel[0]
        self._queue_list.delete(idx)
        del self._task_queue[idx]

    def _clear_queue(self) -> None:
        self._task_queue.clear()
        self._queue_list.delete(0, tk.END)

    def _send_task(self, task: dict) -> None:
        if self._client is None or not self._client.server_is_ready():
            self._append_log('[오류] 액션 서버 미연결 — 재연결 후 다시 시도')
            self._auto_run = False
            return

        goal = ExecuteTask.Goal()
        goal.task_id = task['task_id']
        goal.task_type = task['task_type']
        goal.product_name = task['product_name']
        goal.quantity = task['quantity']
        goal.order_id = task['order_id']
        goal.display_item_id = task['display_item_id']
        goal.target_zone_name = task['target_zone_name']

        self._cur_var.set(f'전송: {self._task_summary(task)}')
        self._progress['value'] = 0.0
        self._append_log(f'>>> 전송 {self._task_summary(task)}')

        send_future = self._client.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    # ── 액션 콜백(executor 스레드) -> ui_queue ───────────────────────────

    def _on_feedback(self, feedback_msg) -> None:
        fb = feedback_msg.feedback
        self._ui_queue.put(('feedback', {
            'state': fb.state,
            'message': fb.message,
            'progress': float(fb.progress),
            'processed_quantity': int(fb.processed_quantity),
        }))

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._ui_queue.put(('rejected', None))
            return
        self._goal_handle = goal_handle
        self._ui_queue.put(('accepted', None))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        wrapper = future.result()
        result = wrapper.result
        self._ui_queue.put(('result', {
            'success': bool(result.success),
            'status': result.status,
            'message': result.message,
            'processed_quantity': int(result.processed_quantity),
            'stock_delta': int(result.stock_delta),
        }))

    def _cancel_current(self) -> None:
        if self._goal_handle is None:
            self._append_log('[취소] 진행 중 goal 없음')
            return
        self._goal_handle.cancel_goal_async()
        self._append_log('[취소] cancel 요청 전송')

    # ── ui_queue 소비(GUI 스레드) ────────────────────────────────────────

    def _drain_ui_queue(self) -> None:
        try:
            while True:
                kind, payload = self._ui_queue.get_nowait()
                self._handle_event(kind, payload)
        except queue.Empty:
            pass
        self._root.after(100, self._drain_ui_queue)

    def _handle_event(self, kind: str, payload) -> None:
        if kind == 'server':
            self._server_var.set(payload)
        elif kind == 'log':
            self._append_log(payload)
        elif kind == 'accepted':
            self._append_log('    goal ACCEPTED')
        elif kind == 'rejected':
            self._append_log('    goal REJECTED')
            self._after_result(success=False)
        elif kind == 'feedback':
            self._progress['value'] = payload['progress']
            self._cur_var.set(
                f"[{payload['state']}] {payload['message']} "
                f"(progress={payload['progress']:.2f}, qty={payload['processed_quantity']})"
            )
            self._append_log(
                f"    FB state={payload['state']} progress={payload['progress']:.2f} "
                f"qty={payload['processed_quantity']} :: {payload['message']}"
            )
        elif kind == 'result':
            self._append_log(
                f"<<< RESULT success={payload['success']} status={payload['status']} "
                f"qty={payload['processed_quantity']} stock_delta={payload['stock_delta']} "
                f":: {payload['message']}"
            )
            self._goal_handle = None
            self._after_result(success=payload['success'])

    def _after_result(self, success: bool) -> None:
        """result 수신 후 자동 진행 처리."""
        if not self._auto_run:
            return
        if not success and self._stop_on_fail.get():
            self._auto_run = False
            self._append_log('[자동] 실패로 중단')
            return
        if self._task_queue:
            self._append_log('[자동] 다음 task 전송')
            self._send_next()
        else:
            self._auto_run = False
            self._append_log('[자동] 큐 소진 — 완료')

    # ── 유틸 ─────────────────────────────────────────────────────────────

    @staticmethod
    def _task_summary(task: dict) -> str:
        return (f"{task['task_type']} id={task['task_id']} "
                f"{task['product_name']} x{task['quantity']} order={task['order_id']}")

    def _append_log(self, line: str) -> None:
        self._log.configure(state='normal')
        self._log.insert(tk.END, line + '\n')
        self._log.see(tk.END)
        self._log.configure(state='disabled')

    def _clear_log(self) -> None:
        self._log.configure(state='normal')
        self._log.delete('1.0', tk.END)
        self._log.configure(state='disabled')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node('task_debug_gui_client')

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    TaskDebugGUI(root, node)
    try:
        root.mainloop()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

import time
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ChatMessage

def clear_screen():
    print("\033[2J\033[H", end="")

COLORS = [
    "\033[31m",  
    "\033[32m",  
    "\033[33m",  
    "\033[34m",  
    "\033[35m",  
    "\033[36m",  
    "\033[91m",  
    "\033[92m",  
    "\033[93m",  
    "\033[94m",  
    "\033[95m",  
    "\033[96m",  
]
RESET = "\033[0m"

def color_for(name):
    return COLORS[hash(name) % len(COLORS)]

def colorize(name):
    return f"{color_for(name)}{name}{RESET}"


class ChatNode(Node):
    def __init__(self, username):
        super().__init__(f'chat_node_{username}')
        self.username = username
        self.chat_history = deque(maxlen=10)
        self.users = set([self.username])  
        self._data_lock = threading.Lock()   
        self._render_lock = threading.Lock()
        self.publisher_ = self.create_publisher(ChatMessage, 'chat', 10)
        self.subscription = self.create_subscription(
            ChatMessage, 'chat', self.listener_callback, 10
        )
        self.presence_pub = self.create_publisher(ChatMessage, 'presence', 10)
        self.presence_sub = self.create_subscription(
            ChatMessage, 'presence', self.presence_callback, 10
        )
        self._last_presence_broadcast = 0.0
        self._presence_timer = self.create_timer(2.0, self._announce_presence)
        self._announce_presence()
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()
        self._render_snapshot([], len(self.users))
    def listener_callback(self, msg: ChatMessage):
        with self._data_lock:
            self.users.add(msg.sender)
            self.chat_history.append(f"{colorize(msg.sender)}: {msg.text}")
            hist = list(self.chat_history)
            cnt  = len(self.users)
        self._render_snapshot(hist, cnt)

    def presence_callback(self, msg: ChatMessage):
        if msg.sender == self.username:
            return
        rebroadcast = False
        with self._data_lock:
            if msg.sender not in self.users:
                self.users.add(msg.sender)
                rebroadcast = True
            hist = list(self.chat_history)
            cnt  = len(self.users)
        self._render_snapshot(hist, cnt)
        now = time.monotonic()
        if rebroadcast and now - self._last_presence_broadcast > 0.5:
            self._announce_presence()
    def _announce_presence(self):
        self._last_presence_broadcast = time.monotonic()
        m = ChatMessage()
        m.sender = self.username
        m.text = "" 
        self.presence_pub.publish(m)

    def _input_loop(self):
        while rclpy.ok():
            try:
                text = input()
            except (EOFError, KeyboardInterrupt):
                rclpy.shutdown()
                return
            text = text.strip()

            if not text:
                with self._data_lock:
                    hist = list(self.chat_history)
                    cnt  = len(self.users)
                self._render_snapshot(hist, cnt)
                continue
            msg = ChatMessage()
            msg.sender = self.username
            msg.text   = text
            self.publisher_.publish(msg)

            with self._data_lock:
                self.users.add(self.username)  
                hist = list(self.chat_history)
                cnt  = len(self.users)
            self._render_snapshot(hist, cnt)

    def _render_snapshot(self, history_snapshot, user_count):
        with self._data_lock:
            users_list = sorted(self.users, key=str.lower)
            colored_users = ", ".join(colorize(u) for u in users_list)

        with self._render_lock:
            clear_screen()
            print("==== Group Chat ====")
            for line in history_snapshot:
                print(line)
            print("\nActive users:", user_count)
            print("Users:", colored_users if users_list else "(none)")
            print("====================")
            print("Type your message and press Enter (Ctrl+C to quit):")


def main():
    rclpy.init()
    try:
        username = input("Enter your username: ").strip()
        if not username:
            username = "anon"
    except (EOFError, KeyboardInterrupt):
        print("\nNo username; exiting.")
        return

    node = ChatNode(username)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

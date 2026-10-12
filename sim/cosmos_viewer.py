#!/usr/bin/env python3
"""Live window for the Cosmos 3 Edge rover agent: what the model sees, thinks and decides.

Reads the files sim/cosmos_server.py writes while it generates (~/.ros/cosmos_runs/live):
the camera frame and status report of the current request, the reasoning as it streams,
and the final action. Needs no ROS; run it any time:  python3 sim/cosmos_viewer.py
"""
import base64
import json
import os
import sys
import time
import tkinter as tk

import cv2
import numpy as np

LIVE = os.path.expanduser(sys.argv[1] if len(sys.argv) > 1 else '~/.ros/cosmos_runs/live')
BG, FG, DIM, ACCENT, WARN = '#15171c', '#e6e6e6', '#9aa0aa', '#7fd17f', '#f0b45a'


def read(name, binary=False):
    try:
        with open(os.path.join(LIVE, name), 'rb' if binary else 'r') as f:
            return f.read()
    except OSError:
        return None


class Viewer:
    def __init__(self, root):
        self.root = root
        root.title('Cosmos 3 Edge - rover agent')
        root.configure(bg=BG)
        root.geometry('1180x760')
        mono = ('DejaVu Sans Mono', 10)
        self.header = tk.Label(root, text='waiting for the model...', font=('DejaVu Sans', 13, 'bold'),
                               bg=BG, fg=FG, anchor='w', padx=10, pady=6)
        self.header.pack(fill='x')
        top = tk.Frame(root, bg=BG)
        top.pack(fill='x', padx=10)
        self.image = tk.Label(top, bg='#000', width=512, height=320)
        self.image.pack(side='left')
        self.prompt = tk.Text(top, height=20, wrap='word', font=mono, bg='#1c1f26', fg=DIM, relief='flat',
                              padx=8, pady=6)
        self.prompt.pack(side='left', fill='both', expand=True, padx=(10, 0))
        tk.Label(root, text='Reasoning (live)', font=('DejaVu Sans', 10, 'bold'), bg=BG, fg=DIM,
                 anchor='w', padx=10).pack(fill='x', pady=(8, 0))
        self.output = tk.Text(root, wrap='word', font=mono, bg='#1c1f26', fg=FG, relief='flat', padx=8, pady=6)
        self.output.tag_configure('action', foreground=ACCENT, font=('DejaVu Sans Mono', 12, 'bold'))
        self.output.pack(fill='both', expand=True, padx=10, pady=(2, 10))
        self.stamp = {}
        self.photo = None
        self.poll()

    def changed(self, name):
        try:
            m = os.stat(os.path.join(LIVE, name)).st_mtime_ns
        except OSError:
            return False
        if self.stamp.get(name) == m:
            return False
        self.stamp[name] = m
        return True

    def poll(self):
        if self.changed('image.jpg'):
            data = read('image.jpg', True)
            img = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR) if data else None
            if img is not None:
                ok, png = cv2.imencode('.png', img)
                self.photo = tk.PhotoImage(data=base64.b64encode(png.tobytes()).decode())
                self.image.configure(image=self.photo, width=img.shape[1], height=img.shape[0])
        if self.changed('prompt.txt'):
            self.set_text(self.prompt, read('prompt.txt') or '')
        if self.changed('output.txt'):
            text = read('output.txt') or ''
            reasoning, _, answer = text.partition('</think>')
            self.output.delete('1.0', 'end')
            self.output.insert('end', reasoning.replace('<think>', '').strip())
            if answer.strip():
                self.output.insert('end', '\n\n' + answer.strip(), 'action')
            self.output.see('end')
        state = read('state.json')
        try:
            s = json.loads(state) if state else None
        except ValueError:
            s = None
        if s:
            n = s.get('request', 0)
            if s['state'] == 'thinking':
                self.header.configure(text=f'Request {n}: thinking...  {time.time() - s["started"]:.0f} s', fg=WARN)
            elif s['state'] == 'done':
                self.header.configure(text=f'Request {n}: {s.get("answer", "")}   '
                                           f'({s.get("seconds", 0):.1f} s, {s.get("tokens", 0)} tokens) '
                                           '- rover executing', fg=ACCENT)
            else:
                self.header.configure(text=f'Request {n}: model error {s.get("answer", "")}', fg='#ff6b6b')
        self.root.after(200, self.poll)

    @staticmethod
    def set_text(widget, text):
        widget.delete('1.0', 'end')
        widget.insert('end', text)


def main():
    root = tk.Tk()
    Viewer(root)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()

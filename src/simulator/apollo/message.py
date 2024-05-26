from src.common.types import *


class Message:
    def __init__(self, uid: str, action: str, frame: int, status=True, body=None):
        self.uid = uid
        self.action = action
        self.frame = frame
        self.status = 'success' if status is True else 'fail' if status is False else status
        self.body = body

    @classmethod
    def from_dict(cls, data: dict):
        header = data['header']
        uid = header['uid']
        action = header['action']
        frame = header['frame']
        status = header['status']
        body = None if len(data['body']) == 0 else data['body']
        return cls(uid, action, frame, status, body)

    def to_dict(self):
        data = {
            'header': {
                'uid': self.uid,
                'action': self.action,
                'frame': self.frame,
                'status': self.status
            },
            'body': {} if self.body is None else self.body
        }
        return data

    @property
    def is_success(self):
        return self.status == 'success'

    def __getitem__(self, key):
        if self.body is None:
            raise KeyError('Body is empty')
        if key not in self.body:
            raise KeyError(f'Key {key} not found in body')
        return self.body[key]


class Url:
    @classmethod
    def websocket_url(cls, host, port, post=''):
        return f'ws://{host}:{port}{"/" if len(post) > 0 else ""}{post}'

from multimethod import multidispatch


class Message:
    @multidispatch
    def __init__(self, uid: str, action: str, frame: int, status=True, body=None):
        self.uid = uid
        self.action = action
        self.frame = frame
        self.status = 'success' if status is True else 'fail' if status is False else status
        if body is not None:
            self.body: dict = body
        else:
            self.body = {}

    @__init__.register
    def __init__dict(self, data: dict):
        header = data['header']
        self.uid = header['uid']
        self.action = header['action']
        self.frame = header['frame']
        self.status = header['status']
        self.body = {} if len(data['body']) == 0 else data['body']

    def to_dict(self):
        data = {
            'header': {
                'uid': self.uid,
                'action': self.action,
                'frame': self.frame,
                'status': self.status
            },
            'body': self.body
        }
        return data

    @property
    def is_success(self):
        return self.status == 'success'

    def __getitem__(self, key):
        return self.body[key]


class Url:
    @classmethod
    def websocket_url(cls, host, port, post=''):
        return f'ws://{host}:{port}{"/" if len(post) > 0 else ""}{post}'

import unittest
from common.algorithm.sequence_solver import SequenceSolver, Symbol


class TestProto(unittest.TestCase):
    def test_hard_constraint(self):
        a = Symbol('a')
        b = Symbol('b')
        c = Symbol('c')
        d = Symbol('d')
        option_list = [
            {a, b},
            {b, c},
            {a, d},
        ]
        neighbors = {
            a: {b: 10, c: 1},
            b: {b: 1, d: 1}
        }


if __name__ == '__main__':
    unittest.main()

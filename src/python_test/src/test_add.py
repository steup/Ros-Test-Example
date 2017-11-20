#!/usr/bin/env python2

from add import Functor
import random
import unittest

class Test_Add(unittest.TestCase):

    def test_add_100(self):
        random.seed(1337)
        i = 100
        while i != 0:
            i=i-1
            a = random.randrange(100)
            b = random.randrange(100)
            f = Functor()
            c = f(a,b)
            self.assertEqual(c,a+b, "%i+%i != %i"%(a,b,c))
            self.assertEqual(type(c), type(a), "type mismatch %s != %s"%(type(c), type(a)))

if __name__ == '__main__':
    import rosunit
    rosunit.unitrun("python_test", 'test_add', Test_Add)

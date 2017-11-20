#!/usr/bin/env python2

class Functor(object):
    def __call__(self, a, b):
        return a+b

    def __repr__(self):
        return "Functor: a+b"

#!/bin/sh

g++ main.cpp -std=c++11 -I sdl2/2.0.8/include/SDL2 -lsdl2 -lpng -g -o raytracer

# TODO
# gcc -fobjc-arc -framework Cocoa -x objective-c -o MicroApp main.m

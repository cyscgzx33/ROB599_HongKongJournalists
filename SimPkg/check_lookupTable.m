clear all;clc;close all;
load('TestTrack');
leftb = TestTrack.bl;
rightb = TestTrack.br;
interp1(leftb(1,:),leftb(2,:),870);
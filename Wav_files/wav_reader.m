% wav file
clc
clear all

[y,Fs] = audioread('file_example_WAV_10MG.wav', 'native');
whos y

[yp,Fs] = audioread('file_example_WAV_10MG.wav');
sound(yp,Fs);
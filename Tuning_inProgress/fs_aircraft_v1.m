%%
close all; clear; clc;
saturationLims = [ -25, 25;
    -25, 10;
    -30, 30;];
saturationLims = saturationLims.*pi ./ 180;
saturationLims = cat(1, saturationLims, [0.5, 10; 0.5, 10]);
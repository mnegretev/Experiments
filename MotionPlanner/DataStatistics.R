setwd("~/Experiments/MotionPlanner");
rm(list = ls());
DataJustina <- read.csv("results_nagoya_justina2.mpd")
DataNav2d <- read.csv("results_nagoya_nav2d.mpd")
distances <-stack(cbind(DataJustina$Distance, DataNav2d$Distance))
# t.test(value~ind, data = distances)

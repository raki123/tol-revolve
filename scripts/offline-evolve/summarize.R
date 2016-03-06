# Use this script to summarize one or multiple concatenated
# runs of evolution data.
# This post helps here:
# http://stackoverflow.com/a/10349375/358873

library(ggplot2)
library(plyr)
library(reshape)

data0 = read.csv("baseline/generations.csv", head=TRUE)
data0$param_set = "No evolution"
data1 = read.csv("plus/generations.csv", head=TRUE)
data1$param_set = "First param set"
data2 = read.csv("plus-aggressive/generations.csv", head=TRUE)
data2$param_set = "Second param set"
data = rbind(data0, data1, data2)
data$param_set = as.factor(data$param_set)
cdata = ddply(data, c("gen", "param_set"), summarise, Fitness=mean(fitness), Velocity=mean(vel), fsd=sd(fitness), vsd=sd(vel))
#mdata = melt(cdata, id=c("gen", "fsd", "vsd"))

ggplot(cdata, aes(gen)) +
  geom_line(aes(y=Fitness, colour=param_set)) +
  geom_ribbon(aes(ymin=Fitness-fsd, ymax=Fitness+fsd, fill=param_set), alpha=0.2, linetype=0) +
  scale_fill_discrete(name="Experiment") +
  scale_colour_discrete(name="Experiment") +
  xlab("Generation") +
  ylab("Fitness")

lastgen1 = data1[data1$gen==max(data1$gen),];
fit_mean1 = mean(lastgen1$fitness);
fit_sd1 = sd(lastgen1$fitness);

lastgen2 = data2[data2$gen==max(data2$gen),];
fit_mean2 = mean(lastgen2$fitness);
fit_sd2 = sd(lastgen2$fitness);
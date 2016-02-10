library(ggplot2)
library(plyr)
library(reshape)

fitness = read.csv("fitness.csv", head=TRUE);
cfitness = ddply(fitness, c("t_sim", "run"), summarise, n=length(fitness), fit=mean(fitness), vel=mean(vel), fsd=sd(fitness), vsd=sd(vel))
cfitness$run = as.factor(cfitness$run);

fcolor <- "#00A6DE";

ggplot(cfitness, aes(t_sim)) +
  geom_line(aes(y=fit), colour=fcolor) +
  ylab("Fitness") + xlab("Simulation time (s)") +
  geom_ribbon(data=cfitness, aes(ymin=fit-fsd, ymax=fit+fsd), alpha=0.2, fill=fcolor, linetype=0);

ggplot(cfitness, aes(x=t_sim)) +
  xlab("Time (s)") + ylab("# of individuals") +
  geom_line(aes(y=n, colour=run));

robots = read.csv("robots.csv", head=TRUE);

max_fit = fitness[fitness$fitness==max(fitness$fitness),]
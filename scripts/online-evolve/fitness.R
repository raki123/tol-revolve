library(ggplot2)
library(plyr)
library(reshape)

fitness = read.csv("fitness.csv", head=TRUE);
cfitness = ddply(fitness, c("t_sim", "run"), summarise, n=length(fitness), fit=mean(fitness), vel=mean(vel), fsd=sd(fitness), vsd=sd(vel))
cfitness$run = as.factor(cfitness$run);

fcolor <- "#00A6DE";

ggplot(cfitness, aes(t_sim)) +
  geom_ribbon(aes(ymin=fit-fsd, ymax=fit+fsd, colour=run, fill=run), linetype=0, alpha=0.2) +
  geom_line(aes(y=fit, colour=run)) +
  ylab("Fitness") + xlab("Simulation time (s)");
# geom_ribbon(data=cfitness, aes(ymin=fit-fsd, ymax=fit+fsd), alpha=0.2, fill=fcolor, linetype=0);

ggplot(cfitness, aes(x=t_sim)) +
  xlab("Time (s)") + ylab("# of individuals") +
  geom_line(aes(y=n, colour=run));

robots = read.csv("robots.csv", head=TRUE);

max_fit = fitness[order(-fitness$fitness),]

wsum = read.csv("summary.csv", head=TRUE);
wsum$run = as.factor(wsum$run);
ggplot(wsum, aes(world_age)) +
  xlab("Time (s)") + ylab("Main battery charge") +
  geom_line(aes(y=charge, colour=run));

ggplot(wsum, aes(world_age)) +
  xlab("Time (s)") + ylab("Part count") +
  geom_line(aes(y=part_count, colour=run));
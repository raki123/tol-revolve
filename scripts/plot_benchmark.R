library(ggplot2)
library(plyr)

bm = read.csv("/media/expdata/output/benchmark.csv", head=TRUE);
bmd = ddply(bm, c("step_size", "population_size"), summarise, fac=mean(factor), fsd=sd(factor));
bmd$step_size = as.factor(bmd$step_size);

ggplot(bmd, aes(population_size, fac)) + 
  geom_line(color="#4285f4") + 
  geom_ribbon(aes(ymin=fac-fsd, ymax=fac+fsd), alpha=0.1, linetype=0, fill="#4285f4") +
  xlab("Population size") +
  ylab("Real time factor (simulation time / real time)");

# Use this script to summarize one or multiple concatenated
# runs of evolution data.
# This post helps here:
# http://stackoverflow.com/a/10349375/358873

library(ggplot2)
library(plyr)
library(reshape)

read_dir_data <- function(odir) {
  tdata = read.csv(paste(odir, "/generations.csv", sep=""), head=TRUE);
  tdata$exp = as.factor(odir);
  return(tdata);
}

dirs = list.files(".");
data = do.call(rbind, lapply(dirs, read_dir_data));

cdata = ddply(data, c("gen", "exp"), summarise, Fitness=mean(fitness), Velocity=mean(vel), fsd=sd(fitness), vsd=sd(vel))
#mdata = melt(cdata, id=c("gen", "fsd", "vsd"))

ggplot(cdata, aes(gen)) +
  geom_line(aes(y=Fitness, colour=exp)) +
  geom_ribbon(aes(ymin=Fitness-fsd, ymax=Fitness+fsd, fill=exp), alpha=0.1, linetype=0) +
  scale_fill_discrete(name="Experiment") +
  scale_colour_discrete(name="Experiment") +
  xlab("Generation") +
  ylab("Fitness")

maxes = cdata[cdata$gen==max(cdata$gen),];

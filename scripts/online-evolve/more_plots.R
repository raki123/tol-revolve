library(plyr);
library(ggplot2);

# Read all needed data
fitness = read.csv("fitness.csv", head=TRUE);
fitness$run = as.factor(fitness$run);

wsum = read.csv("summary.csv", head=TRUE);
wsum = rename(wsum, c("world_age"="t_sim"));

# Add the number of births at any given time as a variable
#fitness$births = 0;
#nlines = nrow(wsum);
#for (i in 1:nlines) {
#  crow = wsum[i,];
#  sel = which(fitness$run == crow$run & fitness$t_sim == crow$t_sim);
#  if (length(sel) > 0) {
#    fitness[sel,]$births = crow$births;
#  }
#}

# Filter outliers...
# TODO How to make this better?
#fitness = fitness[fitness$fitness<0.5,];
cfitness = ddply(fitness, c("births", "run"), summarise, fit=mean(fitness), fsd=sd(fitness))

wsum$run = as.factor(wsum$run);
wsum$ppr = wsum$part_count / wsum$robot_count;

# Dummy facetting (I don't understand this, yet)
# https://github.com/hadley/ggplot2/wiki/Align-two-plots-on-a-page
cfitness$panel = "Fitness";
wsum$panel = "# of individuals";
wsum1=wsum;
wsum1$panel = "# of robot parts";
wsum2 = wsum;
wsum2$panel = "Avg parts / robot"
#wsum3 = wsum;
#wsum3$panel = "Main battery charge";

ggplot(cfitness, aes(births)) + geom_line(aes(y=fit));

ggplot(data = cfitness, mapping = aes(births, colour=run, fill=run)) +
  facet_grid(panel~., scale="free") +
  geom_ribbon(data=cfitness, aes(ymin=fit-fsd, ymax=fit+fsd), linetype=0, alpha=0.2) +
  geom_line(data=cfitness, aes(y=fit)) +
  geom_line(data=wsum, aes(y=robot_count)) +
  geom_line(data=wsum1, aes(y=part_count)) +
#  geom_line(data=wsum2, aes(y=ppr)) +
#  geom_line(data=wsum3, aes(y=charge)) +
  scale_colour_discrete(name="Run") +
  scale_fill_discrete(guide=FALSE) +
  ylab("") + xlab("# of born individuals");


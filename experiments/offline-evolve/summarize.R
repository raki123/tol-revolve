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
  if (tdata[1,]$exp == "plus-gradual") {
    tdata$births = tdata$gen + 15;
  } else {
    tdata$births = tdata$gen * 15 + 15;
  }
  
  rdata = read.csv(paste(odir, "/robots.csv", sep=""), head=TRUE);
  
  return(merge(tdata[tdata$births<=3000,], rdata, by.x=c("robot_id", "run"), by.y=c("id", "run")));
}

dirs = list.files(".");
data = do.call(rbind, lapply(dirs, read_dir_data));

# Unify fitness calculation
data$fitness = 5*data$dvel + data$vel;

# Remove outliers
data = data[data$fitness<1,];

cdata = ddply(data, c("births", "exp"), summarise, 
              fit=mean(fitness), fsd=sd(fitness), 
              ext=mean(extremity_count), esd=sd(extremity_count),
              joints=mean(joint_count), jsd=sd(joint_count),
              motors=mean(motor_count), msd=sd(motor_count),
              sz=mean(nparts), ssd=sd(nparts),
              inp=mean(inputs), isd=sd(inputs),
              hid=mean(hidden), hsd=sd(hidden),
              con=mean(conn), csd=sd(conn)
              );
              
cdata$plot = as.factor("Fitness");
cdata1 = cdata;
cdata2 = cdata;
cdata3 = cdata;
cdata4 = cdata;
cdata5 = cdata;
cdata6 = cdata;
cdata7 = cdata;

cdata1$plot = as.factor("# of joints");
cdata2$plot = as.factor("# of active joints");
cdata3$plot = as.factor("Total size");
cdata4$plot = as.factor("# of extremities");
cdata5$plot = as.factor("# of inputs");
cdata6$plot = as.factor("# of hidden neurons");
cdata7$plot = as.factor("# of connections");

# Fitness over generations
ggplot(cdata, aes(births)) +
  facet_grid(plot~., scale="free") +
  geom_line(aes(y=fit, colour=exp)) +
  geom_ribbon(aes(ymin=fit-fsd, ymax=fit+fsd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata1, aes(y=joints, colour=exp)) +
  geom_ribbon(data=cdata1, aes(ymin=joints-jsd, ymax=joints+jsd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata2, aes(y=motors, colour=exp)) +
  geom_ribbon(data=cdata2, aes(ymin=motors-jsd, ymax=motors+jsd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata3, aes(y=sz, colour=exp)) +
  geom_ribbon(data=cdata3, aes(ymin=sz-ssd, ymax=sz+ssd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata4, aes(y=ext, colour=exp)) +
  geom_ribbon(data=cdata4, aes(ymin=ext-esd, ymax=ext+esd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata5, aes(y=inp, colour=exp)) +
  geom_ribbon(data=cdata5, aes(ymin=inp-isd, ymax=inp+isd, fill=exp), alpha=0.1, linetype=0) +
  
  geom_line(data=cdata6, aes(y=hid, colour=exp)) +
  geom_ribbon(data=cdata6, aes(ymin=hid-hsd, ymax=hid+hsd, fill=exp), alpha=0.1, linetype=0) +

  geom_line(data=cdata7, aes(y=con, colour=exp)) +
  geom_ribbon(data=cdata7, aes(ymin=con-csd, ymax=con+csd, fill=exp), alpha=0.1, linetype=0) +
  
  scale_fill_discrete(name="Experiment") +
  scale_colour_discrete(name="Experiment") +
  xlab("# of births") +
  ylab("");

#last_gens = cdata[cdata$gen==max(cdata$gen),];

# Retrieve all the best robots per experiment
# See http://stackoverflow.com/questions/6289538/aggregate-a-dataframe-on-a-given-column-and-display-another-column
exp_maxes = do.call(rbind,lapply(split(data, list(data$exp, data$run)),function(chunk) chunk[which.max(chunk$fitness),]))
exp_maxes = exp_maxes[order(-exp_maxes$fitness),]

exp_states = ddply(data, ~exp+run, summarise, cur_gen=max(gen));
exp_states = exp_states[order(exp_states$exp, -exp_states$run),];

exp_states

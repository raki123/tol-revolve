# Use this script to summarize one or multiple concatenated
# runs of evolution data.
library(ggplot2)
library(plyr)

data = read.csv("generations.csv", head=TRUE)
cdata = ddply(data, c("gen"), summarise, mean=mean(vel), sd=sd(vel))

linecolor <- "#00A6DE"
ggplot(cdata, aes(x=gen, y=mean)) +
 geom_ribbon(aes(ymin=mean-sd, ymax=mean+sd), alpha=0.2, fill=linecolor, linetype=0) +
 geom_line(colour=linecolor) +
 geom_point(colour=linecolor) +
 xlab("Generation") +
 ylab("Velocity (m/s)")

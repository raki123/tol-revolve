library(ggplot2)
library(plyr)

bm = read.csv("/media/expdata/output/benchmark.csv", head=TRUE);
bmd = ddply(bm, c("step_size", "population_size"), summarise, fac=mean(factor), fsd=sd(factor));
bmd$step_size = as.factor(bmd$step_size);
bmd$population_size_m2 = bmd$population_size^(-2);
bmd$population_size_1 = 1/bmd$population_size;

fit = lm(fac ~ population_size_1 + population_size_m2, data=bmd);

benchmark_fit_func <- function(v, fit) {
  cf = coefficients(fit);
  return(cf[1] + cf[2] * 1/v + cf[3] * v^-2);
}

maxv = max(bmd$population_size);
fit_plot = data.frame(x=5:maxv, y=benchmark_fit_func(5:maxv, fit));

levels = c("Fitted", "Measured");
level_measured = factor("Measured", levels=levels);
level_fitted = factor("Fitted", levels=levels);

ggplot(bmd, aes(population_size, fac)) + 
  geom_line(aes(color=level_measured)) + 
  geom_line(data=fit_plot, aes(x, y, color=level_fitted)) +
  geom_ribbon(aes(ymin=fac-fsd, ymax=fac+fsd, fill=level_measured), alpha=0.1, linetype=0) +
  
  # Fake ribbon to fix the legend
  geom_ribbon(aes(ymin=fac, ymax=fac, fill=level_fitted), alpha=0.0, linetype=0) +
  
  # y=1.0 line
  geom_hline(yintercept=1) +
  annotate("text", label="RTF = 1.0", y=1.2, x=10.0) +
  
  xlab("Population size") +
  ylab("Real time factor (simulation time / real time)")+
  scale_fill_discrete(name="Line") +
  scale_color_discrete(name="Line");
ggsave("/home/elte/mt/Thesis/plots/benchmark.pdf");
summary(fit);


#This scrip shows the results of data analysis previous to modeling.

# Errors as histograms  ------------------------------------------------------------------

rm(list = ls())

obs_005 <- read.csv('Observations_005.csv')
obs_05 <- read.csv('Observations_05.csv')
obs_10 <- read.csv('Observations_10.csv')
obs_20 <- read.csv('Observations_20.csv')

resp_005 <- read.csv('Responses_005.csv')
resp_05 <- read.csv('Responses_05.csv')
resp_10 <- read.csv('Responses_10.csv')
resp_20 <- read.csv('Responses_20.csv')

real_005 <- read.csv('Graund_005.csv')
real_05 <- read.csv('Graund_05.csv')
real_10 <- read.csv('Graund_10.csv')
real_20 <- read.csv('Graund_20.csv')

n_conditions = 4
n_participants = length(resp_005[1,])
gris = rgb(128, 128, 128, maxColorValue = 255)
gris2 = rgb(192, 192, 192, maxColorValue = 255, alpha = 100)
full_pe<-array(NA, dim = c(n_conditions, length(obs_005[, 1]) * length(obs_005[1,])))
full_errors_real <-array(NA, dim = c(n_conditions, length(obs_005[, 1]) * length(obs_005[1,])))

pe_errors <- array(NA, dim = c(length(obs_005[, 1]) * length(obs_005[1,])))
pe_errors2 <- array(NA, dim = c(length(obs_005[, 1])))
errors_real <-array(NA, dim = c(length(obs_005[, 1]) * length(obs_005[1,])))
errors_real2 <-array(NA, dim = c(length(obs_005[, 1])))

total_errors<-array(NA, dim = c(n_conditions, n_participants, length(obs_005[, 1])))
total_pe <-array(NA, dim = c(n_conditions, n_participants, length(obs_005[, 1])))


for (j in 1:n_conditions) {
  if (j == 1) {
    obs = obs_005
    resp = resp_005
    real = real_005
  } else if (j == 2) {
    obs = obs_05
    resp = resp_05
    real = real_05
  } else if (j == 3) {
    obs = obs_10
    resp = resp_10
    real = real_10
  } else {
    obs = obs_20
    resp = resp_20
    real = real_20
  }
  
  t = 0
  
  for (h in 1:n_participants) {
    
    for (i in 2:length(obs_005[, 1])) {
      t = t + 1
      
      pe_errors[t] = obs[i, h] - resp[i, h]
      errors_real[t] = real[i, h] - resp[i, h]
      
      pe_errors2[i] = obs[i, h] - resp[i, h]
      errors_real2[i] = real[i, h] - resp[i, h]
      
    }
    
    total_errors[j, h,] <- errors_real2
    total_pe[j, h,] <- pe_errors2

  }
  
  full_pe[j,] <- pe_errors
  full_errors_real[j,] <- errors_real
  
}

#save(total_errors,file="total_errors.Rdata")

## HIstograms

mean_sd <- array(NA, dim = c(4, 4))

mean_sd[1, 1] <- round(mean(full_pe[1,], na.rm = TRUE), 2)
mean_sd[1, 2] <- round(mean(full_pe[2,], na.rm = TRUE), 2)
mean_sd[1, 3] <- round(mean(full_pe[3,], na.rm = TRUE), 2)
mean_sd[1, 4] <- round(mean(full_pe[4,], na.rm = TRUE), 2)

mean_sd[2, 1] <- round(mean(full_errors_real[1,], na.rm = TRUE), 2)
mean_sd[2, 2] <- round(mean(full_errors_real[2,], na.rm = TRUE), 2)
mean_sd[2, 3] <- round(mean(full_errors_real[3,], na.rm = TRUE), 2)
mean_sd[2, 4] <- round(mean(full_errors_real[4,], na.rm = TRUE), 2)

mean_sd[3, 1] <- round(sd(full_pe[1,], na.rm = TRUE), 2)
mean_sd[3, 2] <- round(sd(full_pe[2,], na.rm = TRUE), 2)
mean_sd[3, 3] <- round(sd(full_pe[3,], na.rm = TRUE), 2)
mean_sd[3, 4] <- round(sd(full_pe[4,], na.rm = TRUE), 2)

mean_sd[4, 1] <- round(sd(full_errors_real[1,], na.rm = TRUE), 2)
mean_sd[4, 2] <- round(sd(full_errors_real[2,], na.rm = TRUE), 2)
mean_sd[4, 3] <- round(sd(full_errors_real[3,], na.rm = TRUE), 2)
mean_sd[4, 4] <- round(sd(full_errors_real[4,], na.rm = TRUE), 2)



library('plotrix')
library('readbitmap')
library('png')

pdf('Errors.pdf',
     width = 11.5,
     height = 6)

layout(matrix(c(seq(1,8)), 2, 4,byrow = T))
par(mai = c(0.1, 0.7, 0.7, 0))

a = readPNG('ufo1.png')

plot(
  0,
  0,
  col = 'white',
  xlim = c(-12, 12),
  ylim = c(-12, 12),
  axes = FALSE,
  ylab = '',
  xlab = ''
)
draw.circle(0, 0, 8, lty = 5, border = 'blue')
#2 sd
draw.arc(
  0,
  0,
  8,
  0,
  2.16,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,   maxColor = 255,
    alpha = 30
  ),
  lwd = 12
)

#1 sd
draw.arc(
  0,
  0,
  8,
  0.54,
  1.62,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,    maxColor = 255,
    alpha = 100
  ),
  lwd = 12
)

rasterImage(a, 2.3, 6.3, 5.8, 7.7)



plot(
  0,
  0,
  col = 'white',
  xlim = c(-12, 12),
  ylim = c(-12, 12),
  axes = FALSE,
  ylab = '',
  xlab = ''
)
draw.circle(0, 0, 8, lty = 5, border = 'blue')
draw.arc(
  0,
  0,
  8,
  0,
  0.96,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,    maxColor = 255,
    alpha = 30
  ),
  lwd = 12
)
draw.arc(
  0,
  0,
  8,
  0.24,
  0.72,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,   maxColor = 255,
    alpha = 100
  ),
  lwd = 12
)

rasterImage(a, 5.2, 2.9, 8.7, 4.2)

plot(
  0,
  0,
  col = 'white',
  xlim = c(-12, 12),
  ylim = c(-12, 12),
  axes = FALSE,
  ylab = '',
  xlab = ''
)
draw.circle(0, 0, 8, lty = 5, border = 'blue')
draw.arc(
  0,
  0,
  8,
  0,
  0.84,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,    maxColor = 255,
    alpha = 30
  ),
  lwd = 12
)
draw.arc(
  0,
  0,
  8,
  0.21,
  0.63,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,
    maxColor = 255,
    alpha = 100
  ),
  lwd = 12
)


rasterImage(a, 5.7, 2.6, 9.3, 3.9)

plot(
  0,
  0,
  col = 'white',
  xlim = c(-12, 12),
  ylim = c(-12, 12),
  axes = FALSE,
  ylab = '',
  xlab = ''
)
draw.circle(0, 0, 8, lty = 5, border = 'blue')
draw.arc(
  0,
  0,
  8,
  0,
  0.76,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,
    maxColor = 255,
    alpha = 30
  ),
  lwd = 12
)
draw.arc(
  0,
  0,
  8,
  0.19,
  0.57,
  col = rgb(
    0,
    0.4470 * 255,
    0.7410 * 255,
    maxColor = 255,
    alpha = 100
  ),
  lwd = 12
)

rasterImage(a, 5.9, 2.3, 9.5, 3.6)

par(mai = c(0.6, 0.7, 0, 0.1))

breaks=seq(from=-4,to=4,by=0.1)
h1 <- hist(total_errors[1,,],plot=FALSE,breaks = breaks)
xvals <- h1$breaks
yvals <- h1$counts/sum(h1$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.25),axes=F,xlab="",ylab="")
h=hist(total_errors[1,,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[1,,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=as.character(c(0,0.05,0.1,0.15,0.20,0.25)),at=c(0,0.05,0.1,0.15,0.20,0.25)
)
text(0.3,0.25,"S/N = 0.05", cex=1.6)
legend(-0.6,0.2,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 1]))
           )))
       ),
       bty = 'n',
       cex = 2)

par(mai = c(0.6, 0.5, 0, 0.3))

h2 <- hist(total_errors[2,,],plot=FALSE,breaks = breaks)
xvals <- h2$breaks
yvals <- h2$counts/sum(h2$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.25),axes=F,xlab="",ylab="")
h=hist(total_errors[2,,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[2,,])) 
plot(h,add=T)


axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=as.character(c("","","","","","")),at=c(0,0.05,0.1,0.15,0.20,0.25)
)
text(0.2,0.25,"S/N = 0.5", cex=1.6)
legend(-0.3,0.2,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 2]))
           )))
       ),
       bty = 'n',
       cex = 2)
par(mai = c(0.6, 0.5, 0, 0.3))

h3<- hist(total_errors[3,,],plot=FALSE,breaks = breaks)
xvals <- h3$breaks
yvals <- h3$counts/sum(h3$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.25),axes=F,xlab="",ylab="")
h=hist(total_errors[3,,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[3,,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=as.character(c("","","","","","")),at=c(0,0.05,0.1,0.15,0.20,0.25)
)
text(0.2,0.25,"S/N = 1", cex=1.6)
legend(-0.3,0.2,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 3]))
           )))
       ),
       bty = 'n',
       cex = 2)

par(mai = c(0.6, 0.5, 0, 0.3))
h4<- hist(total_errors[4,,],plot=FALSE,breaks = breaks)
xvals <- h4$breaks
yvals <- h4$counts/sum(h4$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.25),axes=F,xlab="",ylab="")
h=hist(total_errors[4,,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[4,,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=as.character(c("","","","","","")),at=c(0,0.05,0.1,0.15,0.20,0.25)
)
text(0,0.25,"S/N = 2", cex=1.6)

legend(-0.3,0.2,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 4]))
           )))
       ),
       bty = 'n',
       cex = 2)

mtext("Proportion",side=2,line=-1.6, cex=1.2,outer = T,adj = 0.25)
mtext("Error (generative mean - prediction)",
      side=1,line=-2, cex=1.2,outer = T)


dev.off()



# Example sequence --------------------------------------------------------


rm(list = ls())

obs_005 <- read.csv('Observations_005.csv')
obs_05 <- read.csv('Observations_05.csv')
obs_10 <- read.csv('Observations_10.csv')
obs_20 <- read.csv('Observations_20.csv')

resp_005 <- read.csv('Responses_005.csv')
resp_05 <- read.csv('Responses_05.csv')
resp_10 <- read.csv('Responses_10.csv')
resp_20 <- read.csv('Responses_20.csv')

real_005 <- read.csv('Graund_005.csv')
real_05 <- read.csv('Graund_05.csv')
real_10 <- read.csv('Graund_10.csv')
real_20 <- read.csv('Graund_20.csv')


azul = rgb(0, 0.4470, 0.7410, maxColorValue = 1, alpha = 1)
naranja = rgb(0.8500,0.3250,0.0980,maxColorValue = 1,alpha = 1)
amarillo = rgb(0.9290, 0.6940, 0.1250, maxColorValue = 1)
verde = rgb(0.4660,0.6740,0.1880,maxColorValue = 1,alpha = 1)
rojo= rgb(255, 40, 40, maxColorValue = 255,alpha = 255)
negro = rgb(0, 0, 0, maxColorValue = 255)
gris = rgb(128, 128, 128, maxColorValue = 255)

conditions <- 4
length_data <- 300
n_participants <- 72
ratio <- c(0.05,0.5,1,2)

#p=sample(c(1:72),1,replace = T)
pdf('Sequence.pdf',
    width = 11.5,
    height = 5.5)

layout(matrix(c(1:4),2,2,byrow = T))
par(fig = c(0,1,0,1),mar=c(2,5,3,1),mfrow=c(2,2))

p=24

pe_1=as.matrix(real_005[2:300,p])-as.matrix(resp_005[2:300,p])
pe_2=as.matrix(real_05[2:300,p])-as.matrix(resp_05[2:300,p])
pe_3=as.matrix(real_10[2:300,p])-as.matrix(resp_10[2:300,p])
pe_4=as.matrix(real_20[2:300,p])-as.matrix(resp_20[2:300,p])

lwd=3
lwd_p=2
cex_p=0.8
plot(1:(length(obs_005[2:101,p])),obs_005[2:101,p],
     xlab="",ylab="",type="p",axes=F,xaxt="n",yaxt="n",cex=cex_p,col=rojo,bg=rojo,pch=21)
lines(1:(length(real_005[2:101,p])),real_005[2:101,p],lwd=lwd_p,col="blue")
lines(1:(length(resp_005[2:101,p])),resp_005[2:101,p],lwd=lwd_p)
axis(2,labels=as.character(seq(from=0,to=15,by=3)),las=2,at=seq(from=0,to=15,by=3))
axis(1,labels=F,at=seq(from=0,to=100,by=10))
mtext("S/N = 0.05",side=3,line=0.4)
box(lwd=1.5)

par(mar=c(2,3,3,3))

plot(1:(length(obs_05[2:101,p])),obs_05[2:101,p],
     xlab="",ylab="",type="p",axes=F,xaxt="n",yaxt="n",cex=cex_p,col=rojo,bg=rojo,pch=21)
lines(1:(length(real_05[2:101,p])),real_05[2:101,p],lwd=lwd_p,col="blue")
lines(1:(length(resp_05[2:101,p])),resp_05[2:101,p],lwd=lwd_p) 
axis(2,labels=as.character(seq(from=0,to=-12,by=-3)),las=2,at=seq(from=0,to=-12,by=-3))
axis(1,labels=F,at=seq(from=0,to=100,by=10))
mtext("S/N = 0.5",side=3,line=0.4)
box(lwd=1.5)

par(mar=c(4,5,1,1))

plot(1:(length(obs_10[2:101,p])),obs_10[2:101,p],
     xlab="",ylab="",type="p",axes=F,xaxt="n",yaxt="n",cex=cex_p,col=rojo,bg=rojo,pch=21)
lines(1:(length(real_10[2:101,p])),real_10[2:101,p],lwd=lwd_p,col="blue")
lines(1:(length(resp_10[2:101,p])),resp_10[2:101,p],lwd=lwd_p)
axis(2,labels=as.character(seq(from=0,to=15,by=3)),las=2,at=seq(from=0,to=15,by=3))
axis(1,labels=as.character(seq(from=0,to=100,by=10)),at=seq(from=0,to=100,by=10))
legend(60,6,legend=c("Generative mean","Responses","Observations"),
       lty=c(1,1,NA),pch=c(NA,NA,21),pt.bg="red",
       col=c("blue","black",rojo),lwd=2,bty="n",
       y.intersp = 0.8)
mtext("S/N = 1",side=3,line=0.4)
box(lwd=1.5)

par(mar=c(4,3,1,3))

plot(1:(length(obs_20[2:101,p])),obs_20[2:101,p],
     xlab="",ylab="",type="p",axes=F,xaxt="n",yaxt="n",cex=cex_p,col=rojo,bg=rojo,pch=21)
lines(1:(length(real_20[2:101,p])),real_20[2:101,p],lwd=lwd_p,col="blue")
lines(1:(length(resp_20[2:101,p])),resp_20[2:101,p],lwd=lwd_p)
axis(2,labels=as.character(seq(from=0,to=-24,by=-4)),las=2,at=seq(from=0,to=-24,by=-4))
axis(1,labels=as.character(seq(from=0,to=100,by=10)),at=seq(from=0,to=100,by=10))
mtext("S/N = 2",side=3,line=0.4)
box(lwd=1.5)

### Insets
par(fig = c(0.08,0.28, 0.65, 0.95), new = T,mar=c(3,3,3,3),mgp=c(3,0.5,0)) 
hist(pe_1,axes=F,breaks=seq(-6,6,0.1),main="",
     ylab="",xlab="",
     col=gris,border=gris,xlim=c(-2,2),ylim=c(0,80))
axis(1,cex.axis=0.6,padj = -0.8)
axis(2,cex.axis=0.6,las=2,labels=as.character(seq(from=0,to=80,by=20)),
     at=seq(from=0,to=80,by=20))
mtext("Error",side=1,cex=0.7,line=1)
mtext("Frequency",side=2,cex=0.7,line=1.2)

par(fig = c(0.75,0.95, 0.65, 0.95), new = T,mar=c(3,3,3,3),mgp=c(3,0.5,0)) 
hist(pe_2,axes=F,breaks=seq(-6,6,0.1),main="",ylab="",xlab="Prediction error",
     col=gris,border=gris,xlim=c(-2,2),ylim=c(0,80))
axis(1,cex.axis=0.6,padj = -0.8)
axis(2,cex.axis=0.6,las=2,labels=as.character(seq(from=0,to=80,by=20)),
     at=seq(from=0,to=80,by=20))
mtext("Error",side=1,cex=0.7,line=1)
mtext("Frequency",side=2,cex=0.7,line=1)

par(fig = c(0.08,0.28, 0.2, 0.5), new = T,mar=c(3,3,3,3),mgp=c(3,0.5,0)) 
hist(pe_3,axes=F,breaks=seq(-6,6,0.1),main="",ylab="",xlab="Prediction error",
     col=gris,border=gris,xlim=c(-2,2),ylim=c(0,80))
axis(1,cex.axis=0.6,padj = -0.8)
axis(2,cex.axis=0.6,las=2,labels=as.character(seq(from=0,to=80,by=20)),
     at=seq(from=0,to=80,by=20))
mtext("Error",side=1,cex=0.7,line=1)
mtext("Frequency",side=2,cex=0.7,line=1.2)

par(fig = c(0.55,0.75, 0.13, 0.43), new = T,mar=c(3,3,3,3),mgp=c(3,0.5,0)) 
hist(pe_4,axes=F,breaks=seq(-6,6,0.1),main="",ylab="",xlab="Prediction error",
     col=gris,border=gris,xlim=c(-2,2),ylim=c(0,80))
axis(1,cex.axis=0.6,padj = -0.8)
axis(2,cex.axis=0.6,las=2,labels=as.character(seq(from=0,to=80,by=20)),
     at=seq(from=0,to=80,by=20))
mtext("Error",side=1,cex=0.7,line=1)
mtext("Frequency",side=2,cex=0.7,line=1.2)

mtext("Position",side=2,cex=1.2,line=-2,outer = T)
mtext("Trials",side=1,cex=1.2,line=-1,outer = T,adj=0.51)

dev.off()


# Individual errors (Suplementary) -------------------------------------------------------

rm(list = ls())

obs_005 <- read.csv('Observations_005.csv')
obs_05 <- read.csv('Observations_05.csv')
obs_10 <- read.csv('Observations_10.csv')
obs_20 <- read.csv('Observations_20.csv')

resp_005 <- read.csv('Responses_005.csv')
resp_05 <- read.csv('Responses_05.csv')
resp_10 <- read.csv('Responses_10.csv')
resp_20 <- read.csv('Responses_20.csv')

real_005 <- read.csv('Graund_005.csv')
real_05 <- read.csv('Graund_05.csv')
real_10 <- read.csv('Graund_10.csv')
real_20 <- read.csv('Graund_20.csv')

n_conditions = 4
n_participants = length(resp_005[1,])
gris = rgb(128, 128, 128, maxColorValue = 255)
gris2 = rgb(192, 192, 192, maxColorValue = 255, alpha = 100)
full_pe<-array(NA, dim = c(n_conditions, length(obs_005[, 1]) * length(obs_005[1,])))
full_errors_real <-array(NA, dim = c(n_conditions, length(obs_005[, 1]) * length(obs_005[1,])))

pe_errors <- array(NA, dim = c(length(obs_005[, 1]) * length(obs_005[1,])))
pe_errors2 <- array(NA, dim = c(length(obs_005[, 1])))
errors_real <-array(NA, dim = c(length(obs_005[, 1]) * length(obs_005[1,])))
errors_real2 <-array(NA, dim = c(length(obs_005[, 1])))

total_errors<-array(NA, dim = c(n_conditions, n_participants, length(obs_005[, 1])))
total_pe <-array(NA, dim = c(n_conditions, n_participants, length(obs_005[, 1])))


for (j in 1:n_conditions) {
  if (j == 1) {
    obs = obs_005
    resp = resp_005
    real = real_005
  } else if (j == 2) {
    obs = obs_05
    resp = resp_05
    real = real_05
  } else if (j == 3) {
    obs = obs_10
    resp = resp_10
    real = real_10
  } else {
    obs = obs_20
    resp = resp_20
    real = real_20
  }
  
  t = 0
  
  for (h in 1:n_participants) {
    
    for (i in 2:length(obs_005[, 1])) {
      t = t + 1
      
      pe_errors[t] = obs[i, h] - resp[i, h]
      errors_real[t] = real[i, h] - resp[i, h]
      
      pe_errors2[i] = obs[i, h] - resp[i, h]
      errors_real2[i] = real[i, h] - resp[i, h]
      
    }
    
    total_errors[j, h,] <- errors_real2
    total_pe[j, h,] <- pe_errors2
    
  }
  
  full_pe[j,] <- pe_errors
  full_errors_real[j,] <- errors_real
  
}

#save(total_errors,file="total_errors.Rdata")

## HIstograms


pdf("Individual_errors.pdf",width = 11.5,height = 11.5)
par(mfrow=c(4,4))
for(i in 1:72){

par(mai = c(0.6, 0.7, 0.1, 0.1))
  
  
  mean_sd <- array(NA, dim = c(4, 4))
  
  mean_sd[2, 1] <- round(mean(total_errors[1,i,], na.rm = TRUE), 2)
  mean_sd[2, 2] <- round(mean(total_errors[2,i,], na.rm = TRUE), 2)
  mean_sd[2, 3] <- round(mean(total_errors[3,i,], na.rm = TRUE), 2)
  mean_sd[2, 4] <- round(mean(total_errors[4,i,], na.rm = TRUE), 2)
  
  mean_sd[4, 1] <- round(sd(total_errors[1,i,], na.rm = TRUE), 2)
  mean_sd[4, 2] <- round(sd(total_errors[2,i,], na.rm = TRUE), 2)
  mean_sd[4, 3] <- round(sd(total_errors[3,i,], na.rm = TRUE), 2)
  mean_sd[4, 4] <- round(sd(total_errors[4,i,], na.rm = TRUE), 2)

breaks=seq(from=-4,to=4,by=0.1)
h1 <- hist(total_errors[1,i,],plot=FALSE,breaks = breaks)
xvals <- h1$breaks
yvals <- h1$counts/sum(h1$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.35),axes=F,xlab="",ylab="")
h=hist(total_errors[1,i,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[1,i,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=as.character(seq(from=0,to=0.35,by=0.05)),at=seq(from=0,to=0.35,by=0.05)
)
text(0.3,0.35,"S/N = 0.05", cex=1.6)
text(-1.7,0.35,paste("S",i), cex=1.6,col=rgb(0,0,102, maxColorValue = 255,alpha = 255),font = 2())

legend(-0.6,0.3,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 1]))
           )))
       ),
       bty = 'n',
       cex = 2)

par(mai = c(0.6, 0.5, 0.1, 0.3))

h2 <- hist(total_errors[2,i,],plot=FALSE,breaks = breaks)
xvals <- h2$breaks
yvals <- h2$counts/sum(h2$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.35),axes=F,xlab="",ylab="")
h=hist(total_errors[2,i,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[2,i,])) 
plot(h,add=T)


axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=F,at=seq(from=0,to=0.35,by=0.05)
)
text(0.2,0.35,"S/N = 0.5", cex=1.6)
legend(-0.3,0.3,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 2]))
           )))
       ),
       bty = 'n',
       cex = 2)
par(mai = c(0.6, 0.5, 0.1, 0.3))

h3<- hist(total_errors[3,i,],plot=FALSE,breaks = breaks)
xvals <- h3$breaks
yvals <- h3$counts/sum(h3$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.35),axes=F,xlab="",ylab="")
h=hist(total_errors[3,i,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[3,i,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=F,at=seq(from=0,to=0.35,by=0.05)
)
text(0.2,0.35,"S/N = 1", cex=1.6)
legend(-0.3,0.3,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 3]))
           )))
       ),
       bty = 'n',
       cex = 2)

par(mai = c(0.6, 0.5, 0.1, 0.3))
h4<- hist(total_errors[4,i,],plot=FALSE,breaks = breaks)
xvals <- h4$breaks
yvals <- h4$counts/sum(h4$counts)
length(xvals)
length(yvals)
xvals <- c(xvals,xvals[length(xvals)])
yvals <- c(0,yvals,0)
plot(xvals,yvals,lwd=3,col="black",type="S",xlim=c(-2,2),ylim=c(0,0.35),axes=F,xlab="",ylab="")
h=hist(total_errors[4,i,],plot=F,breaks=breaks)
h$counts=h$counts/sum(!is.na(total_errors[4,i,])) 
plot(h,add=T)

axis(1,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,cex.axis = 1.7,pos=-0.002
)
axis(2,col = gris,col.ticks = gris,lwd = 2,
     lwd.ticks = 2,las = 2,cex.axis = 1.55,
     labels=F,at=seq(from=0,to=0.35,by=0.05)
)
text(0,0.35,"S/N = 2", cex=1.6)

legend(-0.3,0.3,
       c(
         sapply(1:1, function(x)
           as.expression(substitute(
             sigma== n,
             list(n = as.name(mean_sd[4, 4]))
           )))
       ),
       bty = 'n',
       cex = 2)

mtext("Proportion",side=2,line=-1.5, cex=1.2,outer = T)
mtext("Error (generative mean - response)",
      side=1,line=-2, cex=1.2,outer = T)


}

dev.off()







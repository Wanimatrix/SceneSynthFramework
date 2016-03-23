normalMean <- function(p,lb,le) {
	A = c((p[1]-lb[1]),(lb[1]-le[1]))
	B = c((p[2]-lb[2]),(lb[2]-le[2]))
	C = c((p[3]-lb[3]),(lb[3]-le[3]))

	q = A[2]^2 + B[2]^2 + C[2]^2
	r = 2*(A[1]*A[2] + B[1]*B[2] + C[1]*C[2])
	s = A[1]^2 + B[1]^2 + C[1]^2

	t = seq(0,1,0.0001)
	D = sqrt(q*t^2 + r*t + s)
	maxTerm = max(D)
	maxT = t[which.max(D)]
	minT = t[which.min(D)]

	minT
}

distanceToLine <- function(p, lb, le) {
	p2 = closestPointOnLine(p, lb, le)
	sqrt(sum((p2-p)^2))
}

closestPointOnLine <- function(p, lb, le) {
	Q = sum(lb^2)
	R = sum(le^2)
	S = sum(lb*le)
	T = sum(p*lb)
	U = sum(p*le)
	
	t = -(T-U-Q+S) / (Q + R - 2*S)

	lb + (le - lb)*t
}

getSamples <- function(p,lb1,le1,lb2,le2,std,amount) {

	samplesX = c()
	samplesY = c()
	meanX = normalMean(p,lb1,le1)
	p2 = p - (closestPointOnLine(p, lb1, le1) - lb1)
	meanY = normalMean(p2,lb2,le2)

	num = 0
	while(num < amount) {
		u = rnorm(1,meanX,std)
		v = rnorm(1,meanY,std)
		if(u >= 0 && u <= 1 && v >= 0 && v <= 1 && u+v < 1) {
			samplesX = c(samplesX,u)
			samplesY = c(samplesY,v)
			num = num+1
		}
	}

	matrix(c(samplesX,samplesY),2,amount,TRUE)
}


weights = c(0.9,0.1,0)

plot.new()
vMat = matrix(c(0,0,0,1,0,0,0.5,1,0),3,3,TRUE)
polygon(vMat[,1],vMat[,2])

lb1 = c(0,0,0)
le1 = c(1.5,1,0)
#p  = c(0.75,0.8,0)
p  = c(0.75,0.7,0)


d0 = distanceToLine(p,vMat[1,],vMat[2,])
d1 = distanceToLine(p,vMat[1,],vMat[3,])
d2 = distanceToLine(p,vMat[2,],vMat[3,])

print(sprintf("Distances: %f, %f, %f",d0,d1,d2))

lb1 = c()
le1 = c()
lb2 = c()
le2 = c()

if(d0 < d1 || d0 < d2) {
	lb1 = vMat[1,]
	le1 = vMat[2,]
	if(d1 < d2) {
		lb2 = vMat[1,]
		le2 = vMat[3,]		
	} else {
		tmp = lb1
		lb1 = le1
		le1 = tmp
		lb2 = vMat[2,]
		le2 = vMat[3,]
	}

	if(!(d0 < d1 && d0 < d2)) {
		tmp = lb1
		lb1 = lb2
		lb2 = tmp
		tmp = le1
		le1 = le2
		le2 = tmp
	}
} else {
	lb1 = vMat[3,]
	le1 = vMat[1,]
	lb2 = vMat[3,]
	le2 = vMat[2,]
	if(d2 < d1) {
		tmp = lb1
		lb1 = lb2
		lb2 = tmp
		tmp = le1
		le1 = le2
		le2 = tmp
	}
}
print(sprintf("Line 1: (%f,%f) -> (%f,%f)",lb1[1],lb1[2],le1[1],le1[2]))
print(sprintf("Line 2: (%f,%f) -> (%f,%f)",lb1[1],lb1[2],le2[1],le2[2]))

# samplesX = getSamples(p, lb1, le1, 1000)
# clop = closestPointOnLine(p, lb1, le1)
# p2 = p - (closestPointOnLine(p, lb1, le1) - lb1)
# samplesY = getSamples(p2, lb2, le2, 1000)

samples = getSamples(p,lb1,le1,lb2,le2,0.12,1000)
samplesX = samples[1,]
samplesY = samples[2,]

print(samplesX)
print(samplesY)

X = lb1[1] + ((le1[1]-lb1[1])*samplesX + (le2[1]-lb2[1])*samplesY)
Y = lb1[2] + ((le1[2]-lb1[2])*samplesX + (le2[2]-lb2[2])*samplesY)

points(X,Y)
points(p[1],p[2],col="#FF0000")
points(p2[1],p2[2],col="#FF0000")
points(clop[1],clop[2],col="#00FF00")

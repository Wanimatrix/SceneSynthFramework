PDF <- function(t,p,lb,le) {
	origT = t

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

#	print(minT)

	# Calculate normalisation factor (integral of -D + max(D))
#	t = c(0,maxT-0.1,maxT+0.1,1)
	t = c(0,1)
	factors = c(0.5,2,0.5)
	F = sqrt((t + (r/(2*q)))^2 + (sqrt(4*q*s-r^2)/(2*q))^2)
	inner = sqrt(q)* (((t+r/(2*q))*F/2) + (((4*q*s-r^2)*log((t + (r/(2*q)))+F))/(8*q^2)))
#	normFactor = - sum(factors*diff(inner)) + diff(maxterm * c(t[1],t[length(t)]))
	normFactor = - diff(inner) + diff(maxTerm * t)
	
	# Calculate the PDF
	t = origT
	D = sqrt(q*t^2 + r*t + s)
	#F = sqrt((t + (r/(2*q)))^2 + (sqrt(4*q*s-r^2)/(2*q))^2)
	#inner2 = (sqrt(q)* (((t+r/(2*q))*F/2) + (((4*q*s-r^2)*log((t + (r/(2*q)))+F))/(8*q^2))))
	#Ic = - (inner2 - inner[1]) + maxTerm * t

	((-D + maxTerm) / normFactor)

#	abs(dnorm(t, minT, 0.05) - u
}


CDF <- function(t,p,lb,le,u) {
	origT = t

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

#	print(minT)

	# Calculate normalisation factor (integral of -D + max(D))
#	t = c(0,maxT-0.1,maxT+0.1,1)
	t = c(0,1)
	factors = c(0.5,2,0.5)
	F = sqrt((t + (r/(2*q)))^2 + (sqrt(4*q*s-r^2)/(2*q))^2)
	inner = sqrt(q)* (((t+r/(2*q))*F/2) + (((4*q*s-r^2)*log((t + (r/(2*q)))+F))/(8*q^2)))
#	normFactor = - sum(factors*diff(inner)) + diff(maxterm * c(t[1],t[length(t)]))
	normFactor = - diff(inner) + diff(maxTerm * t)
	
	# Calculate the CDF
	t = origT
	F = sqrt((t + (r/(2*q)))^2 + (sqrt(4*q*s-r^2)/(2*q))^2)
	inner2 = (sqrt(q)* (((t+r/(2*q))*F/2) + (((4*q*s-r^2)*log((t + (r/(2*q)))+F))/(8*q^2))))
	Ic = - (inner2 - inner[1]) + maxTerm * t

	(Ic / normFactor) - u

#	abs(dnorm(t, minT, 0.05) - u
}

closestTOnLine <- function(p, lb, le) {
	Q = sum(lb^2)
	R = sum(le^2)
	S = sum(lb*le)
	T = sum(p*lb)
	U = sum(p*le)
	
	-(T-U-Q+S) / (Q + R - 2*S)
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

distanceToLine <- function(p, lb, le) {
	p2 = closestPointOnLine(p, lb, le)
	sqrt(sum((p2-p)^2))
}

getSamples <- function(p,lb,le,amount) {

	samples = c()

	u = runif(amount,0,1)
	foreach(i=1:amount) %do% {
		xmin <- uniroot(CDF, c(0, 1), tol = 0.0001, p = p, lb = lb, le = le, u = u[i])
		samples = c(samples,xmin$root)
	}

	return(samples)

	# PDF = (-D + maxTerm) / normFactor
	# CDF = Ic / normFactor
	
	# print(sprintf("CDF: %f",CDF))
	# points(tOrig,PDF)
	# points(tOrig,(D),col="#FF0000")
	
	# return(PDF)
}


# PDF = getPDF(p,lb,le,t)
# print(calcDistInt(p,lb,le,1,FALSE) - calcDistInt(p,lb,le,0,FALSE))
# / calcDistInt(p,lb,le,1,FALSE) - calcDistInt(p,lb,le,0,FALSE)
# polygon(t,PDF)
# area = abs(polyarea(c(0,t,1),c(0,PDF,0)))
# print(area)

# u = runif(1,0,1)
# print(u)
# str()
# getSamples(p,lb,le,xmin$root)

# CDF(xmin$root,p,lb,le,0)

weights = c(0.9,0.1,0)

plot.new()
vMat = matrix(c(0,0,0,1,0,0,0.5,1,0),3,3,TRUE)
polygon(vMat[,1],vMat[,2])
# vMat = matrix(c(0,0,1,0,1,1,0,1),4,2,TRUE)
# polygon(vMat[,1],vMat[,2])

# A1 = runif(1000,0,1)
# A2 = runif(1000,0,1)

# points(A1*vMat[2,1]+(1-A1)*A2*vMat[3,1],A1*vMat[2,2]+(1-A1)*A2*vMat[3,2])
# points(A1,A2)

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

samplesX = getSamples(p, lb1, le1, 1000)
clop = closestPointOnLine(p, lb1, le1)
p2 = p - (closestPointOnLine(p, lb1, le1) - lb1)
samplesY = getSamples(p2, lb2, le2, 1000)


print(samplesX)
print(samplesY)

X = lb1[1] + ((le1[1]-lb1[1])*samplesX + (le2[1]-lb2[1])*samplesY)
Y = lb1[2] + ((le1[2]-lb1[2])*samplesX + (le2[2]-lb2[2])*samplesY)

points(X,Y)
points(p[1],p[2],col="#FF0000")
points(p2[1],p2[2],col="#FF0000")
points(clop[1],clop[2],col="#00FF00")


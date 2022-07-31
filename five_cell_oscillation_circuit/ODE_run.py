import matplotlib.pyplot as plt
import numpy as np 
from scipy.integrate import solve_ivp
import math as math
n = 5
endtr = 600*1e3
clipt = -55*1e3
thr = 0

# ghc = 0.0025
ghc = 0.005
# ghc = 0.01
gel=0.5 #nS
#gel = 0:0.00025:0.0075
gsynA=5
gsyn = 0:0.00025:0.01
#lgel = length(gel)
#lgsyn = length(gsyn)

N0 = np.empty(n) 
	N0.fill(0)
H0 = np.empty(n)
	H0.fill(0)
Vm0 = np.empty(n)
    Vm0.fill(-65)

t_span=np.array[0 endtr] #not sure about the syntax

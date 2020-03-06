# Skrypt generuje prosta sciezke do testow path_extractora
# Domyslnie sciezka jest prosta, z parametrem 1 jest parabola, z parametrem 2 okregiem

import pickle
import numpy as np
import sys

data = {}
# Straight line
if len(sys.argv) == 1 or sys.argv[1] == '0':
    data['pathpoints'] = [
        (-5+0.2*i, 0)
        for i in range(50)
    ]

# Parabola
elif sys.argv[1] == '1':
    data['pathpoints'] = [
        (-5+0.2*i, (-5+0.2*i)*(-5+0.2*i)/2.5-5)
        for i in range(50)
    ]

# Circle
elif sys.argv[1] == '2':
    data['pathpoints'] = [
        (3*np.sin(i/10.0), 3*np.cos(i/10.0))
        for i in range(62)
    ]

with open('path_data.pkl', 'wb') as f:
    pickle.dump(data, f, 2)

import numpy as np

def integration(fun, t_s, x, h_s):
    for i in range(1, len(t_s)):
        x[:,i] = x[:,i-1] + h_s * (fun(t_s[i-1], x[:,i-1]))

    return t_s, x
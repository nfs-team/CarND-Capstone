from math import sqrt, exp

def logistic(x):
    """
    A function that returns a value between 0 and 1 for x in the 
    range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].

    Useful for cost functions.
    """
    return 2.0 / (1 + exp(-x)) - 1.0

def to_equation(coefficients):
    """
    Takes the coefficients of a polynomial and creates a function of
    time from them.
    """
    def f(t):
        total = 0.0
        for i, c in enumerate(coefficients): 
            total += c * t ** i
        return total
    return f

def differentiate(coefficients):
    """
    Calculates the derivative of a polynomial and returns
    the corresponding coefficients.
    """
    new_cos = []
    for deg, prev_co in enumerate(coefficients[1:]):
        new_cos.append((deg+1) * prev_co)
    return new_cos

def get_f_and_N_derivatives(coeffs, N=3):
    functions = [to_equation(coeffs)]
    for i in range(N):
        coeffs = differentiate(coeffs)
        functions.append(to_equation(coeffs))
    return functions


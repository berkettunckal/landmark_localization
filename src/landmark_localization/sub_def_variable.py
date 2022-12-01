#!/usr/bin/env python
# coding: utf-8
import numpy as np

def undefined_sd_var():
    return sd_var(float('nan'),float('nan'))       

def full_undefined_sd_var():
    return sd_var(-float('inf'),float('inf'))     

def sd_var_from_dict(d):    
    if not 'low' in d:
        raise ValueError('sd_var_from_dict: dictionary has no key low')
    if not 'high' in d:
        raise ValueError('sd_var_from_dict: dictionary has no key high')
    return sd_var(d['low'], d['high'])        

class sd_var(object):
    def __init__(self, low, high):
        if not (type(low) is int or type(low) is float):
            raise ValueError('SDM variable\'s low must be int or float')
        if not (type(high) is int or type(high) is float):
            raise ValueError('SDM variable\'s high must be int or float')
        if low > high:
            raise ValueError('SDM variable init error, low is greater than high')
        self.low = low
        self.high = high
    
    def to_dict(self):
        d = {}
        d['low'] = self.low
        d['high'] = self.high    
        return d
            
    def to_list(self):
        return [self.low, self.high]
    
    def __str__(self):
        return "[{}, {}]".format(self.low, self.high)
    
    def full_defined(self):
        return self.low == self.high
    
    def defined_with_acc(self, acc):
        return abs(self.high - self.low) < acc
    
    def defined(self):        
        if not (np.isnan(self.low) and np.isnan(self.high)):
            return self.low <= self.high
        return False
    
    def __eq__(self, other):
        if type(other) is sd_var:
            return self.low == other.low and self.high == other.high
        if type(other) is int or type(other) is float:
            if self.full_defined:
                return self.low == other
            return false 
                    
    def __lt__(self, other):
        if type(other) is int or type(other) is float:
            return self.high < other 
        raise ValueError('SDM variable cannot be compared with {} type!'.format(type(other)))            
    
    def __le__(self, other):
        if type(other) is int or type(other) is float:
            return self.high <= other 
        raise ValueError('SDM variable cannot be compared with {} type!'.format(type(other)))            
    
    def __gt__(self, other):
        if type(other) is int or type(other) is float:
            return self.low > other 
        raise ValueError('SDM variable cannot be compared with {} type!'.format(type(other)))            
    
    def __ge__(self, other):
        if type(other) is int or type(other) is float:
            return self.low >= other 
        raise ValueError('SDM variable cannot be compared with {} type!'.format(type(other)))            
    
    def __add__(self, other):
        if type(other) is int or type(other) is float:
            return sd_var(self.low + other, self.high + other)
        if type(other) is sd_var:
            return sd_var(self.low + other.low, self.high + other.high)
        raise ValueError('SDM variable cannot be added or substracted with {} type!'.format(type(other)))
    
    def __radd__(self, other):
        return self + other
    
    def __mul__(self, other):
        if type(other) is int or type(other) is float:           
            a = self.low * other
            b = self.high * other
            return sd_var(min(a,b),max(a,b))
        if type(other) is sd_var:
            a1 = self.low * other.low
            a2 = self.low * other.high
            b1 = self.high * other.low
            b2 = self.high * other.high
            return sd_var(min(a1,b1,a2,b2),max(a1,b1,a2,b2))
        raise ValueError('SDM multiply cannot be with {} type, only int, float and sd_var'.format(type(other)))
    
    def __rmul__(self, other):
        return self * other
                    
    def __sub__(self, other):
        return self + (other * -1)
    
    def __rsub__(self, other):
        return -1 * self + other
    
    def __truediv__(self, other):
        if type(other) is int or type(other) is float:
            return self * (1/other)
        if type(other) is sd_var:
            if other == full_undefined_sd_var():
                return full_undefined_sd_var()
            
            if 0 in other and other.low != 0 and other.high != 0:
                return full_undefined_sd_var()
            else:
                if other.high != 0:
                    a1 = self.low / other.high
                    b1 = self.high / other.high
                else:
                    a1 = float('inf') * int(np.sign(self.low))
                    b1 = float('inf') * int(np.sign(self.high))
                if other.low != 0:            
                    a2 = self.low / other.low            
                    b2 = self.high / other.low
                else:
                    a2 = float('inf') * int(np.sign(self.low))
                    b2 = float('inf') * int(np.sign(self.high))
                    
            return sd_var(min(a1,b1,a2,b2),max(a1,b1,a2,b2)) 
        
        raise ValueError('SDM devide cannot be with {} type, only int, float and sd_var'.format(type(other)))
                       
    def __rtruediv__(self, other):
        if type(other) is int or type(other) is float:
            a = other / self.low
            b = other / self.high
            return sd_var(min(a,b), max(a,b))
        
    def get_sqrt(self):
        if self.high < 0:
            return undefined_sd_var()
        if self.low < 0:
            return sd_var(0, float(np.sqrt(self.high)))
        return sd_var(float(np.sqrt(self.low)), float(np.sqrt(self.high)))
    
    def intersects(self, other, acc=0):
        if type(other) is sd_var:
            if min(self.high, other.high) + acc >= max(self.low, other.low):
                return True
            return False
        raise ValueError('SDM intersects can be done only with other sd_var, not {}'.format(type(other)))
            
    def assign(self, other):
        if type(other) is sd_var:
            #if min(self.high, other.high) >= max(self.low, other.low):
            if self.intersects(other):
                return sd_var(max(self.low, other.low), min(self.high, other.high))
            else:
                return undefined_sd_var()                
        raise ValueError('SDM assign can be done only with other sd_var, not {}'.format(type(other)))
    
    def join(self, other):
        if type(other) is sd_var:
            return sd_var(min(self.low, other.low),max(self.high, other.high))
        raise ValueError('SDM join can be done only with other sd_var, not {}'.format(type(other)))
    
    def around(self, dec):
        self.low = float(np.around(self.low, dec))
        self.high = float(np.around(self.high, dec))
            
    def __pow__(self, other):
        if type(other) is int or type(other) is float:
            a = self.low**other
            b = self.high**other            
            if 0 in self:
                return sd_var(min(a,b,0),max(a,b))
            else:
                return sd_var(min(a,b),max(a,b))            
            #return sd_var(min(a,b),max(a,b))
        raise ValueError('SDM power can be done only with int or float, not {}'.format(type(other)))
    
    def squeeze(self, value):
        self.low *= 1-value
        self.high *= value   
        
    def __contains__(self, item):
        if type(item) is int or type(item) is float:
            return item >= self.low and item <= self.high
        if type(item) is sd_var:
            return (item.low in self) and (item.high in self)            
        raise ValueError('SDM contains can be done only with int, float or sd_var, not {}'.format(type(item)))
    
    def sin(self):
        a = float(np.sin(self.low))
        b = float(np.sin(self.high))
        return sd_var(min(a,b), max(a,b))
    
    def cos(self):
        a = float(np.cos(self.low))
        b = float(np.cos(self.high))
        return sd_var(min(a,b), max(a,b))
    
    def tan(self):        
        if np.pi/2 in self or -np.pi/2 in self:
            return full_undefined_sd_var()
        a = float(np.tan(self.low ))
        b = float(np.tan(self.high )) 
        return sd_var(min(a,b), max(a,b))
    
    def arcsin(self):
        if self > 1:            
            #print(self)
            return undefined_sd_var()
        if self < -1:
            #print(self)
            return undefined_sd_var()
        if self.high > 1 and self.low < -1:
            return sd_var(-np.pi, np.pi)
        if self.high > 1:
            return sd_var(float(np.arcsin(self.low)), np.pi)
        if self.low < -1:
            return sd_var(-np.pi, float(np.arcsin(self.high)))
        a = float(np.arcsin(self.low))
        b = float(np.arcsin(self.high))
        return sd_var(min(a,b),max(a,b))
    
    def arccos(self):
        if self > 1:            
            return undefined_sd_var()
        if self < -1:
            return undefined_sd_var()
        if self.high > 1 and self.low < -1:
            return sd_var(-np.pi, np.pi)
        if self.high > 1:
            return sd_var(float(np.arccos(self.low)), np.pi)
        if self.low < -1:
            return sd_var(-np.pi, float(np.arccos(self.high)))
        a = float(np.arccos(self.low))
        b = float(np.arccos(self.high))
        return sd_var(min(a,b),max(a,b))
    
    def arctan(self):
        a = float(np.arctan(self.low))
        b = float(np.arctan(self.high))
        return sd_var(min(a,b), max(a,b))
    
    '''
    returns array of angles normilized to -pi;pi interval
    '''
    def norm_angle(self):
        if self.low < -np.pi and self.high > np.pi:
            return [sd_var(-np.pi, np.pi)]
        if self < -np.pi:
            a = self + np.pi*2
            return a.norm_angle()
        if self > np.pi:
            a = self - 2*np.pi
            return a.norm_angle()
        if self.low < -np.pi:
            a = sd_var(-np.pi, self.high)
            b = sd_var(self.low + 2*np.pi, np.pi)
            return [a] + b.norm_angle()
        if self.high > np.pi:
            a = sd_var(self.low, np.pi)
            b = sd_var(-np.pi, self.high - 2*np.pi)
            return [a] + b.norm_angle()
        return [self]
    
    def acc_eq(self, other, acc = 0.01):
        if acc == 0:
            return self == other
        if type(other) is sd_var:
            return np.abs(self.low - other.low) < acc and np.abs(self.high - other.high) < acc
        raise ValueError('acc_eq can be done only with two sd_vars, not {}'.format(type(other)))
    
    def abs(self):
        if self.low >= 0:
            return self
        if self.low < 0 and self.high >= 0:
            return sd_var( 0, float(max(np.abs(self.low, self.high)) ))        
        return sd_var( float(np.abs(self.high)), float(np.abs(self.high)) )         
    
    def module(self):
        return self.high - self.low
    
    def center(self):
        return (self.high + self.low)/2

##
# CLASSLESS FUNCTIONS
##

'''
atan2
gets two sd_vars
returns list of sd_vars (! not sd_mi)
'''
def atan2(y, x, eps = 0.00001):
    atan = (y/x).arctan()
    if x > 0:
        return [atan]
    if x < 0 and y >= 0:
        return [atan + np.pi]
    if x < 0 and y < 0:
        return [atan - np.pi]
    #return [atan]
    # so it is complicated
    xa = []
    if 0 in x:
        if x.high > eps:
            xa.append(sd_var(eps,x.high))
        if x.low < -eps:
            xa.append(sd_var(x.low, -eps))
    else:
        xa.append(x)
    ya = []
    if 0 in y:
        if y.high > eps:
            ya.append(sd_var(eps,y.high))
        if y.low < -eps:
            ya.append(sd_var(y.low, -eps))
    else:
        ya.append(y)
    res = []    
    for x_ in xa:
        #print(x_)
        for y_ in ya:
            #print(y_)
            res += atan2(y_, x_)
    return res
    
def is_close(a, b, sigma = 0.01):
    if np.abs(a.low - b.low) > sigma:
        return False
    if np.abs(a.high - b.high) > sigma:
        return False
    return True
    
            
#██╗░░░██╗███╗░░██╗██╗████████╗  ████████╗███████╗░██████╗████████╗░██████╗
#██║░░░██║████╗░██║██║╚══██╔══╝  ╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝██╔════╝
#██║░░░██║██╔██╗██║██║░░░██║░░░  ░░░██║░░░█████╗░░╚█████╗░░░░██║░░░╚█████╗░
#██║░░░██║██║╚████║██║░░░██║░░░  ░░░██║░░░██╔══╝░░░╚═══██╗░░░██║░░░░╚═══██╗
#╚██████╔╝██║░╚███║██║░░░██║░░░  ░░░██║░░░███████╗██████╔╝░░░██║░░░██████╔╝
#░╚═════╝░╚═╝░░╚══╝╚═╝░░░╚═╝░░░  ░░░╚═╝░░░╚══════╝╚═════╝░░░░╚═╝░░░╚═════╝░

if __name__ == '__main__':    
    
    # unit tests here    
    print("UNIT TESTING FOR SUBDEFENITE VARIABLE ARITHMETIC LOGIC")
    passed = 0
    tests_cnt = 0
    ## =========================================
    print("\n1.1 Init correctly")
    tests_cnt+=1
    try:
        print("\t{}".format(sd_var(1,2.5)))
        print("Test passed")
        passed+=1
    except ValueError as er:
        print("\tInit is failed, but shouldn't! Error: {}".format(er))
        print("Test failed")        
    ## =========================================
    print("\n2.1. Init incorrectly")
    tests_cnt+=1
    try:
        print("\t{}".format(sd_var(3,2)))
        print("Test failed")
        
    except ValueError as er:
        print("\tInit is failed, and it MUST! Error text: {}".format(er))
        print("Test passed")
        passed+=1
    ## =========================================
    print("\n2.2. Init incorrectly")
    tests_cnt+=1
    try:
        print("\t{}".format(sd_var(2,'4')))
        print("Test failed")
    except ValueError as er:
        print("\tInit is failed, and it MUST! Error text: {}".format(er))
        print("Test passed")
        passed+=1
    ## =========================================
    print("\n3.1. Equal")
    tests_cnt+=1
    try:
        a = sd_var(1,2)
        b = sd_var(1,2)
        print("\t{} == {}".format(a,b))
        if a == b:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, not equal!")
    except ValueError as er:
        print("\tComparation is failed! Error text: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n3.2. Equal (float int)")
    tests_cnt+=1
    try:
        a = sd_var(1,2)
        b = sd_var(1.,2.)
        print("\t{} == {}".format(a,b))
        if a == b:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, not equal!")
    except ValueError as er:
        print("\tComparation is failed! Error text: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n3.3. Equal (not)")
    tests_cnt+=1
    try:
        a = sd_var(1,2)
        b = sd_var(1,3)
        print("\t{} != {}".format(a,b))
        if a == b:
            print("Test failed, equal")
        else:
            print("Test passed")
            passed+=1
    except ValueError as er:
        print("\tComparation is failed! Error text: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n3.4. Equal to int")
    tests_cnt+=1
    try:
        a = sd_var(1,1)
        b = 1
        print("\t{} == {}".format(a,b))
        if a == b:
            print("Test passed")
            passed+=1            
        else:
            print("Test failed, equal")
    except ValueError as er:
        print("\tComparation is failed! Error text: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n4.1. Add with int")
    tests_cnt+=1
    try:
        a = sd_var(0,10.5)
        b = 5
        c = a + b
        d = sd_var(5,15.5)
        print("\t{} + {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tAddition is failed! Error text: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n4.2. Add with float right")
    tests_cnt+=1
    try:
        a = sd_var(0,10.5)
        b = 5.5
        c = b + a
        d = sd_var(5.5,16)
        print("\t{} + {} = {}".format(b,a,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tAddition is failed! Error text: {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n4.3. Add two sdm vars")
    tests_cnt+=1
    try:
        a = sd_var(0,5.5)
        b = sd_var(5,10)
        c = a + b
        d = sd_var(5,15.5)
        print("\t{} + {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tAddition is failed! Error text: {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n4.4. Add commutativus check")
    tests_cnt+=1
    flag_passed = True
    try:
        for i in range(100):
            a1 = np.random.uniform(-10,10)
            a2 = np.random.uniform(-10,10)
            b1 = np.random.uniform(-10,10)
            b2 = np.random.uniform(-10,10)
            a = sd_var(min(a1,a2),max(a1,a2))
            b = sd_var(min(b1,b2),max(b1,b2))
            if a + b != b + a:
                print("Test failed, for {} and {}".format(a, b))
                flag_passed = False
                break
        if flag_passed:
            print("Test passed")
            passed+=1
    except ValueError as er:
        print("\tAddition is failed! Error text: {}".format(er))
        print("Test failed")    
    ## ========================================
    print("\n5.1. Multiply positive int right")
    tests_cnt+=1
    try:        
        a = sd_var(1,5.5)
        b = 2
        c = b * a
        d = sd_var(2,11)
        print("\t{} * {} = {}".format(b,a,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nMultiply failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n5.2. Multiply negative int")
    tests_cnt+=1
    try:        
        a = sd_var(1,5.5)
        b = -1
        c = a * b
        d = sd_var(-5.5,-1)
        print("\t{} * {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nMultiply failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n5.3. Multiply positive sdm var")
    tests_cnt+=1
    try:        
        a = sd_var(1,2)
        b = sd_var(2,4)
        c = a * b
        d = sd_var(2,8)
        print("\t{} * {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nMultiply failed! Error {}".format(er))
        print("Test failed")
    ## =======================================
    print("\n5.4. Multiply negative sdm var")
    tests_cnt+=1
    try:        
        a = sd_var(1,2)
        b = sd_var(-4,-2)
        c = a * b
        d = sd_var(-8,-2)
        print("\t{} * {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nMultiply failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n5.5. Multiply commutativus check")
    tests_cnt+=1
    flag_passed = True
    try:
        for i in range(100):
            a1 = np.random.uniform(-10,10)
            a2 = np.random.uniform(-10,10)
            b1 = np.random.uniform(-10,10)
            b2 = np.random.uniform(-10,10)
            a = sd_var(min(a1,a2),max(a1,a2))
            b = sd_var(min(b1,b2),max(b1,b2))
            if a * b != b * a:
                print("Test failed, for {} and {}".format(a, b))
                flag_passed = False
                break
        if flag_passed:
            print("Test passed")
            passed+=1
    except ValueError as er:
        print("\tMultiply is failed! Error text: {}".format(er))
        print("Test failed")           
    ## ========================================
    print("\n6.1. Substract with int")
    tests_cnt+=1
    try:
        a = sd_var(0,10.5)
        b = 5
        c = a - b
        d = sd_var(-5,5.5)
        print("\t{} - {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tSubstriction is failed! Error text: {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n6.2. Substract with other sdm")
    tests_cnt+=1
    try:
        a = sd_var(0,10.5)
        b = sd_var(1,2)
        c = a - b
        d = sd_var(-2,9.5)
        print("\t{} - {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tSubstriction is failed! Error text: {}".format(er))
        print("Test failed")     
    ## ========================================
    print("\n6.3. Substract from float")
    tests_cnt+=1
    try:
        a = sd_var(0,10.5)
        b = 5
        c = b - a
        d = sd_var(-5.5,5)
        print("\t{} - {} = {}".format(b,a,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\tSubstriction is failed! Error text: {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n7.1. True devide int")
    tests_cnt+=1
    try:        
        a = sd_var(2,4)
        b = 2
        c = a / b
        d = sd_var(1,2)
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n7.2. True devide int")
    tests_cnt+=1
    try:        
        a = sd_var(20,40)
        b = sd_var(2,4)
        c = a / b
        d = sd_var(5,20)
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")      
    ## ========================================
    print("\n7.3. Int true devide sd_var")
    tests_cnt+=1
    try:        
        a = 4
        b = sd_var(1,2)
        c = a / b
        d = sd_var(2,4)
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n7.4. Negative float true devide sd_var")
    tests_cnt+=1
    try:        
        a = -4.5
        b = sd_var(1,2)
        c = a / b
        d = sd_var(-4.5,-2.25)
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n7.5. Positive sd_var true devide sd_var with 0")
    tests_cnt+=1
    try:        
        a = sd_var(1,2)
        b = sd_var(0,2)
        c = a / b
        d = sd_var(0.5,float('inf'))
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n7.6. Negative sd_var true devide sd_var with 0")
    tests_cnt+=1
    try:        
        a = sd_var(-2,-1)
        b = sd_var(0,2)
        c = a / b
        d = sd_var(-float('inf'),-0.5)
        print("\t{} / {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nDevide failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n8.1. Square root all positive")
    tests_cnt+=1
    try:        
        a = sd_var(4,16)        
        c = a.get_sqrt() 
        d = sd_var(2,4)
        print("\tsqrt({}) = {}".format(a,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n8.2. Square root all negative")
    tests_cnt+=1
    try:        
        a = sd_var(-16,-4)        
        c = a.get_sqrt()         
        print("\tsqrt({}) = [nan, nan]".format(a))
        if c.defined():
            print("Test failed, must be undeifined, but {}".format(c))
        else:
            print("Test passed")
            passed+=1
            
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n8.3. Square root all negative and positive")
    tests_cnt+=1
    try:        
        a = sd_var(-16,16)        
        c = a.get_sqrt() 
        d = sd_var(0,4)
        print("\tsqrt({}) = {}".format(a,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n8.4. Square root all negative and zero")
    tests_cnt+=1
    try:        
        a = sd_var(-16,0)        
        c = a.get_sqrt() 
        d = sd_var(0,0)
        print("\tsqrt({}) = {}".format(a,c))
        if c == d and c.full_defined():
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n9.1. Less than int")
    tests_cnt+=1
    try:        
        a = sd_var(-16,0)        
        b = 1                
        print("\t{} < {} and {} > {}".format(a,b,b,a))
        if a < b and b > a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")
    ## ========================================
    print("\n9.2. Greater or eq than float")
    tests_cnt+=1
    try:        
        a = sd_var(3,20)        
        b = 1.1                
        print("\t{} >= {} and {} <= {}".format(a,b,b,a))
        if a >= b and b <= a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nSqrt failed! Error {}".format(er))
        print("Test failed")    
    ## ========================================
    print("\n10.1. Assign intersected")
    tests_cnt+=1
    try:        
        a = sd_var(3,20)        
        b = sd_var(10, 100)
        c = a.assign(b)
        d = sd_var(10,20)
        print("\t {} : {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nAssign failed! Error {}".format(er))
        print("Test failed!")
    ## ========================================
    print("\n10.2. Assign not intersected")
    tests_cnt+=1
    try:        
        a = sd_var(3,20)        
        b = sd_var(30, 100)
        c = a.assign(b)
        d = undefined_sd_var()
        print("\t {} : {} = {}".format(a,b,c))
        if not c.defined():
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nAssign failed! Error {}".format(er))
        print("Test failed!")
    ## ========================================
    print("\n11.1. Power int")
    tests_cnt+=1
    try:        
        a = sd_var(2,4)        
        b = 2
        c = a**b
        d = sd_var(4,16)
        print("\t {} ^ {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nPower failed! Error {}".format(er))
        print("Test failed!")
    ## ========================================
    print("\n11.2. Power float")
    tests_cnt+=1
    try:        
        a = sd_var(4,16)        
        b = 1/2
        c = a**b
        d = sd_var(2,4)
        print("\t {} ^ {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nPower failed! Error {}".format(er))
        print("Test failed!")
    ## ========================================
    print("\n12.1. Join intersected")
    tests_cnt+=1
    try:        
        a = sd_var(4,16)        
        b = sd_var(10,20)
        c = a.join(b)
        d = sd_var(4,20)
        print("\t {} ++ {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nPower failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n12.2. Join not intersected")
    tests_cnt+=1
    try:        
        a = sd_var(4,5)        
        b = sd_var(10,20)
        c = a.join(b)
        d = sd_var(4,20)
        print("\t {} ++ {} = {}".format(a,b,c))
        if c == d:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nPower failed! Error {}".format(er))
        print("Test failed!")   
        
    ## ========================================
    print("\n13.1. Contains float")
    tests_cnt+=1
    try:        
        a = sd_var(4,5)                
        c = 4.5        
        print("\t {} in {}".format(c, a))
        if c in a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nContains failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n13.2. Not contains int")
    tests_cnt+=1
    try:        
        a = sd_var(4,5)                
        c = 6        
        print("\t not {} in {}".format(c, a))
        if not c in a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nContains failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n13.3. Contains sd_var")
    tests_cnt+=1
    try:        
        a = sd_var(4, 10)                
        c = sd_var(5, 9)       
        print("\t {} in {}".format(c, a))
        if c in a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nContains failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n13.4. Not contains sd_var")
    tests_cnt+=1
    try:        
        a = sd_var(4, 10)                
        c = sd_var(5, 12)       
        print("\t {} in {}".format(c, a))
        if not c in a:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(d))
    except ValueError as er:
        print("\nContains failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n14.1. Sinus")
    tests_cnt+=1
    try:        
        a = sd_var(0, np.pi/6)                
        b = a.sin()
        c = sd_var(0, 0.5)       
        print("\t sin({}) = {}".format(a, b))
        
        if c.high - b.high <= 0.01:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(c))
    except ValueError as er:
        print("\tSin failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n14.2. Arcsinus normal")
    tests_cnt+=1
    try:        
        a = sd_var(-0.5, 0.5)                
        b = a.arcsin()
        c = sd_var(-np.pi/6, np.pi/6)       
        print("\t sin({}) = {}".format(a, b))
        
        #if c.high - b.high <= 0.01 and c.low - b.low <= 0.01:
        if c.acc_eq(b, 0.01):
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(c))
    except ValueError as er:
        print("\tArcsin failed! Error {}".format(er))
        print("Test failed!")   
    ## ========================================
    print("\n14.3. Arcsinus not fully normal")
    tests_cnt+=1
    try:        
        a = sd_var(-2, 0.5)                
        b = a.arcsin()
        c = sd_var(-np.pi, np.pi/6)       
        print("\t sin({}) = {}".format(a, b))
        
        if c.high - b.high <= 0.01 and c.low - b.low <= 0.01:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(c))
    except ValueError as er:
        print("\tArcsin failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n14.4. Arcsinus not normal")
    tests_cnt+=1
    try:        
        a = sd_var(-3, 3)                
        b = a.arcsin()
        c = sd_var(-np.pi, np.pi)       
        print("\t sin({}) = {}".format(a, b))
        
        if c.high - b.high <= 0.01 and c.low - b.low <= 0.01:
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(c))
    except ValueError as er:
        print("\tArcsin failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n14.5. Arcsinus not normal 2")
    tests_cnt+=1
    try:        
        a = sd_var(-3, -2)                
        b = a.arcsin()
        #c = sd_var(-np.pi, np.pi)       
        print("\t sin({}) = {}".format(a, b))
        
        if not b.defined():
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(undefined_sd_var()))
    except ValueError as er:
        print("\tArcsin failed! Error {}".format(er))
        print("Test failed!")          
    ## ========================================
    print("\n14.6. Tan 1")
    tests_cnt+=1
    try:        
        a = sd_var(-1, 1)                                        
        b = sd_var(-1.557,1.557)
        c = a.tan()
        
        print("\t tan({}) = {}".format(a, c))        
        if is_close(b,c):
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(b))
    except ValueError as er:
        print("\tTan failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n14.7. Tan 2")
    tests_cnt+=1
    try:        
        a = sd_var(-1, 2)                                        
        b = full_undefined_sd_var()
        c = a.tan()
        
        print("\t tan({}) = {}".format(a, c))        
        if is_close(b,c):
            print("Test passed")
            passed+=1
        else:
            print("Test failed, must be {}".format(b))
    except ValueError as er:
        print("\tTan failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n15.1. Arctan2 x > 0 y > 0")
    tests_cnt+=1
    try:        
        a = sd_var(1, 2)                        
        b = sd_var(1, 2)
        c = atan2(a,b)
        print("\t atan2({},{}) = {}".format(a, b, c[0]))        
        if len(c) == 1:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tAtan2 failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n15.2. Arctan2 x < 0 y > 0")
    tests_cnt+=1
    try:        
        a = sd_var(-2, -1)                        
        b = sd_var(1, 2)
        c = atan2(a,b)
        print("\t atan2({},{}) = {}".format(a, b, c[0]))        
        if len(c) == 1:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tAtan2 failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n15.3. Arctan2 x < 0 y < 0")
    tests_cnt+=1
    try:        
        a = sd_var(-2, -1)                        
        b = sd_var(-2, -1)
        c = atan2(a,b)
        print("\t atan2({},{}) = {}".format(a, b, c[0]))        
        if len(c) == 1:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tAtan2 failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n15.4. Arctan2 0 e x y > 0")
    tests_cnt+=1
    try:        
        a = sd_var(-2, 2)                        
        b = sd_var(3, 4)
        c = atan2(a,b)        
        print("\t atan2({},{}) = ".format(a, b))        
        for c_ in c:
            print(c_)
        if len(c) == 1:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tAtan2 failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n15.5. Arctan2 0 e x, 0 e y")
    tests_cnt+=1
    try:        
        a = sd_var(-2, 2)                        
        b = sd_var(-1, 1)
        c = atan2(a,b)        
        print("\t atan2({},{}) = ".format(a, b))        
        for c_ in c:
            print(c_)
        if len(c) == 4:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tAtan2 failed! Error {}".format(er))
        print("Test failed!")  
    ## ========================================
    print("\n16.1. Norm angle")
    tests_cnt+=1
    try:        
        a = sd_var(0, 4)    
        c = a.norm_angle()        
        print("\tnorm angle a -->".format(a))        
        for c_ in c:
            print(c_)
        if len(c) == 2:
            print("Test passed")
            passed+=1
        else:
            print("Test failed")
    except ValueError as er:
        print("\tNorm angle failed! Error {}".format(er))
        print("Test failed!")
    
    print("\nDONE! PASSED {} FROM {}".format(passed, tests_cnt))
        

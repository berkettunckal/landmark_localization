#!/usr/bin/env python
# coding: utf-8
import numpy as np
import landmark_localization.sub_def_variable as sd_variable
from landmark_localization.sub_def_variable import sd_var
from functools import cmp_to_key

class sd_mi(object):
    '''
    do_norm is False means that you are super sure that vars are already normilized!!!
    '''
    def __init__(self, sd_vars = [], do_norm = True, acc = 0):
        for var in sd_vars:
            if not type(var) is sd_var:
                raise ValueError('Init failed. Sub Def multi interval supports only sd_var as members! Not {}'.format(type(var)))
        self.sd_vars = sd_vars
        if do_norm:
            self.norm(acc)
        self.normilized = True
        self.i = 0
        
    def to_list(self):
        l = []
        for sdv in self.sd_vars:
            l.append(sdv.to_dict())
        return l
                
    def __str__(self):
        str_mi = "{"
        for i, var in enumerate(self.sd_vars):
            str_mi += var.__str__()
            if i < len(self.sd_vars)-1:
                str_mi += ", "
        str_mi += "}"
        return str_mi
    
    def __repr__(self):
        return self.__str__()
    
    def __len__(self):
        return len(self.sd_vars)
    
    '''
    iterator stuff
    '''
    def switch_next(self):
        self.i += 1
        if self.i >= len(self.sd_vars):
            self.i = 0
            return False
        return True
        
    def get_cur(self):
        return self.sd_vars[self.i]
        
    def __getitem__(self, indices):
        return self.sd_vars[indices]
        
    '''
    Merges all self-intersections, and sort multi interval by increase
    '''
    def norm(self, acc = 0):
        # sort
        self.sd_vars.sort(key=cmp_to_key(lambda x, y: 1 if x.low > y.low else -1))
        # merge
        while True:
            all_ok = True
            for i in range(len(self.sd_vars)-1):
                if self.sd_vars[i].intersects(self.sd_vars[i+1], acc):
                    self.sd_vars[i] = self.sd_vars[i].join(self.sd_vars[i+1])
                    del self.sd_vars[i+1]
                    all_ok = False
                    break
            if all_ok:
                break
            
    def __eq__(self, other):
        if type(other) is sd_mi:
            if not self.normilized:
                self.norm()
            if not other.normilized:
                other.norm()
            if len(self) != len(other):
                return False
            for v1, v2 in zip(self.sd_vars, other.sd_vars):
                if v1 != v2:
                    return False
            return True
        raise ValueError('Sub Def Multi Interval can be compared only with other Sub Def Multi Interval, not {}'.format(type(other)))
    
    def __lt__(self, other):
        if type(other) is sd_mi:
            raise NotImplementedError('Comparation sd_mi with sd_mi not yet implemented')
        return self[-1] < other
    
    def __le__(self, other):
        if type(other) is sd_mi:
            raise NotImplementedError('Comparation sd_mi with sd_mi not yet implemented')
        return self[-1] <= other
    
    def __gt__(self, other):
        if type(other) is sd_mi:
            raise NotImplementedError('Comparation sd_mi with sd_mi not yet implemented')
        return self[0] > other
    
    def __ge__(self, other):
        if type(other) is sd_mi:
            raise NotImplementedError('Comparation sd_mi with sd_mi not yet implemented')
        return self[0] >= other
    
    def get_min(self):
        if not self.normilized:
            self.norm()
        return self.sd_vars[0].low
        
    def get_max(self):
        max_el = self.sd_vars[0].high
        for var in self.sd_vars:
            if max_el < var.high:
                max_el = var.high
        return max_el
    
    def __add__(self, other):
        if type(other) is int or type(other) is float:
            res = []
            for var in self.sd_vars:
                res.append(var + other)
            return sd_mi(res, do_norm = False)
        if type(other) is sd_var:
            res = []
            for var in self.sd_vars:
                res.append(var + other)
            return sd_mi(res)
        if type(other) is sd_mi:
            a = sd_var(other.get_min(), other.get_max())
            return self + a
        raise ValueError('Sub Def Multi Interval can be added only with: int, float, sd_var and sd_mi, not {}'.format(type(other)))
    
    def __radd__(self, other):
        return self + other
    
    def __mul__(self, other):
        if type(other) is int or type(other) is float or type(other) is sd_var:
            res = []
            for var in self.sd_vars:
                res.append(var * other)
            return sd_mi(res)
        if type(other) is sd_mi:
            a = sd_var(other.get_min(), other.get_max())
            return self * a
        raise ValueError('Sub Def Multi Interval can be multiplied only with: int, float, sd_var and sd_mi, not {}'.format(type(other)))
            
    def __rmul__(self, other):
        return self * other
    
    def __sub__(self, other):
        return self + (other * -1)
    
    def __rsub__(self, other):
        return -1 * self + other
    
    def __pow__(self, other):
        if type(other) is int or type(other) is float:
            res = []
            for var in self.sd_vars:
                res.append(var ** other)
            return sd_mi(res)
        raise ValueError('Sub Def Multi Interval can be powered only with: int, float not {}'.format(type(other)))
                
    def get_sqrt(self, neg_values = True):
        res = []
        for var in self.sd_vars:
            sqrt = var.get_sqrt()
            if sqrt.defined():
                res.append(sqrt)
                if neg_values:
                    res.append(sqrt * -1)
        return sd_mi(res)
    
    def assign(self, other):        
        if type(other) is sd_mi:
            res = []
            for var1 in self.sd_vars:
                for var2 in other.sd_vars:
                    if var1.intersects(var2):
                        res.append(var1.assign(var2))
            
            return sd_mi(res)
        raise ValueError('Sub Def Multi Interval can be assigned only with other sd_mi not {}'.format(type(other)))
    
    def around(self, dec):
        for var in self.sd_vars:
            var.around(dec)
            
    def full_defined(self):
        if len(self) != 1:
            return False
        return self.sd_vars[0].full_defined()
    
    def defined(self):
        if len(self) == 0:
            return False
        for var in self.sd_vars:
            if var.defined():
                return True
        return False
    
    def defined_with_acc(self, acc):
        for var in self.sd_vars:
            if not var.defined_with_acc(acc):
                return False
        return True         
    
    def squeeze(self, value):
        for var in self.sd_vars:
            var.squeeze(value)
            
    def __contains__(self, item):
        if type(item) is int or type(item) is float or type(item) is sd_var:
            for var in self.sd_vars:
                if item in var:
                    return True
            return False
        if type(item) is sd_mi:            
            for i_v in item.sd_vars:
                if not i_v in self:
                    return False
            return True    
        raise ValueError('Sub Def Multi Interval contains can be done only with int, float, sd_var and sd_mi not {}'.format(type(item)))
    
    def sin(self):
        res = []
        for var in self.sd_vars:
            res.append(var.sin())
        return sd_mi(res)
    
    def cos(self):
        res = []
        for var in self.sd_vars:
            res.append(var.cos())
        return sd_mi(res)
    
    def tan(self):
        res = []
        for var in self.sd_vars:
            res.append(var.tan())
        return sd_mi(res)
    
    def arcsin(self):
        res = []
        for var in self.sd_vars:
            res.append(var.arcsin())
        return sd_mi(res)
    
    def arccos(self):
        res = []
        for var in self.sd_vars:
            res.append(var.arccos())
        return sd_mi(res)
    
    def __truediv__(self, other):
        if type(other) is float or type(other) is int or type(other) is sd_var:
            res = []
            for var in self.sd_vars:
                res.append(var / other)
            return sd_mi(res)         
        if type(other) is sd_mi:
            a = sd_var(other.get_min(), other.get_max())
            return self / a
        raise ValueError('Sub Def Multi Interval devide cannot be with {} type, only int, float and sd_var'.format(type(other)))
    
    def __rtruediv__(self, other):
        if type(other) is int or type(other) is float:
            res = []
            for var in self.sd_vars:
                res.append(other / var)
            return sd_mi(res)         
        raise ValueError('Sub Def Multi Interval cannot devide type {}, only int, float'.format(type(other)))
    
    def arctan(self):
        res = []
        for var in self.sd_vars:
            res.append(var.arctan())
        return sd_mi(res)
    
    def norm_angle(self):
        res = []
        for var in self.sd_vars:
            res += var.norm_angle()
        return sd_mi(res)
    
    def acc_eq(self, other, acc = 0.01):
        if type(other) is sd_mi:
            if not self.normilized:
                self.norm()
            if not other.normilized:
                other.norm()
            if len(self) != len(other):
                return False
            for v1, v2 in zip(self.sd_vars, other.sd_vars):
                if not v1.acc_eq(v2, acc):
                    return False
            return True
        raise ValueError('Sub Def Multi Interval can be compared only with other Sub Def Multi Interval, not {}'.format(type(other)))
    
    def intersects(self, other, acc = 0):
        if type(other) is sd_mi:
            for var1 in self.sd_vars:
                for var2 in other.sd_vars:
                    if var1.intersects(var2, acc):
                        return True
            return False
        raise ValueError('Sub Def Multi Interval can be intersected only with other Sub Def Multi Interval, not {}'.format(type(other)))
    
    def module(self):
        m = 0
        for sd_var in self.sd_vars:
            m += sd_var.module()
        return m
    
##
# CLASSLESS FUNCTIONS
##

def atan2(y, x):
    res = []
    for y_ in y.sd_vars:
        for x_ in x.sd_vars:
            res+=sd_variable.atan2(y_,x_)
    a = sd_mi(res)
    a.norm(acc = 0.01)
    return a
    
#██╗░░░██╗███╗░░██╗██╗████████╗  ████████╗███████╗░██████╗████████╗░██████╗
#██║░░░██║████╗░██║██║╚══██╔══╝  ╚══██╔══╝██╔════╝██╔════╝╚══██╔══╝██╔════╝
#██║░░░██║██╔██╗██║██║░░░██║░░░  ░░░██║░░░█████╗░░╚█████╗░░░░██║░░░╚█████╗░
#██║░░░██║██║╚████║██║░░░██║░░░  ░░░██║░░░██╔══╝░░░╚═══██╗░░░██║░░░░╚═══██╗
#╚██████╔╝██║░╚███║██║░░░██║░░░  ░░░██║░░░███████╗██████╔╝░░░██║░░░██████╔╝
#░╚═════╝░╚═╝░░╚══╝╚═╝░░░╚═╝░░░  ░░░╚═╝░░░╚══════╝╚═════╝░░░░╚═╝░░░╚═════╝░
            
if __name__ == '__main__':    
    
    # unit tests here 
    print("UNIT TESTING FOR SUBDEFENITE MILTI INTERVAL ARITHMETIC LOGIC")
    passed = 0
    tests_cnt = 0
    ## =========================================
    print("\n1.1 Init correctly")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3, 4)
        A = sd_mi([a, b])
        print("\t{}".format(A))
        print("Test passed")
        passed+=1
    except ValueError as er:
        print("\tInit is failed, but it shouldn't! Error: {}".format(er))
        print("Test failed")  
    ## =========================================
    print("\n1.2 Init incorrectly")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = 8
        A = sd_mi([a, b])
        print("\t{}".format(A))
        print("Test failed")
        
    except ValueError as er:
        print("\tInit is failed, but it SHOULD, don't worry! Error: {}".format(er))
        print("Test passed")  
        passed+=1
    ## =========================================
    print("\n1.3 Equal")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        B = sd_mi([sd_var(1,2), sd_var(3.5,4.5)])
        print("\t{} == {}".format(A, B))
        if A == B:
            print("Test passed")  
            passed+=1
        else:
            print("Test failed")
        
    except ValueError as er:
        print("\tComparation is failed! Error: {}".format(er))
    ## =========================================
    print("\n1.4 Not equal")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3, 4)
        c = sd_var(3, 3.5)
        A = sd_mi([a, b])
        B = sd_mi([a, c])
        print("\t{} != {}".format(A, B))
        if A != B:
            print("Test passed")  
            passed+=1
        else:
            print("Test failed")
        
    except ValueError as er:
        print("\tComparation is failed! Error: {}".format(er))
    ## =========================================
    print("\n1.5. Init correctly, but not normilized")
    tests_cnt+=1
    try:
        a = sd_var(1, 4)
        b = sd_var(3, 5)
        c = sd_var(6, 7)
        A = sd_mi([a, b, c])
        B = sd_mi([sd_var(1,5), c])
        print("\t{} {} {} --> {}".format(a,b,c,A))
        if A == B:
            print("Test passed")
            passed+=1
        else:
            print("\tWrong! Should be {}".format(B))
            print("Test failed")
    except ValueError as er:
        print("\tInit is failed, but it shouldn't! Error: {}".format(er))
        print("Test failed")  
    ## =========================================
    print("\n2.1. Add int")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = 1
        B = A + c
        C = sd_mi([sd_var(2,3),sd_var(4.5,5.5)])
        print("\t{} + {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tAddition is failed! Error: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n2.2. Add sd_var")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = sd_var(-1,1)
        B = A + c
        C = sd_mi([sd_var(0,5.5)])
        print("\t{} + {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tAddition is failed! Error: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n2.3. Add sd_mi")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(7.5, 8.5)
        A = sd_mi([a, b])
        D = sd_mi([sd_var(-1,0),sd_var(1,2)])
        B = A + D
        C = sd_mi([sd_var(0,4), sd_var(6.5, 10.5)])
        print("\t{} + {} = {}".format(A, D, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tAddition is failed! Error: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n3.1. Multiply float")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = 0.5
        B = A * c
        C = sd_mi([sd_var(0.5,1),sd_var(1.75,2.25)])
        print("\t{} * {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n3.2. Multiply sd_var")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = sd_var(-1,1)
        B = A * c
        C = sd_mi([sd_var(-4.5,4.5)])
        print("\t{} * {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n3.3. Multiply sd_var 2")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = sd_var(1,2)
        B = A * c
        C = sd_mi([sd_var(1,9)])
        print("\t{} * {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n3.4. Multiply sd_mi")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = sd_mi([sd_var(1,2),sd_var(3,4)])
        B = A * c
        C = sd_mi([sd_var(1,18)])
        print("\t{} * {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## =========================================
    print("\n3.5. Multiply neg int")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(3.5, 4.5)
        A = sd_mi([a, b])
        c = -1
        B = A * c
        C = sd_mi([sd_var(-4.5,-3.5),sd_var(-2,-1)])
        print("\t{} * {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")
        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n4.1. Power int")
    tests_cnt+=1
    try:
        a = sd_var(1, 2)
        b = sd_var(4, 6)
        A = sd_mi([a, b])
        c = 2
        B = A ** c
        C = sd_mi([sd_var(1,4), sd_var(16, 36)])
        print("\t{} ** {} = {}".format(A, c, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n5.1. Sqrt positive")
    tests_cnt+=1
    try:
        a = sd_var(1, 4)
        b = sd_var(9, 16)
        A = sd_mi([a, b])        
        B = A.get_sqrt()
        C = sd_mi([sd_var(-4, -3),sd_var(-2,-1),sd_var(1,2), sd_var(3, 4)])
        print("\tsqrt({}) = {}".format(A, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n5.2. Sqrt half-neg and positive")
    tests_cnt+=1
    try:
        a = sd_var(-4, 4)
        b = sd_var(9, 16)
        A = sd_mi([a, b])        
        B = A.get_sqrt()
        C = sd_mi([sd_var(-4, -3),sd_var(-2,2),sd_var(3, 4)])
        print("\tsqrt({}) = {}".format(A, B))
        if B == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n6.1. Assign")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        B = sd_mi([sd_var(0, 6), sd_var(5, 6), sd_var(15, 20)])
        D = A.assign(B)        
        C = sd_mi([sd_var(0, 4),sd_var(15, 16), sd_var(19,20)])
        print("\t{} : {} = {}".format(A, B, D))
        if D == C:
            print("Test passed")  
            passed+=1
        else:
            print("\tWrong! Must be {}".format(C))
            print("Test failed")        
    except ValueError as er:
        print("\tMultiplication is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n7.1. Contains int")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = 0                
        print("\t{} in {}".format(b, A))
        if b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n7.2. Not contains int")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = 6                
        print("\tnot {} in {}".format(b, A))
        if not b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n7.3. Contains sd_var")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = sd_var(10,15)                
        print("\t{} in {}".format(b, A))
        if b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================    
    print("\n7.4. Not contains sd_var")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = sd_var(-5,1)                
        print("\tnot {} in {}".format(b, A))
        if not b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================    
    print("\n7.5. Contains sd_mi")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = sd_mi([sd_var(-1,1), sd_var(10,15)])
        print("\t{} in {}".format(b, A))
        if b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n7.6. Not contains sd_mi")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        b = sd_mi([sd_var(-1,1), sd_var(8,17)])                
        print("\tnot {} in {}".format(b, A))
        if not b in A:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tContains is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n8.1. Arctan")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, 4), sd_var(9, 16), sd_var(19, 21)])        
        B = sd_mi([sd_var(-1,1), sd_var(8,17)])                        
        C = atan2(A, B)
        print("\tatan2({} ,{}) = {}".format(A, B, C))
        if True:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tArctan2 is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n8.2. Arctan of circle agains inner point")
    tests_cnt+=1
    try:
        X = sd_mi([sd_var(-1,1)])
        Y = sd_mi([sd_var(-1,1)])
        a = [0.1,0.1]
        A = atan2(Y -a[1], X-a[0])
        B = sd_mi([sd_var(-np.pi, np.pi)])
        print("atan({}, {}) = {}".format(Y,X,A))
        if A.sd_vars[0].intersects(B.sd_vars[0], acc = 0.01):
            print("Test passed")
            passed+=1
        else:
            print("\nAtan2 failed, must be {}".format(B))
    except ValueError as er:
        print("\tAtan2 failed, Error: {}".format(er))
    ## ==========================================
    print("\n8.3. Arctan of circle agains outer point")
    tests_cnt+=1
    try:
        X = sd_mi([sd_var(-1,1)])
        Y = sd_mi([sd_var(-1,1)])
        a = [100,0.1]
        A = atan2(Y -a[1], X-a[0])
        B = sd_mi([sd_var(-np.pi, np.pi)])
        print("atan({}-{}, {}-{}) = {}".format(Y,a[1],X,a[0],A))
        if A.sd_vars[0].intersects(B.sd_vars[0], acc = 0.01):
            print("Test passed")
            passed+=1
        else:
            print("\nAtan2 failed, must be {}".format(B))
    except ValueError as er:
        print("\tAtan2 failed, Error: {}".format(er))
    ## ==========================================
    print("\n8.4. Arctan of circle agains outer point 2")
    tests_cnt+=1
    try:
        X = sd_mi([sd_var(-1,1)])
        Y = sd_mi([sd_var(-1,1)])
        a = [-100,0]
        A = atan2(Y -a[1], X-a[0])
        B = sd_mi([sd_var(-np.pi, np.pi)])
        print("atan({}-{}, {}-{}) = {}".format(Y,a[1],X,a[0],A))
        if A.sd_vars[0].intersects(B.sd_vars[0], acc = 0.01):
            print("Test passed")
            passed+=1
        else:
            print("\nAtan2 failed, must be {}".format(B))
    except ValueError as er:
        print("\tAtan2 failed, Error: {}".format(er))
    ## =============================================
    print("\n8.5. Arctan of square agains inner point")
    tests_cnt+=1
    try:
        X = sd_mi([sd_var(-100,70)])
        Y = sd_mi([sd_var(-50,80)])
        a = [-20,10]
        A = atan2(Y -a[1], X-a[0])
        B = sd_mi([sd_var(-np.pi, np.pi)])
        print("atan({}-{}, {}-{}) = {}".format(Y,a[1],X,a[0],A))
        if A.sd_vars[0].intersects(B.sd_vars[0], acc = 0.01):
            print("Test passed")
            passed+=1
        else:
            print("\nAtan2 failed, must be {}".format(B))
    except ValueError as er:
        print("\tAtan2 failed, Error: {}".format(er))
    ## ==========================================
    print("\n9.1. Norm angle")
    tests_cnt+=1
    try:        
        A = sd_mi([sd_var(-4, -2), sd_var(2, 4)])        
        B = A.norm_angle()                        
        print("\tnorm angle({}) --> {}".format(A, B))
        if True:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tArctan2 is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n9.2. Norm angle of sd_var and const")
    tests_cnt+=1
    try:        
        a = 1
        A = sd_mi([sd_var(-np.pi, np.pi)])      
        B = (A+a).norm_angle()                        
        print("\tnorm angle({} + {}) --> {}".format(A, a, B))
        if True:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tArctan2 is failed! Error: {}".format(er))
        print("Test failed")
    ## ==========================================
    print("\n9.3. Norm angle of sd_var and sd_var")
    tests_cnt+=1
    try:        
        a = sd_var(0,1)
        A = sd_mi([sd_var(-np.pi, np.pi)])      
        B = (A+a).norm_angle()                        
        print("\tnorm angle({} + {}) --> {}".format(A, a, B))
        if True:
            print("Test passed")  
            passed+=1
        else:            
            print("Test failed")        
    except ValueError as er:
        print("\tArctan2 is failed! Error: {}".format(er))
        print("Test failed")
    
    
    print("\nDONE! PASSED {} FROM {}".format(passed, tests_cnt))

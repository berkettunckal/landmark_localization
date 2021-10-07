#!/usr/bin/env python
# coding: utf-8
import numpy as np
from landmark_localization.sub_def_variable import sd_var
from landmark_localization.sub_def_multi_interval import sd_mi, atan2
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import time
import itertools

'''
functions unifications, to work with sd_mi and floats
'''
def u_sqrt(var, neg_values = True):
    if type(var) is int or type(var) is float:
        return float(np.sqrt(var))
    if type(var) is sd_var:
        if neg_values:
            raise ValueError('unificated sqrt can return sd_var only with neg_values param is False!')
        return var.get_sqrt()
    if type(var) is sd_mi:
        return var.get_sqrt(neg_values = neg_values)
    else:
        raise ValueError('unificated sqrt can works only with int, float, sd_var or smd_mi, not {} of type {}'.format(var, type(var)))
    
def u_atan2(y, x):
    if type(y) is sd_mi or type(x) is sd_mi:
        return atan2(y, x)
    if (type(y) is float or type(y) is int) and (type(x) is float or type(x) is int):
        return float(np.arctan2(y, x))
    else:
        raise ValueError('unificated atan2 can works only with int\\float or sd_mi, not {} and {}'.format(type(y), type(x)))
    
def u_tan(var):
    if type(var) is int or type(var) is float:
        return float(np.tan(var))
    if type(var) is sd_mi:
        return var.tan()
    else:
        raise ValueError('unificated tan can works only with int\\float or smd_mi, not {}'.format(type(var)))
    
def u_sin(var):
    if type(var) is int or type(var) is float:
        return float(np.sin(var))
    if type(var) is sd_mi:
        return var.sin()
    else:
        raise ValueError('unificated sin can works only with int\\float or smd_mi, not {}'.format(type(var)))
    
def u_cos(var):
    if type(var) is int or type(var) is float:
        return float(np.cos(var))
    if type(var) is sd_mi:
        return var.cos()
    else:
        raise ValueError('unificated cos can works only with int\\float or smd_mi, not {}'.format(type(var)))
    
def u_norm_angle(var):
    if type(var) is int or type(var) is float:
        return float((var + np.pi) % (2*np.pi) - np.pi)
    if type(var) is sd_mi:
        return var.norm_angle()
    else:
        raise ValueError('unificated norm_angle can works only with int\\float or smd_mi, not {}'.format(type(var)))
    
def u_arcsin(var):
    if type(var) is int or type(var) is float:
        return float(np.arcsin(var))
    if type(var) is sd_mi:
        return var.arcsin()
    else:
        raise ValueError('unificated arcsin can works only with int\\float or smd_mi, not {}'.format(type(var)))
    
'''
Sub Defenite Model class
'''
class SDM(object):
    
    def __init__(self, var_acc = 3, stop_acc = 0.01, verbose = True, max_steps = 100, max_mc_rolls = 64, use_correctness_check = True):
        self.variables = {}
        self.constrants = {}
        self.correctnesses = {}
        self.var_acc = var_acc
        self.stop_acc = stop_acc
        self.active_func = []
        self.verbose = verbose
        self.max_steps = max_steps                
        self.max_mc_rolls = max_mc_rolls
        self.use_correctness_check = use_correctness_check
        '''
        functions:
            name:
                calls: int # how much times was been called
                updates: int # how many updates did
        duration:
            start:
            end:
            total: float
            
        '''
        self.reset_stats()
    
    '''
    type_var: 'COORD', 'ANGLE'
    '''
    def register_variable(self, name, init_value, type_var = "COORD"):
        var = {}
        var['NAME'] = name
        var['VALUE'] = init_value
        var['TYPE'] = type_var
        self.variables[name] = var
    
    '''
    proc: type of process to handle this constrant
        DIRECT_SUBSTITUTION - easy calc, but works only work for monotonous functions
        ROV_MONTE_CARLO - gets range of value with MonteCarlo aglorithm
    '''
    def register_constrant(self, name, function, out_var, input_vars, proc = 'DIRECT_SUBSTITUTION'):
        constrant = {}
        constrant['NAME'] = name
        constrant['FUNC'] = function
        constrant['OUT'] = out_var
        constrant['INPUT'] = input_vars        
        constrant['PROC'] = proc
        self.constrants[name] = constrant
        
    def register_correctness(self, name, function, input_vars):
        correctness = {}
        correctness['NAME'] = name
        correctness['FUNC'] = function
        correctness['INPUT'] = input_vars
        self.correctnesses[name] = correctness
        
    def check_correctness_new(self, name):
        input_vals = []
        correct = []
        for var_name in self.correctnesses[name]['INPUT']:
            input_vals.append(list(range(len(self.variables[var_name]['VALUE']))))
        cnt = 0
        for el in itertools.product(*input_vals):
            cnt+=1
            #print(el, self.correctnesses[name]['INPUT'][0])
            #print(self.variables[self.correctnesses[name]['INPUT'][0]])
            inp = [sd_mi([self.variables[self.correctnesses[name]['INPUT'][i]]['VALUE'][v]]) for i, v in enumerate(el)]
            #print('INPUT', inp)
            if self.correctnesses[name]['FUNC'](inp):
                correct.append(el)
        #print(correct)
        if len(correct) == 0:
            return False
        if len(correct) == cnt:
            return True
        for i, var_name in enumerate(self.correctnesses[name]['INPUT']):
            ok = (set([n[i] for n in correct]))
            #print(ok)
            #print("{} -->".format(self.variables[var_name]['VALUE']))
            self.variables[var_name]['VALUE'] = sd_mi([self.variables[var_name]['VALUE'][o] for o in ok])
            #print("--> {}".format(self.variables[var_name]['VALUE']))
            return True
    
    '''
    def check_correctness(self, name):
        input_vals = []
        for var_name in self.correctnesses[name]['INPUT']:
            input_vals.append(self.variables[var_name]['VALUE'])
        return self.correctnesses[name]['FUNC'](input_vals)
    '''
        
    def apply_constrant(self, name):
        if self.variables[self.constrants[name]['OUT']]['VALUE'].defined_with_acc(self.stop_acc):
            if self.verbose:
                print("\t[{}] value {} defined by desired acc {}".format(self.constrants[name]['OUT'],
                                                                     self.variables[self.constrants[name]['OUT']]['VALUE'],
                                                                     self.stop_acc))
            return 0
                       
        input_vals = []
        for var_name in self.constrants[name]['INPUT']:
            input_vals.append(self.variables[var_name]['VALUE'])      
        
        if self.constrants[name]['PROC'] == 'DIRECT_SUBSTITUTION':
            updated_value = self.constrants[name]['FUNC'](input_vals)
        elif self.constrants[name]['PROC'] == 'ROV_MONTE_CARLO':
            updated_value = get_rov_monte_carlo_new(self.constrants[name]['FUNC'], input_vals, mc_rolls = self.max_mc_rolls)                
        else:
            raise ValueError('Unknown constrant procces type {}'.format(self.constrants[name]['PROC']))
        
        if len(updated_value.sd_vars) == 0:
            if self.verbose:
                print("\n*** NO SOLUTION FOUND ***\n")
            return -1
        
        if self.var_acc != -1:
            updated_value.around(self.var_acc)
        if self.variables[self.constrants[name]['OUT']]['TYPE'] == 'ANGLE':
            updated_value = updated_value.norm_angle()
        
        assigned_value = self.variables[self.constrants[name]['OUT']]['VALUE'].assign(updated_value)        
        if self.var_acc != -1:
            assigned_value.around(self.var_acc)
        
        if self.verbose:
            print('\t[{}: {} ] {} : {} --> {}'.format(name,self.constrants[name]['OUT'],
                                     self.variables[self.constrants[name]['OUT']]['VALUE'],
                                     updated_value,
                                     assigned_value))
        updated = 0
        if assigned_value.defined():        
            if self.var_acc != -1:
                self.variables[self.constrants[name]['OUT']]['VALUE'].around(self.var_acc)
                        
            approved = True
            if self.use_correctness_check:
                for n_c, f_c in self.correctnesses.items():
                    #print(self.constrants[name]['OUT'], f_c['INPUT'])
                    if self.constrants[name]['OUT'] in f_c['INPUT']:
                        '''
                        if not self.check_correctness(n_c):
                            approved = False
                            if verbose:
                                print("\t[correctness {} FAILED]".format(n_c))
                            #break
                            return -1
                        '''
                        #self.check_correctness_new(n_c)
                        #print("Checking {} correctness...".format(n_c))
                        if not self.check_correctness_new(n_c):
                            approved = False
                            #if self.verbose:
                            #print("\t[correctness {} FAILED]".format(n_c))
                            #break
                            return -1
                        #else:
                            #print("\rOK")
            
            if approved:                
                if (self.variables[self.constrants[name]['OUT']]['VALUE']).acc_eq(assigned_value, self.stop_acc):
                    updated = 0                    
                else:
                    updated = 1
                self.variables[self.constrants[name]['OUT']]['VALUE'] = assigned_value
                
        return updated
            
    def proc_complex(self):
        self.reset_stats()
        start_time_proc = time.time()
        for ans in self.proc_complex_yeild():
            pass
        end_time_proc = time.time()
        self.stats['duration']['proc_complex'] = end_time_proc - start_time_proc
        return ans
        
    def proc_complex_yeild(self):
        i = 0
        self.active_func = list(self.constrants.keys())
        while self.max_steps == 0 or i < self.max_steps:                        
            i+=1
            if len(self.active_func) == 0:
                if self.verbose:
                    print("Process end")
                break
            if self.verbose:
                print("Step {}".format(i))         
                print("\t{} | {}".format(self.active_func[0],self.active_func[1:]))            
            
            if self.active_func[0] in self.stats['functions']:
                self.stats['functions'][self.active_func[0]]['calls'] += 1
            else:
                self.stats['functions'][self.active_func[0]] = {}
                self.stats['functions'][self.active_func[0]]['calls'] = 1
                self.stats['functions'][self.active_func[0]]['updates'] = 0
            
            result = self.apply_constrant(self.active_func[0])
            if result == 1:
                
                self.stats['functions'][self.active_func[0]]['updates'] += 1                
            
                if self.verbose:
                    print("\tUpdated: yes")
                # add all functions to active list which is input with current output
                input_var = self.constrants[self.active_func[0]]['OUT']
                for constrant in self.constrants.values():
                    if not constrant['NAME'] in self.active_func and input_var in constrant['INPUT'] :
                        self.active_func.append(constrant['NAME'])
            elif result == 0:
                if self.verbose:
                    print("\tUpdated: no")
            elif result == -1:
                break
                
            del self.active_func[0]   
            
            yield self.variables                

        if self.verbose:
            print("+++SDM ANSWER+++")
            for var in self.variables.values():
                print("[{}] : {}".format(var['NAME'], var['VALUE']))        
            
        self.stats['duration']['steps'] = i
        yield self.variables                    

    def reset_stats(self):
        self.stats = {}
        self.stats['functions'] = {}
        self.stats['duration'] = {}
        
    def get_stats_str(self):
        st = []
        for f, s in self.stats['functions'].items():
            st.append("[ {} ]: {}/{} - {:.2f}%".format(f,s['updates'],s['calls'],100*s['updates']/s['calls']))
        return st
        
    def print_function_stats(self):
        for f, s in self.stats['functions'].items():
            print("[ {} ]: {}/{} - {:.2f}%".format(f,s['updates'],s['calls'],100*s['updates']/s['calls']))
        
def get_extremums_monte_carlo(func, domain, mc_rolls = 64):    
    # TODO
    pass

def get_rov_monte_carlo_new(func, domain, mc_rolls = 64):
    res = []
    for el in itertools.product(*domain):
        # el - array of sd_vars
        borders = []
        for v in el:
            borders.append(sd_mi([v]))
        res += func(borders).sd_vars
        for _ in range(mc_rolls):
            random_piece = []
            for v in el:
                a1 = np.random.uniform(v.low, v.high)
                a2 = np.random.uniform(v.low, v.high)
                a = sd_mi([sd_var(min(a1,a2),max(a1,a2))])
                random_piece.append(a)
            res += func(random_piece).sd_vars
    return sd_mi(res)

def get_rov_monte_carlo(func, domain, mc_rolls = 64):    
    # only one variable
    if len(domain) == 1:
        res = [] # list of sd_vars
        for var in domain[0].sd_vars:
            a = sd_mi([var])
            b = func([a])
            res += b.sd_vars
            for _ in range(mc_rolls):
                a1 = np.random.uniform(var.low, var.high)
                a2 = np.random.uniform(var.low, var.high)
                a = sd_mi([sd_var(min(a1,a2),max(a1,a2))])
                res += func([a]).sd_vars
        return sd_mi(res)
    else:
        res = [] # list of sd_vars
        for dom in domain:            
            dom.i = 0
        for dom in domain:            
            while True:#dom.switch_next():                
                sd_vars = [] # list of sd_vars, which is Cartesian product
                sd_mis = [] # same, but each std_var forms sd_mi
                for dom_ in domain: 
                    var = dom_.get_cur() # it is sd_var                                                            
                    sd_vars.append( var )
                    mi = sd_mi([var])
                    sd_mis.append(mi)
                    #print(var)
                # now we have the Cartesian product
                # here Monte-Carlo is                
                res += func(sd_mis).sd_vars
                for _ in range(mc_rolls):
                    A = []
                    for var in sd_vars:
                        a1 = np.random.uniform(var.low, var.high)
                        a2 = np.random.uniform(var.low, var.high)
                        a = sd_mi([sd_var(min(a1,a2),max(a1,a2))])
                        A.append(a)
                    res += func(A).sd_vars
                if not dom.switch_next():
                    break
            #print(res)
        return sd_mi(res)        

if __name__ == '__main__':
    # unit tests
    
    BASIC_EXAMPLE_1 = False # NOTE: depricated?
    BASIC_EXAMPLE_2 = False # NOTE: depricated?
    CIRCLES_1 = False
    CIRCLES_2 = False
    TEST_MONTE_CARLO = False
    TEST_MONTE_CARLO_2 = False
        
    # BASIC EXAMPLE 1
    '''
    source: https://www.computer-museum.ru/frgnhist/narinani_3.htm
    F1: y = x - 1
    F2: 4x + 3y = 12
    x = [0, 4]
    y = [0, 4]
    '''
    if BASIC_EXAMPLE_1:
        print("++BASIC EXAMPLE++")
        # init model
        basic_example = SDM(var_acc = 3)        
        basic_example.register_variable('x', sd_var(0, 4))
        basic_example.register_variable('y', sd_var(0, 4))
        # f1: y = x - 1
        def f1(var):
            return var[0] - 1            
        basic_example.register_constrant(name = 'f1',
                                         function = f1,
                                         out_var = 'y',
                                         input_vars = ['x'])
        # f2: x = (12 - 3y)/4
        def f2(var):
            return (12 - 3*var[0])/4
        basic_example.register_constrant(name = 'f2',
                                         function = f2,
                                         out_var = 'x',
                                         input_vars = ['y'])
        basic_example.proc_complex()
        
    # BASIC EXAMPLE 2
    '''
    source: https://www.computer-museum.ru/frgnhist/narinani_6.htm
    F1: x + y = 12
    F2: 2*x = y
    x = [0, 100]
    y = [0, 100]
    answer x,y = (4,8)
    '''    
    if BASIC_EXAMPLE_2:
        print("++BASIC EXAMPLE 2++")
        # init model
        basic_example2 = SDM(var_acc = 0)      
        basic_example2.register_variable('x', sd_var(0, 100))
        basic_example2.register_variable('y', sd_var(0, 100))
        # f1: x = 12 - y
        def f1(var):
            return 12 - var[0]
        basic_example2.register_constrant(name = 'f1',
                                         function = f1,
                                         out_var = 'x',
                                         input_vars = ['y'])
        # f2: y = 2x
        def f2(var):
            return 2*var[0]
        basic_example2.register_constrant(name = 'f2',
                                         function = f2,
                                         out_var = 'y',
                                         input_vars = ['x'])
        # f3: y = 12 - x
        def f3(var):
            return 12 - var[0]
        basic_example2.register_constrant(name = 'f3',
                                         function = f3,
                                         out_var = 'y',
                                         input_vars = ['x'])
        # f4: x = y/2
        def f4(var):
            return var[0]/2
        basic_example2.register_constrant(name = 'f4',
                                         function = f4,
                                         out_var = 'x',
                                         input_vars = ['y'])
        
        basic_example2.proc_complex()
    
    # TWO CIRCLES 1
    '''
    test for square equations, circles intersects in one point
    F1: (x-2)^2 + (y-2)^2 = 4
    F2: (x-3)^2 + (y-2)^2 = 1
    answer x,y = (4,2)
    '''
    if CIRCLES_1:
        print("++CIRCLES 1++")
        # init model
        circles1 = SDM(var_acc = 3)      
        circles1.register_variable('x', sd_var(0, 10))
        circles1.register_variable('y', sd_var(0, 10))
        # f1: x = sqrt(4 - (y-2)^2) + 2
        # f2: y = sqrt(4 - (x-2)^2) + 2
        # f3: x = sqrt(1 - (y-2)^2) + 3
        # f4: y = sqrt(1 - (x-3)^2) + 2
        def f1(var):
            return (4 - (var[0] - 2)**2).get_sqrt() + 2
        circles1.register_constrant(name="f1",
                                    function=f1,
                                    out_var='x',
                                    input_vars=['y'])
        circles1.register_constrant(name="f2",
                                    function=f1,
                                    out_var='y',
                                    input_vars=['x'])
        def f3(var):
            return (1 - (var[0] - 2)**2).get_sqrt() + 3
        circles1.register_constrant(name="f3",
                                    function=f3,
                                    out_var='x',
                                    input_vars=['y'])
        def f4(var):
            return (1 - (var[0] - 3)**2).get_sqrt() + 2
        circles1.register_constrant(name="f4",
                                    function=f4,
                                    out_var='y',
                                    input_vars=['x'])
        circles1.proc_till_defines()
    # TWO CIRCLES 2
    '''
    test for square equations, circles intersects in two points
    F1: (x-2)^2 + (y-2)^2 = 4
    F2: (x-4)^2 + (y-2)^2 = 1
    answer x,y = 
    '''
    if CIRCLES_2:
        print("++CIRCLES 2++")
        # init model
        circles2 = SDM(var_acc = 4)      
        circles2.register_variable('x', sd_mi([sd_var(0, 10)]))
        circles2.register_variable('y', sd_mi([sd_var(0, 10)]))
        # f1: x = sqrt(4 - (y-2)^2) + 2
        # f2: y = sqrt(4 - (x-2)^2) + 2
        # f3: x = sqrt(1 - (y-2)^2) + 4
        # f4: y = sqrt(1 - (x-4)^2) + 2        
        def f1(var):
            return (4 - (var[0] - 2)**2).get_sqrt() + 2
        circles2.register_constrant(name="f1",
                                    function=f1,
                                    out_var='x',
                                    input_vars=['y'],
                                    proc = 'ROV_MONTE_CARLO')
        circles2.register_constrant(name="f2",
                                    function=f1,
                                    out_var='y',
                                    input_vars=['x'],
                                    proc = 'ROV_MONTE_CARLO')
        def f3(var):
            return (1 - (var[0] - 2)**2).get_sqrt() + 4
        circles2.register_constrant(name="f3",
                                    function=f3,
                                    out_var='x',
                                    input_vars=['y'],
                                    proc = 'ROV_MONTE_CARLO')
        def f4(var):
            return (1 - (var[0] - 4)**2).get_sqrt() + 2
        circles2.register_constrant(name="f4",
                                    function=f4,
                                    out_var='y',
                                    input_vars=['x'],
                                    proc = 'ROV_MONTE_CARLO')        
        circles2.proc_complex()
    '''
    Test how Monte-Carlo works with circles
    X**2 + Y**2 = 1
    '''
    if TEST_MONTE_CARLO:
        print("++TEST MONTE CARLO++")
        # f1: *X = sqrt(1 - *Y^2)
        def func(var):
            return (1 - var[0]**2).get_sqrt()
        while(True):
            x1 = np.random.uniform(-1.2, 1.2)
            x2 = np.random.uniform(-1.2, 1.2)
            domain = [sd_mi([sd_var(min(x1,x2),max(x1,x2))])]
            
            rov = get_rov_monte_carlo(func, domain)
            
            print("{} --> {}".format(domain[0],rov))
            plt.cla()
            plt.xlim(-1.5,1.5)
            plt.ylim(-1.5,1.5)
            ax = plt.gca()
            ax.add_patch(mpatches.Circle((0,0), 1 , ec="k", fill=False, ls='--'))
            plt.plot([x1,x2],[-1,-1],'r-')
            for r in rov.sd_vars:
                plt.plot([-1,-1],[r.low,r.high],'b-')
            plt.grid()
            plt.pause(5)
    '''
    Test how Monte-Carlo works with circles
    *X^2 + *Y^2 = *R^2
    '''
    if TEST_MONTE_CARLO_2:
        print("++TEST MONTE CARLO 2++")
        # f1: *X = sqrt(*R - *Y^2)
        def func(var):
            return (var[1] - var[0]**2).get_sqrt()
        while(True):
            x1 = np.random.uniform(-1.2, 1.2)
            x2 = np.random.uniform(-1.2, 1.2)
            r_min = np.random.uniform(0, 1)
            domain = [sd_mi([sd_var(min(x1,x2),max(x1,x2))]), sd_mi([sd_var(r_min,1)])]
            
            rov = get_rov_monte_carlo(func, domain)
            
            print("{} --> {}".format(domain[0],rov))
            plt.cla()
            plt.xlim(-1.5,1.5)
            plt.ylim(-1.5,1.5)
            ax = plt.gca()
            ax.add_patch(mpatches.Circle((0,0), 1 , ec="k", fill=False, ls='--'))
            ax.add_patch(mpatches.Circle((0,0), r_min , ec="k", fill=False, ls='--'))
            plt.plot([x1,x2],[-1,-1],'r-')
            for r in rov.sd_vars:
                plt.plot([-1,-1],[r.low,r.high],'b-')
            plt.grid()
            plt.pause(5)

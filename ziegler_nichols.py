def ziegler_nichols(ku, Tu):

    pessen  = {'kp': 0.7,
               'ki': 1.75,
               'kd': 0.105}

    some_ov = {'kp': 1/3,
               'ki': 2/3,
               'kd': 1/9}

    no_ov =   {'kp': 0.20,
               'ki': 0.40,
               'kd': 2/30}

    PI =      {'kp': 0.45,
               'ki': 0.54,
               'kd': 0}

    P  =      {'kp': 0.5,
               'ki': 0,
               'kd': 0}

    def print_results(mode, mode_name, ku, Tu):
        print(f'{mode_name} mode: kp={mode["kp"]*ku:.2f}, kd={mode["kd"]*ku*Tu:.2f}, ki={mode["ki"]*ku/Tu:.2f}')
        return

    print("\n")
    print_results(pessen , 'Pessen Integral Rule', ku, Tu)
    print_results(some_ov, 'Some overshoot'      , ku, Tu)
    print_results(no_ov  , 'NO overshoot'        , ku, Tu)
    print_results(PI     , 'PI only (!!)'        , ku, Tu)
    print_results(P      , 'P only      '        , ku, Tu)
    print("\n")
    
    return

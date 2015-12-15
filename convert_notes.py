# notes are specified as (<note>|rest):duration
# duration must be an integer multiple of smallest_quant

import re

# estimated longest time between keys
# determined through experimentation: longest possible time between keys
smallest_quant = 2.2            # seconds

# sleep time before going down
# experimentally determined to be longest required time to stabilize before
# going to down position
sleep_time = .7                 # seconds

# array of dicts:
# 'type' => 'move-and-press' or 'keep' or 'off'
# 'num' => int, for 'move-and-press'

def parse_file(fname):
    with open(fname, 'r') as f:
        print('file:')
        print(fname)
        return parse_lines_of_file(f)

def parse_lines_of_file(f):
    res = []
    print('parsing file:')
    for x in f:
        print(x)
        if re.match('^\\s*$', x): continue
        elif re.search('^rest:',x):
            res.append({'type': 'off'})
        else:
            res.append({
                'type': 'move-and-press',
                'num': int(re.search('^([1-8])', x).group(1))
            })
        duration = int(re.search(':([0-9]+)$', x).group(1))
        for i in range(0, duration):
            res.append({'type': 'keep'})
    return res

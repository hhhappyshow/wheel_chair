"""Fine mouse encoder preserving the proven legacy report order and coarse steps.

Coordinates retain the legacy 25-HID-unit scale. Whole steps emit exactly the
same X-then-Y report sequence as legacy d/a followed by s/w; fractional tails
add fine precision without merging axes or inventing large pointer jumps.
"""
import math
HID_UNITS_PER_LEGACY_STEP=25
REPORT_STEP=25

def quantize_coordinate(value):
    if not math.isfinite(value):raise ValueError('Nonfinite coordinate')
    return round(value*25)/25

def displacement_reports(dx,dy):
    if not all(math.isfinite(v) for v in (dx,dy)):raise ValueError('Nonfinite displacement')
    raw=[round(dx*25),round(dy*25)]
    if sum(math.ceil(abs(v)/25) for v in raw)>128:raise ValueError('Displacement too large')
    reports=[]
    for axis,value in enumerate(raw):
        sign=1 if value>=0 else -1
        whole,remainder=divmod(abs(value),25)
        for step in [25]*whole+([remainder] if remainder else []):
            delta=[0,0];delta[axis]=sign*step;reports.append(tuple(delta))
    return reports

def encode_displacement(dx,dy):
    result=[]
    for x,y in displacement_reports(dx,dy):
        a=x&255;b=y&255
        result.append(f'~{a:02X}{b:02X}{a^b^0xA5:02X}\n'.encode('ascii'))
    return b''.join(result)

def is_axis_y_positive(fingerVector):
    if(fingerVector >= 0):
        rc = True
    else:
        rc = False
    return rc

def is_axis_x_positive(fingerVector):
    if(fingerVector >= 0):
        rc = True
    else:
        rc = False
    return rc

def is_finger_thumb(finger):
    if(finger >= 4):
        rc = True
    else:
        rc = False
    return rc

def is_less_than_0(angle):
    if(angle < 0):
        rc = True
    else:
        rc = False
    return rc

def is_greater_than_180(angle):
    if(angle > 180):
        rc = True
    else:
        rc = False
    return rc
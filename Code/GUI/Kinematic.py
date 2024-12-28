import math

def IK_T6(Px, Py, Pz, Solution):
    # Declare link lengths
    l0x = -165
    l0y = 0
    l1 = 75
    l2 = 210
    l3 = 220
    l4 = 22
    l5 = 20
    
    # Adjust Py to account for potential coordinate system convention
    Px = (Px - 0.0) * 10
    Py = (Py + 0.0)* 10
    Pz = (Pz + 0.8)* 10

    # Calculate Theta1 (q1) - Two possible solutions for q1
    q1 = math.degrees(math.atan2(Py - l0y, Px - l0x))
    q1_1 = q1 + 180

    # Calculate intermediate terms E and F
    E = Px * math.cos(math.radians(q1)) + Py * math.sin(math.radians(q1)) - l0x * math.cos(math.radians(q1)) - l0y * math.sin(math.radians(q1)) - l4
    E_1 = Px * math.cos(math.radians(q1_1)) + Py * math.sin(math.radians(q1_1)) - l0x * math.cos(math.radians(q1_1)) - l0y * math.sin(math.radians(q1_1)) - l4
    F = Pz - l1 + l5

    # Calculate Theta3 (q3) - Two possible solutions for q3
    G = E**2 + F**2 - l2**2 - l3**2
    c3 = G / (2 * l2 * l3)

    # Ensure c3 is within valid range to prevent complex results
    if c3 >= 1:
        s3_1 = 0
        s3_2 = 0
    else:
        s3_1 = math.sqrt(1 - c3**2)
        s3_2 = -math.sqrt(1 - c3**2)

    q3_1 = math.degrees(math.atan2(s3_1, c3))
    q3_2 = math.degrees(math.atan2(s3_2, c3))

    # Calculate Theta2 (q2) for each solution of q3
    # For q1 and q3_1 (elbow-up)
    c2_1 = (E * (l2 + l3 * math.cos(math.radians(q3_1))) + F * l3 * math.sin(math.radians(q3_1))) / ((l2 + l3 * math.cos(math.radians(q3_1)))**2 + (l3 * math.sin(math.radians(q3_1)))**2)
    s2_1 = (F * (l2 + l3 * math.cos(math.radians(q3_1))) - E * l3 * math.sin(math.radians(q3_1))) / ((l2 + l3 * math.cos(math.radians(q3_1)))**2 + (l3 * math.sin(math.radians(q3_1)))**2)
    q2_1 = math.degrees(math.atan2(s2_1, c2_1))

    # For q1 and q3_2 (elbow-down)
    c2_2 = (E * (l2 + l3 * math.cos(math.radians(q3_2))) + F * l3 * math.sin(math.radians(q3_2))) / ((l2 + l3 * math.cos(math.radians(q3_2)))**2 + (l3 * math.sin(math.radians(q3_2)))**2)
    s2_2 = (F * (l2 + l3 * math.cos(math.radians(q3_2))) - E * l3 * math.sin(math.radians(q3_2))) / ((l2 + l3 * math.cos(math.radians(q3_2)))**2 + (l3 * math.sin(math.radians(q3_2)))**2)
    q2_2 = math.degrees(math.atan2(s2_2, c2_2))

    # For q1_1 and q3_1 (elbow-up)
    c2_3 = (E_1 * (l2 + l3 * math.cos(math.radians(q3_1))) + F * l3 * math.sin(math.radians(q3_1))) / ((l2 + l3 * math.cos(math.radians(q3_1)))**2 + (l3 * math.sin(math.radians(q3_1)))**2)
    s2_3 = (F * (l2 + l3 * math.cos(math.radians(q3_1))) - E_1 * l3 * math.sin(math.radians(q3_1))) / ((l2 + l3 * math.cos(math.radians(q3_1)))**2 + (l3 * math.sin(math.radians(q3_1)))**2)
    q2_3 = math.degrees(math.atan2(s2_3, c2_3))

    # For q1_1 and q3_2 (elbow-down)
    c2_4 = (E_1 * (l2 + l3 * math.cos(math.radians(q3_2))) + F * l3 * math.sin(math.radians(q3_2))) / ((l2 + l3 * math.cos(math.radians(q3_2)))**2 + (l3 * math.sin(math.radians(q3_2)))**2)
    s2_4 = (F * (l2 + l3 * math.cos(math.radians(q3_2))) - E_1 * l3 * math.sin(math.radians(q3_2))) / ((l2 + l3 * math.cos(math.radians(q3_2)))**2 + (l3 * math.sin(math.radians(q3_2)))**2)
    q2_4 = math.degrees(math.atan2(s2_4, c2_4))

    # Solution Selection
    if Solution == 1:
        q1 = q1
        q2 = q2_1
        q3 = q3_1
    elif Solution == 2:
        q1 = q1
        q2 = q2_2
        q3 = q3_2
    elif Solution == 3:
        q1 = q1_1
        q2 = q2_3
        q3 = q3_1
    else:
        q1 = q1_1
        q2 = q2_4
        q3 = q3_2

    return q1, q2, q3

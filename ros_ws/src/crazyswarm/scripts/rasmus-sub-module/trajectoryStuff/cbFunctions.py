def heuristicFunction(distance):
    # distance: distance in meters
    # velocity: velocitvelocity in meters / second

    if distance < 0.45:
        velocity = 0
    elif 0.45 <= distance and distance < 1.2:
        velocity = 0.5
    elif 1.2 <= distance and distance < 3.6:
        velocity = 1.5
    elif 3.6 <= distance and distance < 7.6:
        velocity = 2.5
    else:
        velocity = 3

    return velocity
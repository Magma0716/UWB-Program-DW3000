distance_A1_A2 = 2.0
anchor_positions = {
    'A1': (0.0, 0.0),
    'A2': (distance_A1_A2, 0.0),
    'A3': (distance_A1_A2 / 2.0, distance_A1_A2),
}

print(anchor_positions['A1'][0])
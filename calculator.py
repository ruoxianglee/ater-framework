def calculate_reduced_frequency(
    f_s_max: list[float],
    varphi: int,
    Delta_t: int,
    # buffer_sizes: list[int],
    Nb: list[int],
    regulation_position: list[int] # 4 items
)->tuple[float, int]:
    numerators = []
    for index, value in enumerate(regulation_position):
        if value == 1:
            sum_of_buffer_msgs = 0
            for i in range(index + 1):
                sum_of_buffer_msgs += Nb[i]
            numerators.append((varphi * Delta_t * 1000) / f_s_max[index] - sum_of_buffer_msgs)

    minimum_messages = min(numerators)
    return (varphi * Delta_t * 1000) / minimum_messages, numerators.index(minimum_messages)

def calculate_increased_frequency(
    max_sub_period: list[float],
    sigma: int,
    Delta_t: int,
    Nb: list[int],
    regulation_position: list[int], # 4 items
    pre_position
)->float:
    numerators = []
    for index, value in enumerate(regulation_position):
        if value == 1:
            sum_of_buffer_msgs = 0
            for i in range(index + 1):
                sum_of_buffer_msgs += Nb[i]
            numerators.append(( sigma * Delta_t * 1000) / max_sub_period[index] - sum_of_buffer_msgs)

    minimum_messages = min(numerators)
    if pre_position != None:
        if pre_position == numerators.index(minimum_messages):
            return (sigma * Delta_t * 1000) / minimum_messages
        else:
            return 0.0
    else:
        return (sigma * Delta_t * 1000) / minimum_messages
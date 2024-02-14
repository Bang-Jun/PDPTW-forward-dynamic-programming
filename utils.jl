using JuMP, Gurobi
import Random

function generate_data(seed_value, num_requests)
    Random.seed!(seed_value)

    # num_requests & num_nodes
    num_nodes = 2 * num_requests + 2 # Includes two depot nodes

    # service times
    service_times = rand(1:3, 2*num_requests) 
    service_times = vcat(0, service_times, 0) # add depot service time 

    # locations
    locations = [rand(0:10, 2) for _ in 1:(2*num_requests + 1)] 
    locations = vcat(locations, [locations[1]]) # add end depot 

    # loads
    loads = [1 for _ in 1:num_requests] # pickup
    loads = vcat(loads, -loads) # add delivery
    loads = vcat(0, loads, 0) # add depot

    # depot
    depot_start = 1

    # generate_distance_matrix
    function generate_distance_matrix(locations)
        num_nodes = length(locations)
        
        travel_times = zeros(num_nodes, num_nodes)
        
        for i in 1:num_nodes
            for j in 1:num_nodes
                if i != j
                    dx = locations[i][1] - locations[j][1]
                    dy = locations[i][2] - locations[j][2]
                    travel_times[i, j] = round(sqrt(dx^2 + dy^2))
                end
            end
        end
        
        return travel_times
    end

    # travel_times
    travel_times = generate_distance_matrix(locations)
    
    # time windows
    time_windows = []
    pick_up = []
    delivery = []
    c = 10 * num_requests
    for i in 1:num_requests
        A_p = rand(0:c)
        A_d = A_p + service_times[i + num_requests + 1] + travel_times[i+1, i + num_requests + 1]
        push!(pick_up, (A_p, A_p + c/2))
        push!(delivery, (A_d, A_d + c/2))
    end
    time_windows = vcat((0, 0), pick_up, delivery, (0, 20*num_requests)) # add depot 

    return time_windows, travel_times, service_times, loads
end

# find visited nodes
function find_indices(set, a, num_nodes)
    indices = Int[]
    mask = bitstring(set)
    for (index, char) in enumerate(mask)
        if char == '1'
            push!(indices, length(mask) - index + 1)
        end
    end

    one_indices = reverse(indices)
    
    if a == 1
        return one_indices
    else
        zero_indices = [i for i in 1:num_nodes]
        zero_indices = setdiff(zero_indices, one_indices)
        return zero_indices
    end
end

# calculate current load
function cur_load(set, loads)
    indices = find_indices(set, 1, length(loads))
    total_load = 0
    for index in indices
        total_load += loads[index]
    end

    return total_load
end
    
# count number of nodes visited
function count_ones(set) 
    count = 0
    mask = bitstring(set)
    mask = string(mask)

    for char in mask
        if char == '1'
            count += 1
        end
    end
    return count
end

# partial ordering
function partial_order(vector)
    indices = Int[]
    for i in 1:length(vector)
        count = 0
        for j in 1:length(vector)
            if vector[j][1] <= vector[i][1] && vector[j][2] <= vector[i][2] && i != j && vector[j][1:2] != vector[i][1:2]
                count += 1
            end
        end
        if count == 0 
            push!(indices, i)
        end
    end
    new = [vector[index] for index in indices]
    new = unique(new)
    return new
end

# find trajectory
function find_trajectory(dict, preceding_label_num, num_nodes)
    last_label = preceding_label_num
    traj = []
    
    for i in 1:num_nodes
        for (key, value) in dict
            if value[1] == last_label
                push!(traj, (key[2]))
                last_label = value[2] # update label 
            end
        end
    end
    push!(traj, ((1)))
    traj = reverse(traj)
    return traj
end

# criteria 1

# criteria 2
function criteria_2(set, node, num_requests)
    if num_requests + 2 <= node <= 2*num_requests + 1 # delivery node
        if (set & (1 << (node - num_requests - 1))) != 0 # node visited
            return true
        else
            return false
        end
    elseif 2 <= node <= num_requests + 1 # pickup node
        return true
    else # depot
        if set == ((1 << (2*num_requests + 1) - 1))
            return true
        else
            return false
        end
    end
end

# criteria 3
function criteria_3(set, node, num_requests, loads, capacity)
    if 2 <= node <= num_requests + 1
        if cur_load(set, loads) + loads[node] <= capacity
            return true
        else
            return false
        end
    else
        return true
    end
end

# criteria 4 
function criteria_4(set, node, time_windows, travel_times, service_times)
    # Initialize minimum values
    min_value = Inf
    min_index = 0
    new_time_windows = [(time_windows[unvisited], unvisited) for unvisited in find_indices(set, 0, length(service_times)) if unvisited != node]
    # Iterate through the vector
    for time_window in new_time_windows
        second_element = time_window[1][2]
        if second_element < min_value
            min_value = second_element
            min_index = time_window[2]
        end
    end
    if min_index == 0
        return true
    else 
        if time_windows[node][1] + service_times[node] + travel_times[node, min_index] <= min_value
            return true
        else
            return false
        end
    end
end

# find t_cur_node, 이제 label의 list를 반환 
function t_node_test(dict, set, node, label)
    filtered_values = filter(kv -> kv[1][1:3] == (set, node, label), dict)

    if isempty(filtered_values)
        return 
    else
        time = getindex(first(filtered_values)[1][3], 1)
        return time
    end
end

# criteria 5
function criteria_5(dict, set, cur_node, next_node, label, time_windows, travel_times, service_times)
    if t_node_test(dict, set, cur_node, label) === nothing
        return false
    elseif t_node_test(dict, set, cur_node, label) + service_times[cur_node] + travel_times[cur_node, next_node] <= time_windows[next_node][2]
        return true
    else
        return false
    end
end

# criteria 6
function criteria_6(dict, set, cur_node, next_node, label, time_windows, travel_times, service_times)
    # Initialize minimum values
    min_value = Inf
    min_index = 0
    num_nodes = length(service_times)
    unvisited_indices = find_indices(set, 0, num_nodes)
    new_time_windows = [(time_windows[unvisited], unvisited) for unvisited in unvisited_indices if unvisited != next_node] # unvisited and not next_node
    # Iterate through the vector
    for time_window in new_time_windows
        second_element = time_window[1][2]
        if second_element < min_value
            min_value = second_element
            min_index = time_window[2]
        end
    end
    if min_index == 0
        return true
    else
        if t_node_test(dict, set, cur_node, label) === nothing
            return false
        else 
            t_next_node = max(time_windows[next_node][1], t_node_test(dict, set, cur_node, label) + service_times[cur_node] + travel_times[cur_node, next_node])
            if t_next_node + service_times[next_node] + travel_times[next_node, min_index] <= time_windows[min_index][2]
                return true
            else
                return false
            end
        end
    end
end

# criteria 7
function criteria_7(dict, set, cur_node, next_node, label, time_windows, travel_times, service_times, num_requests)
    min_value = Inf
    min_index = 0
    second_min_value = Inf
    second_min_index = 0
    unvisited_indices = find_indices(set, 0, length(service_times))
    new_time_windows = [(time_windows[unvisited], unvisited) for unvisited in unvisited_indices if unvisited ≠ next_node && num_requests + 2 <= unvisited <= 2*num_requests + 1] # unvisited pickup and not next_node
    for time_window in new_time_windows
        second_element = time_window[1][2]

        if second_element < min_value
            second_min_value = min_value
            second_min_index = min_index

            min_value = second_element
            min_index = time_window[2]
        elseif second_element < second_min_value
            second_min_value = second_element
            second_min_index = time_window[2]
        end
    end
    if min_index == 0 || second_min_index == 0 
        return true
    else
        if t_node_test(dict, set, cur_node, label) === nothing
            return false
        else
            t_next_node = max(time_windows[next_node][1], t_node_test(dict, set, cur_node, label) + service_times[cur_node] + travel_times[cur_node, next_node])
            if max(time_windows[min_index][1], t_next_node + service_times[next_node] + travel_times[next_node, min_index]) + service_times[min_index] + travel_times[min_index, second_min_index] <= time_windows[second_min_index][2] || max(time_windows[second_min_index][1], t_next_node + service_times[next_node] + travel_times[next_node, second_min_index]) + service_times[second_min_index] + travel_times[second_min_index, min_index] <= time_windows[min_index][2]
                return true
            else
                return false
            end
        end
    end
end

# criteria 8
function criteria_8(dict, set, cur_node, next_node, label, time_windows, travel_times, service_times, num_requests)
    min_value = Inf
    min_index = 0
    second_min_value = Inf
    second_min_index = 0
    num_nodes = length(service_times)
    unvisited_indices = find_indices(set, 0, num_nodes)
    new_time_windows = [(time_windows[unvisited], unvisited) for unvisited in unvisited_indices if unvisited ≠ next_node && 2 <= unvisited <= num_requests + 1 && criteria_2(set, unvisited, num_requests)] # unvisited delivery which its origin visited and not next_node
    for time_window in new_time_windows
        second_element = time_window[1][2]

        if second_element < min_value
            second_min_value = min_value
            second_min_index = min_index

            min_value = second_element
            min_index = time_window[2]
        elseif second_element < second_min_value
            second_min_value = second_element
            second_min_index = time_window[2]
        end
    end
    if min_index == 0 || second_min_index == 0 
        return true
    else
        if t_node_test(dict, set, cur_node, label) === nothing
            return false
        else 
            t_next_node = max(time_windows[next_node][1], t_node_test(dict, set, cur_node, label) + service_times[cur_node] + travel_times[cur_node, next_node])
            if max(time_windows[min_index][1], t_next_node + service_times[next_node] + travel_times[next_node, min_index]) + service_times[min_index] + travel_times[min_index, second_min_index] <= time_windows[second_min_index][2] || max(time_windows[second_min_index][1], t_next_node + service_times[next_node] + travel_times[next_node, second_min_index]) + service_times[second_min_index] + travel_times[second_min_index, min_index] <= time_windows[min_index][2]
                return true
            else
                return false
            end
        end
    end
end

# gurobi solver
function build_pdptw_model(time_windows, travel_times, service_times, capacity)
    model = Model(Gurobi.Optimizer)
    set_optimizer_attribute(model, "OutputFlag", 0)
    # num_requests & num_nodes
    num_nodes = size(time_windows, 1)
    num_requests = div(num_nodes - 2, 2)

    depot_start = 1
    depot_end = num_nodes
    
    # loads
    loads = [1 for _ in 1:num_requests] # pickup
    loads = vcat(loads, -loads) # add delivery
    loads = vcat(0, loads, 0) # add depot


    # pickup & delivery points 
    P = [i for i in 2:num_requests + 1]
    D = [i for i in num_requests + 2:2*num_requests + 1]

    # variables
    @variable(model, T[1:num_nodes] >= 0) # start time variable
    @variable(model, x[1:num_nodes, 1:num_nodes], Bin) # decision variable
    @variable(model, Q[1:num_nodes] >=0) # load variable 

    # objective function: minimize total travel time 
    @objective(model, Min, sum(travel_times .* x)) 
    
    # constraints
    # (6.18), (6.19)
    for i in P # for all i in P
        @constraint(model, sum(x[i, j] for j = 1:num_nodes) == 1)
        @constraint(model, (sum(x[i, j] for j = 1:num_nodes) - sum(x[num_requests+i, j] for j = 1:num_nodes)) == 0) 
    end
    
    # (6.20), (6.21), (6.22)
    @constraint(model, sum(x[1, j] for j in 1:num_nodes) == 1) # start depot
    for i in vcat(P, D)
        @constraint(model, (sum(x[i, j] for j = 1:num_nodes) - sum(x[j, i] for j = 1:num_nodes)) == 0)
    end
    @constraint(model, sum(x[j, num_nodes] for j in 1:num_nodes) == 1) # end depot
    
    # time window constraint (6.23)
    for i in 1:num_nodes
        for j in 1:num_nodes
            @constraint(model, T[j] >= (T[i] + service_times[i] + travel_times[i, j]) * x[i, j])
        end
    end
    
    # (6.27)
    for i in 1:num_nodes
        @constraint(model, time_windows[i][1] <= T[i])
        @constraint(model, time_windows[i][2] >= T[i])
    end

    # precedence constraint (6.25)
    for i in P
        @constraint(model, T[num_requests+i] - T[i] - travel_times[i, num_requests+i] - service_times[i] >= 0)
    end

    # load constraint 
    # (6.24)
    for i in 1:num_nodes
        for j in 1:num_nodes
            @constraint(model, Q[j] >= (loads[j] + Q[i]) * x[i,j]) 
        end
    end
    
    # (6.28)
    for i in 1:num_nodes
        @constraint(model, Q[i] <= min(capacity, capacity + loads[i]))
        @constraint(model, max(0, loads[i]) <= Q[i])
    end

    #return model
    # Solve the optimization problem
    optimize!(model)

    # Check if the optimization was successful and a solution was found
    return model
end
include("utils.jl")

# state: (set, current position, label(time, cost)), value: (label number, preceding label number)
function pdptw_dynamic_programming_forward(time_windows, travel_times, service_times, loads, capacity)
    memo = Dict{Tuple{Int, Int, Tuple{Float64, Float64}}, Tuple{Int, Int}}() # (set, current position, label(time, cost)), (label number, preceding label number)
    num_nodes = length(service_times)
    num_requests = div(num_nodes - 2, 2)
    label_num = 1
    memo[1, 1, (0, 0)] = (1, 0)
    for k in 1:num_nodes
        if k == 1 # first iteration
            for j in 2:num_requests+1
                if loads[1] + loads[j] <= capacity
                    label_num += 1
                    memo[1 | 1 << (j - 1), j, (max(time_windows[j][1], time_windows[1][1] + travel_times[1, j]), travel_times[1, j])] = (label_num, 0)
                end
            end
        elseif k == num_nodes # last iteration 
            ans = Inf
            last_label = 0
            final_labels = filter(kv -> kv[1][1:2] == ((1 << num_nodes) - 1, num_nodes), memo)
            for (key, value) in final_labels
                if ans > key[3][2]
                    ans = key[3][2]
                    last_label = value[2]
                end
            end
            trajectory = find_trajectory(memo, last_label, num_nodes)
            for (key, value) in final_labels
                if value[2] == last_label
                    push!(trajectory, (key[2]))
                end
            end
            return trajectory, ans
        else # 2 <= k <= num_nodes - 1 iteration 
            for mask in 1:(1 << (num_nodes - 1)) - 1
                if count_ones(mask) == k && string(bitstring(mask))[end] == '1' # for sets of size k
                    for j in 2:num_nodes
                        if (mask & (1 << (j - 1))) == 0  # if node j is not visited, criteria 1
                            if criteria_2(mask, j, num_requests) && criteria_3(mask, j, num_requests, loads, capacity) && criteria_4(mask, j, time_windows, travel_times, service_times)
                                for i in 2:num_nodes
                                    if (mask & (1 << (i - 1))) != 0 # if node i is visited
                                        filtered_values = filter(kv -> kv[1][1:2] == (mask, i), memo)
                                        candidates = Vector{Any}()
                                        for (key, value) in filtered_values
                                            label = key[3]
                                            if criteria_5(memo, mask, i, j, label, time_windows, travel_times, service_times) && criteria_6(memo, mask, i, j, label, time_windows, travel_times, service_times) && criteria_7(memo, mask, i, j, label, time_windows, travel_times, service_times, num_requests) && criteria_8(memo, mask, i, j, label, time_windows, travel_times, service_times, num_requests)
                                                preceding_label_num = memo[mask, i, (key[3][1], key[3][2])][1]
                                                push!(candidates, (max(time_windows[j][1], key[3][1] + service_times[i] + travel_times[i, j]), key[3][2] + travel_times[i, j], preceding_label_num)) # (time, cost, label number, preceding label number)
                                            end
                                        end
                                        if isempty(candidates)
                                            nothing
                                        else
                                            candidates = unique(candidates)
                                            filtered_labels = partial_order(candidates)
                                            for label in filtered_labels
                                                label_num += 1
                                                memo[mask | 1 << (j - 1), j, (label[1], label[2])] = (label_num, label[3])
                                            end
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end

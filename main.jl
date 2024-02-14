using JuMP
import Random

include("utils.jl")
include("pdptw_dp.jl")

function solve_pdptw(num_requests)
    capacity = 3
    seed_value = 1
    feasible_count = 0
    trajectory = nothing
    cost = nothing
    elapsed_time = nothing
    while feasible_count < 1
        seed_value += 1
        time_windows, travel_times, service_times, loads = generate_data(seed_value, num_requests)
        model = build_pdptw_model(time_windows, travel_times, service_times, capacity)
        if termination_status(model) == MOI.OPTIMAL || termination_status(model) == MOI.LOCALLY_SOLVED
            feasible_count += 1
            elapsed_time = @elapsed begin            
                trajectory, cost = pdptw_dynamic_programming_forward(time_windows, travel_times, service_times, loads, capacity) 
            end
        end
    end
    return trajectory, cost, elapsed_time
end


println(solve_pdptw(13))
println(solve_pdptw(14))
println(solve_pdptw(15))


# solve_pdptw(8) Elapsed time: 6.328393262 seconds
# solve_pdptw(9) Elapsed time: 6.854134493 seconds
# solve_pdptw(10) Elapsed time: 685.465225119 seconds
# solve_pdptw(11) Elapsed time: 3791.456047298 seconds
# solve_pdptw(12) Elapsed time: 50003.074374832 seconds

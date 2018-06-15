function record_data( sim_time, state, control )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    global Var;
    
    Var.Record.sim_time(Var.RecordId) = sim_time;
    Var.RecordId = Var.RecordId + 1;

end


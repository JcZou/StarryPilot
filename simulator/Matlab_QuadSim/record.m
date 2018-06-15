record_val = get(Var.record_hd, 'value');
if record_val
    % record data
    Record.sim_time(Record.id) = Var.sim_time;
    Record.state(Record.id) = QuadrotorState;
    Record.control(Record.id) = QuadrotorControl;
    Record.id = Record.id + 1;
end
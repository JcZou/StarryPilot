function new_rb = rb_push(rb, data)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

rb.buffer(:,rb.head) = data;
rb.head = rb.head + 1;
if rb.head > rb.size
   rb.head = 1; 
end

new_rb = rb;

end


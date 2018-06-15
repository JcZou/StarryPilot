function [new_rb, data] = rb_pop(rb)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

if rb.tail ~= rb.head
   data = rb.buffer(:,rb.tail);
   rb.tail = rb.tail + 1;
   if rb.tail > rb.size
      rb.tail = 1; 
   end
end

new_rb = rb;

end


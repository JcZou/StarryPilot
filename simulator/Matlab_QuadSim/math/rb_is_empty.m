function res = rb_is_empty(rb)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
if rb.tail == rb.head
    res = true;
else
    res = false;
end

end


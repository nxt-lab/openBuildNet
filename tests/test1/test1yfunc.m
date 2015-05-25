function test1yfunc(thenode, irupdate)
%Callback for UPDATE_Y of the main update for test1script.m.
%It prints out the current time and requests an irregular future update.
%irupdate is the index of the irregular update

t = thenode.currentSimTime;
fprintf('At %d UPDATE_Y for MAIN_UPDATE\n', t);

% randomly request future update
if rand() >= 0.5
    future = t + randi([11 16], 1);
    r = thenode.requestFutureUpdate(future, irupdate, 10);
    if r == 0
        fprintf('At %d Event request was accepted for time %d.\n', t, future);
    else
        fprintf('At %d Event request failed with error = %d.\n', t, r);
    end
end

end

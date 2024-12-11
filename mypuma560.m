%% Part 4.1

function myrobot = mypuma560(DH)
    for i = 1:size(DH,1)
        L(i) = Link(DH(i,:), 'standard');
    end
    myrobot = SerialLink(L,'name', 'mypuma560');
end
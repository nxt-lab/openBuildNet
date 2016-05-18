function controller_opt(node, mpccontroller)
global US
tic
U = mpccontroller{node.input('x')};
toc
uk = U(:,1);
node.output('u', uk);
US = [US, uk];
end
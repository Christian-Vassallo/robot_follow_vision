
function robotStateMSG(message)
global RBTSTATEMSG;
%disp([sprintf('Message received: '), message.getData()]);
%disp(message.getData());
%disp('trigger');
RBTSTATEMSG = message.getData();
end




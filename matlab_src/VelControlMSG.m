
function VelControlMSG(message)
global VelCONMSG;
%disp([sprintf('Message received: '), message.getData()]);
%disp(message.getData());
%disp('trigger');
VelCONMSG = message.getData();
end


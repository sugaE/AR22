function sim_robot(DH,q,jtype)
%% Robot simulation for Froward Kinematics (FK)
%Input: DH parameters as shown below, q is the robot configuration in the oint space, jtype conteins the joint's type (0 revolute, 1 prismatic)

%      | a_1 | alpha_1 | d_1 | theta_1 |
% DH = | ... | ....... | ... | ....... | 
%      | a_n | alpha_n | d_n | theta_n |
%
% with alpha_i and theta_i in radiant
links = Link.empty;
for i=(1:size(DH,1))
    if( jtype(i) == 0)
        links(i) = Revolute('a',DH(i,1),'alpha',DH(i,2),'d',DH(i,3));
    else
        links(i) = Prismatic('a',DH(i,1),'alpha',DH(i,2),'theta',DH(i,4),'qlim',[0 1]);
    end    
end

myrobot = SerialLink(links,'name', 'My beautiful robot');
clf;
plot(myrobot,q','jaxes');
box on;
[msg, id] = lastwarn;
warning('off', id)
end
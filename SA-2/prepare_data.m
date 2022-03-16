function [Q, E] = prepare_data(DH, jtype, q, q_des, dim, ind)

% dim: dimention of arm, 2 or 3
% ind: name of output image file

arguments
    DH
    jtype
    q
    q_des
    dim {mustBeInteger, mustBeInRange(dim, 2, 3)}
    ind
end

err_thred = 0.05;

[T] = FK(DH, jtype, q_des);
p_des = T(1:dim,4);               % desired end-effector configuration
R = T(1:3,1:3);
phi_des = atan2(R(2,1),R(1,1)); 
if dim == 2
    o_e = phi_des;
else
    theta_e = atan2(-R(3,1), sqrt(R(3,2)^2+R(3,3)^2));
    psi_e = atan2(R(3,2), R(3,3));
    o_e = [theta_e;psi_e;phi_des];
end


[Q, E] = IK(DH, jtype, q, p_des, o_e); 

fig1=figure();
plot(transpose(Q));
xlabel('num iter');
ylabel('q'); 
legend('q_1','q_2','q_3','q_4','q_5','q_6');
exportgraphics(fig1, fullfile("images/"+"q"+ind+"."+datestr(now,'DDHHMMSS')+".png"), BackgroundColor="none", Resolution=600);


fig2=figure();
plot(transpose(E));
xlabel('num iter');
ylabel('e');

% start error threshold
hold on; 
y = zeros(size(E, 2), 2);
y(:, 1) = y(:, 1) + err_thred;
y(:, 2) = y(:, 2) - err_thred;
plot(y, '--k');
hold off;
% end error threshold

if size(E, 1) == 3
%     2d 
    legend('e_x','e_y','e_{phi}', 'err_{threshold}');
else
%     3d
    legend('e_x','e_y','e_z','e_{phi}','e_{theta}','e_{psi}', 'err_{threshold}');
end
exportgraphics(fig2, fullfile("images/"+"e"+ind+"."+datestr(now,'DDHHMMSS')+".png"), BackgroundColor="none", Resolution=600);

 

%% Simulate the robot at each step of the iterative inverse kinematics 
% and observe how the end-effector reaches the desired pose

% figure("Name","EXAMPLE "+ind);
% for steps = 1:size(Q,2)
%     sim_robot(DH,Q(:,steps),jtype)
% end

end


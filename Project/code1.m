%%%%%%%% Semester task 2019 -2020 Robotics I
%%%%%%%% Motion Simulation of Robotic Manipulator with 3 rotational DOF%%%%%%%%%%%%

%%%%%%%% Name:Giannios Georgios Taxiarxis
%%%%%%%% AcademicId:031 16 156 


clear all; 
close all;
clc;


%%A.Task Space

%%Parameters for lengths of links 
l0 = 4;
l(1) = 7.0;  
l(2) = 3.0;  
l(3) = 3.0;

dt = 0.1;   %for sampling

Tf=10.0;    %10 sec will be the duration of AB movement	
t=0:dt:Tf;  
T=0:dt:2*Tf;%Period

%Declaration of A,B points 
%Be careful!! Start and final position must be IN Workspace
%Else atan2(,) function -> will not work properly


start_position = [0,0,13];
final_position = [3,4,13];

%Workspace limits 

%Check if A,B are in workspace

zmax = l0+sqrt(l(1)^2+(l(2)+l(3))^2);
xmax = l(2)+l(3);
ymax = sqrt(l(1)^2+(l(2)+l(3))^2);

A_is_in_workspace = 1;
B_is_in_workspace = 1;

if (start_position(1)> xmax | start_position(2)>ymax | start_position(3) > zmax)
    disp('A point is not in Workspace')
    A_is_in_workspace = 0;
    
end

if (final_position(1)> xmax | final_position(2)>ymax | final_position(3) > zmax)
    disp('B point is not in Workspace')
    B_is_in_workspace = 0;
end

%Check Singularity q3 =0 throught geometrical place
q3_is_pi = 0;
if(start_position(1)^2+(start_position(3)-l0)^2 ==l(1))
    disp('A point is not in Workspace,q3 = pi!!Singularity')
    q3_is_pi = 1;
end

if(final_position(1)^2+(final_position(3)-l0)^2 ==l(1))
    disp('B point is not in Workspace,q3 = pi')
    q3_is_pi = 1;
end
    

if ((and(A_is_in_workspace,B_is_in_workspace) == 1) &(q3_is_pi == 0))
N=3;    %Number of periodic movements
        %Watch for statements in the end
dt = 0.1;
Tf = 10.0;
t = 0:dt:Tf;
t_all=length(t);

%%%%%%%%%%Path planning using polynomial interpolation%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Go to Simulation.pdf for further explanation 


%%%%%%%%%Positon of end-effector%%%%%%%%%%%%%
    start_velocity = [0,0,0];
    final_velocity = [0,0,0];

    a0=start_position(1);
    b0=start_position(2);
    c0=start_position(3);

    a1=start_velocity(1);
    b1=start_velocity(2);
    c1=start_velocity(3);

    a2=(3/Tf^2)*(final_position(1)-start_position(1)) -(2/Tf)*start_velocity(1)- (1/Tf)*final_velocity(1);
    b2=(3/Tf^2)*(final_position(2)-start_position(2)) -(2/Tf)*start_velocity(2)- (1/Tf)*final_velocity(2);
    c2=(3/Tf^2)*(final_position(3)-start_position(3)) -(2/Tf)*start_velocity(3)- (1/Tf)*final_velocity(3);

    a3=(-2/Tf^3)*(final_position(1)-start_position(1)) + (1/Tf^2)*(start_velocity(1)+final_velocity(1));
    b3=(-2/Tf^3)*(final_position(2)-start_position(2)) + (1/Tf^2)*(start_velocity(2)+final_velocity(2));
    c3=(-2/Tf^3)*(final_position(3)-start_position(3)) + (1/Tf^2)*(start_velocity(3)+final_velocity(3));

    t_final = length(t);
    %%cordinates of position of end - effector
    for i=1:t_final  
        xd(i)=a0 + a1*t(i) + a2*t(i)^2 + a3*t(i)^3;
        yd(i)=b0 + b1*t(i) + b2*t(i)^2 + b3*t(i)^3;
        zd(i)=c0 + c1*t(i) + c2*t(i)^2 + c3*t(i)^3;
    end
 
    
%for k=2:kmax;    
%xd(k) = xd(k-1) + lambda_x*dt;    
%yd(k) = yd(k-1) + lambda_y*dt; 
%end  


%corresponding movement from B to A
xd_ba = zeros(1,length(t)-1);
yd_ba = zeros(1,length(t)-1);
zd_ba = zeros(1,length(t)-1);

for i=1:(t_final-1) 
    xd_ba(i)=xd(t_all-i);
    yd_ba(i)=yd(t_all-i);
    zd_ba(i)=zd(t_all-i);    
end
%Now if we concat these arrays we take the profile of motion in one Period
xd_one_period = cat(2,xd,xd_ba);
yd_one_period = cat(2,yd,yd_ba);
zd_one_period = cat(2,zd,zd_ba);

 
figure;

plot(T,xd_one_period);
ylabel('pe_x (cm)'); 
xlabel('time t (sec)');
title('Κυματομορφή pe_x συναρτήσει του χρόνου σε μια περίοδο');

figure;
plot(T,yd_one_period)
ylabel('pe_y (cm)'); 
xlabel('time t (sec)');
title('Κυματομορφή pe_y συναρτήσει του χρόνου σε μια περίοδο ');


figure;
plot(T,zd_one_period)
ylabel('pe_z (cm)'); 
xlabel('time t (sec)');
title('Κυματομορφή pe_z συναρτήσει του χρόνου σε μια περίοδο');


%%%%%%%%%Velocity of end-effector%%%%%%%%%%%%%
x=[xd(:) yd(:) zd(:)];  

%%cordinates of velocity of end-effector stored by columns(1->x,2->y,3_z)
for i=2:length(t)
    dx(i,:) = (x(i,:) - x(i-1,:))/dt;
end


fig2=figure;
plot(t,dx(:,1)); 
ylabel('Vx (cm/sec)'); 
xlabel('time t (sec)');  
title('Κυματομορφή ταχύτητας pe_x συναρτήσει του χρόνου');

figure;
plot(t,dx(:,2));  
ylabel('Vy (cm/sec)'); 
xlabel('time t (sec)');   
title('Κυματομορφή ταχύτητας  pe_y συναρτήσει του χρόνου');

figure;
plot(t,dx(:,3)); 
ylabel('Vz (cm/sec)'); 
xlabel('time t (sec)');  
title('Κυματομορφή ταχύτητας pe_z συναρτήσει του χρόνου');

syms pe_x pe_y pe_z 
syms q1 q2 q3

%%%%%%%%%%PART_B:inverse Kinematics Model%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Go to Simulation.pdf for further explanation 


syms pe_x pe_y pe_z 
syms q1 q2 q3
%Cordinates of end effector expressed in Oo System (Initial)
P=[pe_x;pe_y;pe_z];

q1 =2*atan2(-pe_x+sqrt(pe_x^2+(pe_z-l0)^2-l(1)^2),l(1)+pe_z-l0);
q3 = acos( ((-sin(q1)*l0+cos(q1)*pe_x+sin(q1)*pe_z) ^2 + pe_y^2 -l(2)^2 -l(3)^2)/(2*l(2)*l(3))); 
q2 = atan2(pe_y, -sin(q1)*l0 +cos(q1)*pe_x+sin(q1)*pe_z) - asin( l(3)*sin(q3) / (sqrt(pe_y^2 + (-sin(q1)*l0+cos(q1)*pe_x+sin(q1)*pe_z)^2)));



for i=1:length(t)
    %Coordinates of End-Effector expressed in O
    
    pe_x=xd(i);
    pe_y=yd(i);
    pe_z=zd(i);
    
    q_1(i) =2*atan2(-pe_x+sqrt(pe_x^2+(pe_z-l0)^2-l(1)^2),l(1)+pe_z-l0)
    q_3(i) = acos(((-sin(q_1(i))*l0+cos(q_1(i))*pe_x+sin(q_1(i))*pe_z)^2 + (pe_y)^2 -l(2)^2 -l(3)^2)/(2*l(2)*l(3))); 
    q_2(i) = atan2(pe_y,(-sin(q_1(i))*l0+cos(q_1(i))*pe_x+sin(q_1(i))*pe_z)) - asin(l(3)*sin(q_3(i))/sqrt(((-sin(q_1(i))*l0+cos(q_1(i))*pe_x+sin(q_1(i))*pe_z))^2 +(pe_y)^2));

end

fig3=figure; 
subplot(1,3,1); 
plot(t,q_1(:)); 
ylabel('q1 (rad)'); 
xlabel('time t (sec)');  

subplot(1,3,2); 
plot(t,q_2(:)); 
ylabel('q2 (rad)'); 
xlabel('time t (sec)');   
title('Γωνίες Στροφής κάθε χρονική στιγμη');

subplot(1,3,3); 
plot(t,q_3(:)); 
ylabel('q3 (rad)'); 
xlabel('time t (sec)'); 




%Speed of robot joints
q=[q_1(:) q_2(:) q_3(:)];

for i=2:length(t)
    dq(i,:) = (q(i,:) - q(i-1,:))/dt;
end
fig4 = figure; 
subplot(1,3,1); 
plot(t,dq(:,1));
ylabel('Ταχύτητα q1 (rad/sec)'); 
xlabel('time t (sec)'); 

subplot(1,3,2);
plot(t,dq(:,2));
ylabel('Ταχύτητα q2 (rad/sec)'); 
xlabel('time t (sec)');   
title('Ταχύτητες Γωνιών Στροφής κάθε χρονική στιγμη');

subplot(1,3,3);
plot(t,dq(:,3));
ylabel('Ταχύτητα q3 (rad/sec)'); 
xlabel('time t (sec)');  

%%%%%%%%%%PART_C:Animation%%%%%%%%%%%%%%%%%%
%%%%%%%%%%Go to Simulation.pdf for further explanation 


%%%%%%%%%%In this stage we find the postion of every joint
%%%%%%%%%%using the 4th column of arrays A

for ti=1:t_all
    
    %4th column of array A_from_0_to_0'
    xq1(ti) = 0;  
    yq1(ti) = 0; 
    zq1(ti) = l0;
    
    %4th column of array A_from_0_to_1
    xq2(ti) =-l(1).*sin(q_1(ti));   
    yq2(ti) =0;  
    zq2(ti) =l(1).*cos(q_1(ti))+l0;
    
    %4th column of array A_from_0_to_S(auxiliary)
    xq3(ti) =l(2).*cos(q_1(ti)).*cos(q_2(ti)) -l(1).*sin(q_1(ti));   
    yq3(ti) = l(2).*sin(q_2(ti));    
    zq3(ti) =l(2).*sin(q_1(ti)).*cos(q_2(ti))+l(1).*cos(q_1(ti))+l0;
    
    %4th column of array A_from_0_to_E
    xde(ti) = l(3)*cos(q_1(ti)).*cos(q_2(ti)+q_3(ti))+l(2)*cos(q_1(ti)).*cos(q_2(ti))-l(1)*sin(q_1(ti));
    yde(ti) = l(3)*sin(q_2(ti)+q_3(ti))+l(2)*sin(q_2(ti)); 
    zde(ti) = l(3)*sin(q_1(ti)).*cos(q_2(ti)+q_3(ti))+l(2)*sin(q_1(ti)).*cos(q_2(ti))+l(1)*cos(q_1(ti))+l0;
  
end



fig2 = figure; 
axis([-20 20 -20 20 0 20])
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel ('z (cm)');
plot3(xd,yd,zd,'rs'); 
dtk=20;


%%Plot animation such as sample_script.m 
for i=1:N
    for ti=1:dtk:t_all  
           pause(0.2);
           plot3([0,xq1],[0,yq1], [0,zq1]);
           plot3([xq1],[yq1],[zq1],'o');  
           plot3([xq1(ti),xq2(ti)],[xq1(ti),yq2(ti)], [zq1(ti),zq2(ti)]);
           plot3([xq2(ti)],[yq2(ti)], [zq2(ti)],'o');  
           plot3([xq2(ti),xq3(ti)],[yq2(ti),yq3(ti)],[zq2(ti),zq3(ti)]);	
           plot3([xq3(ti)],[yq3(ti)],[zq3(ti)],'y*');  
           plot3([xq3(ti),xde(ti)],[yq3(ti),yde(ti)],[zq3(ti),zde(ti)]);
           plot3([xde(ti)],[yde(ti)],[zde(ti)],'g+');  
      end 
     for ti=t_all:-dtk:1
           pause(0.2);	
           plot3([0,xq1],[0,yq1], [0,zq1]);
           plot3([xq1],[yq1],[zq1],'o');   
           plot3([xq1(ti),xq2(ti)],[xq1(ti),yq2(ti)], [zq1(ti),zq2(ti)]);	
           plot3([xq2(ti)],[yq2(ti)], [zq2(ti)],'o');   
           plot3([xq2(ti),xq3(ti)],[yq2(ti),yq3(ti)],[zq2(ti),zq3(ti)]);	
           plot3([xq3(ti)],[yq3(ti)],[zq3(ti)],'y*');  
           plot3([xq3(ti),xde(ti)],[yq3(ti),yde(ti)],[zq3(ti),zde(ti)]);
           plot3([xde(ti)],[yde(ti)],[zde(ti)],'g+');    
     end       
end
end

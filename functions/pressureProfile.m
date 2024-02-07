function [P, robot_loads, robot_load] = pressureProfile(step,p_max,p_fak,roundF,gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext)
k = 0;
for p = (0:step:p_max-(roundF+1)*step)%150000 
    k = k+1;
    pressure = p_fak.*[p; p; p]; %p/3 y
%     pressure2 = p_fak(:,2).*[p; p; p];
    P(:,k) = pressure;
    robot_load = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);
    robot_loads{k} = robot_load;
%     if mod(p,10000)==0
%         for j=1:100
%             k=k+1;
%             pressure = [1*p; 0*p; 0*p]; %p/3 y
%             P(:,k) = pressure;
%             robot_load = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);
%             robot_loads{k} = robot_load;
%         end
%     end
end
p = tipFcn(p_max-roundF*step,k+1,k+1+roundF,step);
for i=1:length(p)
    k=k+1;
    pressure = p_fak.*[p(i); p(i); p(i)]; %p/3 y
%     pressure2 = p_fak(:,2).*[p(i); p(i); p(i)];
    P(:,k) = pressure;
    robot_load = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);
    robot_loads{k} = robot_load;
end

for p = (p_max-(roundF+1)*step:-step:0)%150000 
    k = k+1;
    pressure = p_fak.*[p; p; p]; %p/3 y
%     pressure2 = p_fak(:,2).*[p; p; p];
    P(:,k) = pressure;
    robot_load = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);
    robot_loads{k} = robot_load;
%     if mod(p,10000)==0
%         for j=1:100
%             k=k+1;
%             pressure = [1*p; 0*p; 0*p]; %p/3 y
%             P(:,k) = pressure;
%             robot_load = Load(pressure, gravity, tip_mass, n_ext, m_ext, s_n_ext, s_m_ext);
%             robot_loads{k} = robot_load;
%         end
%     end
end
end

function p = tipFcn(pStart,iStart,iTip,step)
    iEnd = iTip+(iTip-iStart);
    B = (iEnd-iStart)/(iStart^2-iEnd^2);
    a = step * B/(1+2*iStart*B);
    b = step-2*a*iStart;
    c = pStart-a*iStart^2-b*iStart;
    kk = 0;
    for iFunc = iStart:iEnd
        kk=kk+1;
        p(kk) = a*iFunc^2+b*iFunc+c;
    end
end
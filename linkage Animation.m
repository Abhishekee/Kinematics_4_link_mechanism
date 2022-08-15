clear all
close all

LO = 7;
L1 = 3;
L2 = 7.5;
L3 = 4.5;

L_PA = 4;
alpha = 35;

w2 = 5;
theta2 = 0:2:360;

for i = 1:length(theta2)
  AC(i) = sqrt(L0^2 + L1^2 - 2*L0*L1*cosd(theta2(i)));
  beta(i) = acosd((L0^2 + AC(i)^2 - L1^2) / (2*L0(i)*AC(i)));
  psi(i) = acosd((L2^2 + AC(i)^2 - L3^2) / (2*L2*AC(i)));
  lambda(i) = acosd((L3^2 + AC(i)^2 - L2^2) / (2*L3*AC(i)));
  
  theta3(i) = psi(i) - beta(i);
  theta4(i) = 180 - lambda(i) -  beta(i);
  
    
  if theta2(i) > 180
      theta3(i) = psi(i) + beta(i);
      theta4(i) = 180 - lambda(i) + beta(i);
  end
  
  %Define joint position
  Ox(i) = 0;
  Oy(i) = 0;
  
  Ax(i) = Ox(i) + L1*cosd(theta2(i));
  Ay(i) = Oy(i) + L1*sind(theta2(i));
  
  Bx(i) = Ox + Ax(i) + L2*cosd(theta3(i));
  By(i) = Oy + Ay(i) + L2*sind(theta3(i));
  
  Cx(i) = L0;
  Cy(i) = 0;
  
  Px(i) = Ax(i) + L_AP*cosd(alpha + theta3(i));
  PY(i) = AY(i) + L_AP*sind(alpha + theta3(i));
  
  plot( [Ox(i) Ax(i)], [Oy(i) Ay(i);[Ax(i) Bx(i)], [Ay(i) By(i)], ...
        [Bx(i) Cx(i)], [By(i) Cy(i)], 'LineWidth', 3 )
        
     hold on;   
    grid on;
    axis equal
    axis ( [-5 15 -5 10]);
    drawnow;
    hold off;
        
end

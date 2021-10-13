classdef mpf
    methods(Static)
        %q: matriz (MxN), entonces M (Mx1) es un vector de índices de manipulabilidad para cada configuración de las articulaciones
       
        function [m,md] = DinamicManipulabilty(robot, q)
            dof= logical([1 1 1 1 1 1]);
            
            J = robot.jacob0(q);
            J = J(dof,:);
            m2 = det(J * J');
            m2 = max(0, m2); 
            m = sqrt(m2);%Manipulability
            
            %Dynamic manipulability
            MI= robot.inertia(q);
            md1=det(J * inv(MI'*MI)*J');
            md2=max(0, md1); 
            md=sqrt(md2);
        end
        function InertiaData = LoadInertiaData
            % Use this function to calculate inertia data in standard DH notation
            % Output: InertiaData = [Ixx, Iyy, Izz, Ixy, Iyz, Ixz]
            data = mpf.GetInertiaData;

            % Unfortunately, the above data was generated for link coordinate systems
            % that are not aligned with the DH coordinate systems. Therefore, rotation
            % matricies will be defined for each link that will transform the data
            % above to the DH coordinate systems.
            T(:,:,1) = eye(3); 
            T(:,:,2) = eye(3);  
            T(:,:,3) = rotx(-pi/2); 
            T(:,:,4) = eye(3);  
            T(:,:,5) = rotx(pi/2)*rotz(pi);  
            T(:,:,6) = rotx(pi/2)*rotz(pi/2);  

            InertiaData = zeros(6,6);
            for n = 1:6
                I = [data(n,1), data(n,2), data(n,3); data(n,2), data(n,4), data(n,5); ...
                     data(n,3), data(n,5), data(n,6)];
                I = T(1:3,1:3,n) * I * T(1:3,1:3,n)';
                InertiaData(n,:) = [I(1,1),I(2,2),I(3,3),I(1,2),I(2,3),I(1,3)]; 
            end

        end 
       

        function data = GetInertiaData
        % components:
        %   Link(i) = [ Ixx, Ixy, Ixz, Iyy, Iyz, Izz ]
        data = [0.001	0	0	0	0.001	0.0001; 
                0.001	0	0	0	0.001	0.0001; 
                0.001	0	0	0	0.001	0.0001; 
                0.001	0	0	0	0.001	0.0001;
                0.001	0	0	0	0.001	0.0001; 
                0.001	0	0	0	0.001	0.0001];
        end

        
        
        
          %Ángulo formado por los dos vectores dado por tres puntos
          function a = anglePoints3D( p1,  p2,  p3)
                 v1=[p1.X-p2.X p1.Y-p2.Y p1.Z-p2.Z];
                 v2=[p3.X-p2.X p3.Y-p2.Y p3.Z-p2.Z];
                 a=acosd(dot(v1,v2)/(norm(v1)*norm(v2)));
          end
          %Cálculo de la rotación del hombro o codo
          function rotG = angleRotation(pShoulder, pElbow, pControlRotation, pGiro)
                angPlaneY= mpf.anglePointPlane(pShoulder, pElbow, 'Y');
                if angPlaneY<90 %Brazo en dirección frontal
                    rotG=mpf.anglePointPlane(pGiro, pControlRotation,'Z');
                else
                    rotG=mpf.anglePointPlane(pGiro, pControlRotation,'X');
                end 
          end
            
          %interpolación de los límites de articulaciones del robot y la
          %cámara
          function gR= interp(gK, nQ)
                q1=struct('robot',{[80,-110]},'kinect',{[0,180]});  
                q2=struct('robot',{[-180,20]},'kinect',{[0,180]}); 
                q3=struct('robot',{[-70,140]},'kinect',{[0,180]});
                q4=struct('robot',{[10,180]},'kinect',{[0,170]});  
                q5=struct('robot',{[-130,60]},'kinect',{[0,180]});
                q6=struct('robot',{[-120,40]},'kinect',{[90,270]});
                s=struct('q1', q1, 'q2', q2, 'q3',q3, 'q4',q4, 'q5', q5, 'q6', q6);
                fns = fieldnames(s);
                infoRobot=s.(fns{nQ}).robot;
                infoKinect=s.(fns{nQ}).kinect;
                gR=(gK-infoKinect(1))/(infoKinect(2)-infoKinect(1))*(infoRobot(2)-infoRobot(1))+infoRobot(1);
          end
          
          %Longitudes de los links
          function length = euDist(p0,p1)
              vA=[p0.X p0.Y p0.Z];
              vB=[p1.X p1.Y p1.Z];
              length= norm(vA - vB);
          end
          
          %Cálculo del ángulo formado por dos puntos proyectado en un plano
           function ang= anglePointPlane(p0, p1, plane)
                if (plane=='Y') 
                    v1X = p0.X;
                    v1Y = p0.Y;
                    v1Z = p0.Z-1;

                    p2X = p1.X;
                    p2Y = p0.Y;
                    p2Z = p1.Z;
                end 

                if (plane=='Z')
                    v1X = p0.X;
                    v1Y = p0.Y-1;
                    v1Z = p0.Z;

                    p2X = p1.X;
                    p2Y = p1.Y;
                    p2Z = p0.Z;
                end 

                if (plane=='X')
                    v1X = p0.X;
                    v1Y = p0.Y-1;
                    v1Z = p0.Z;

                    p2X = p0.X;
                    p2Y = p1.Y;
                    p2Z = p1.Z;
                end 
                 v1=[p2X-p0.X p2Y-p0.Y p2Z-p0.Z];
                 v2=[v1X-p0.X v1Y-p0.Y v1Z-p0.Z];
                 ang=acosd(dot(v1,v2)/(norm(v1)*norm(v2)));
           end 
          
    end
end

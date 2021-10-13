cd 'C:\Users\oscar\OneDrive - Universidad Nacional Autónoma de Honduras\f_Docturado FC-UA\Desarrollo Doctorado\Proyectos\EMG-Kinect\Matlab\Pruebas CSV y MAT\Pruebas CSV y MAT'
S = dir('**/*.csv');
for i=1:length(S)
%for i=1:3
    path=S(i).folder;
    folderSplit= regexp(S(i).folder,'\','split');
    nameFolder= folderSplit(length(folderSplit))
    nameNewFile=strrep(nameFolder,'.','_')
    fileName=S(i).name;
    fullPath=strcat(path,'\',fileName)
    
    CSV = readtable(fullPath,'delimiter',';'); %prueba 1
    heightCSV=height(CSV);
    matrizOut=zeros(heightCSV,6);
    matrizOutGrados=zeros(heightCSV,7);
    matrizLengthLinks=zeros(heightCSV,3);
    
    cd 'C:\Users\oscar\OneDrive - Universidad Nacional Autónoma de Honduras\f_Docturado FC-UA\Desarrollo Doctorado\Proyectos\EMG-Kinect\Matlab';
    
    for row = 1:heightCSV
         jDataBody=string(CSV{row,10});
         jsonBody=jsondecode(jDataBody);
         joints=jsonBody.Joints;
         %obtener joints necesarios para cálculo de ángulos
         ShoulderRight=joints.ShoulderRight.Position;
         ElbowRight=joints.ElbowRight.Position;
         WristRight=joints.WristRight.Position;
         HandRight=joints.HandRight.Position;
         ThumbRight=joints.ThumbRight.Position;
         HandTipRight=joints.HandTipRight.Position;

        %calcular los ángulos (proyección, interpolación)
        q1=mpf.interp(mpf.anglePointPlane(ShoulderRight,WristRight,'Y'),1);
        q2=mpf.interp(mpf.anglePointPlane(ShoulderRight,WristRight,'Z'),2);
        q3=mpf.interp(mpf.angleRotation(ShoulderRight, ElbowRight, WristRight, ElbowRight),3);
        q4=mpf.interp(mpf.anglePoints3D(ShoulderRight,ElbowRight,WristRight),4);
        q5=mpf.interp(mpf.angleRotation(ShoulderRight, ElbowRight,ThumbRight , WristRight),5);
        q6=mpf.interp(mpf.anglePoints3D(HandTipRight,WristRight,ElbowRight),6);

        %vector
        qR = [ deg2rad(q1) deg2rad(q2) deg2rad(q3) deg2rad(q4) deg2rad(q5) deg2rad(q6) ];

        %agregar elemento a matriz
        matrizOut(row, :)= qR;

        %información adicional, para efectos de validación: ángulos en grados
        %sin interpolación, para verificar información de la cámara
        q1K= mpf.anglePointPlane(ShoulderRight,WristRight,'Y');
        q2K= mpf.anglePointPlane(ShoulderRight,WristRight,'Z');
        q3K= mpf.angleRotation(ShoulderRight, ElbowRight, WristRight, ElbowRight);
        q4K= mpf.anglePoints3D(ShoulderRight,ElbowRight,WristRight);
        q5K= mpf.angleRotation(ShoulderRight, ElbowRight,ThumbRight , WristRight); 
        q6K= mpf.anglePoints3D(HandTipRight,WristRight,ElbowRight);
        time= CSV{row,1};
        qGrados = [time q1K q2K q3K q4K q5K q6K ];
        matrizOutGrados(row, :)= qGrados;

        rl1=mpf.euDist(ShoulderRight,ElbowRight);
        rl2=mpf.euDist(ElbowRight,WristRight);
        rl3=mpf.euDist(WristRight,HandRight);%HandTipRight);

        matrizLengthLinks(row, :) =[rl1 rl2 rl3 ];
    end  

    %filtro
    matrizOutSmooth=smoothdata(matrizOut,'sgolay');
    qFinal=matrizOutSmooth;

    %tamaños links
    L1=mean(matrizLengthLinks(:,1));
    L2=mean(matrizLengthLinks(:,2));
    L3=mean(matrizLengthLinks(:,3));


    sLinks= zeros(1,6);
    sLinks(3)=L1;
    sLinks(5)=L2;
    sLinks(6)=L3;


    % Crear link utilizando parámetros D-H    
    %           [THETA          D       A       ALPHA   SIGMA]
    L(1) = Link([ 0           0         0           pi/2        0] );
    L(2) = Link([ 0           0         0           pi/2        0] );
    L(3) = Link([ 0           -L1       0           pi/2        0] );  %-0.18
    L(4) = Link([ 0           0         0          -pi/2        0] );
    L(5) = Link([ 0           L2        0          -pi/2        0] ); %0.21
    L(6) = Link([ 0           0         L3         0           0] ); %0.05


    % Límites joints
    L(1).qlim=[deg2rad(-110) deg2rad(80)];
    L(2).qlim=[deg2rad(-180) deg2rad(20)];
    L(3).qlim=[deg2rad(-70) deg2rad(70)];
    L(4).qlim=[deg2rad(10) deg2rad(180)];
    L(5).qlim=[deg2rad(-130) deg2rad(60)];
    L(6).qlim=[deg2rad(-120) deg2rad(40)];
    rob = SerialLink(L, 'name','rob');


    %Dinámica
    rdata = zeros (6,3);
    rdata(1,:) = [0,0,0];
    rdata(2,:) = [0,0,0];
    rdata(3,:) = [0,-L1/2,0];    
    rdata(4,:) = [0,0,0];
    rdata(5,:) = [0,-L2/2,0];   
    rdata(6,:) = [0,L3/2,0];  

    IData = mpf.LoadInertiaData; % Intoducimos en IData los valores de Inercia


    % mano 0.006
    % brazo inf derecho 0.016
    % brazo sup derecho 0.027
    mass = [0, 0, 70*0.027, 0, 70*0.016, 70*0.006];   % masas de los eslabones
    Ixx = IData(:,1)';   % Valores de las matrices de inercia
    Iyy = IData(:,2)';
    Izz = IData(:,3)';
    Ixy = IData(:,4)';
    Iyz = IData(:,5)';
    Ixz = IData(:,6)';
    Jm = zeros(1,6);        % Inercia del motor

    for i=1:6
        rob.links(i).m = mass(i);
        rob.links(i).r = rdata(i,:);
        rob.links(i).I = [Ixx(1,i) Ixy(1,i) Ixz(1,i);
                             Ixy(1,i) Iyy(1,i) Iyz(1,i);
                               Ixz(1,i) Iyz(1,i) Izz(1,i)];
        rob.links(i).Jm = Jm(i);
    end

    IndManipulability=zeros(heightCSV,1);
    IndDinamicManipulability=zeros(heightCSV,2);
    matrizOutputCSV=zeros(heightCSV,4);
    
    for row = 1:heightCSV
        qRow=qFinal(row,:);
        J = jacob0(rob,qRow);
        detJ(row)=det(J);
        normJ(row) = norm(J,'fro');
        normInvJ(row) = norm(inv(J),'fro');
        destreza(row) =  normJ(row)*normInvJ(row);
        cl (row) = 1/destreza(row);
        IndManipulability(row)=rob.maniplty(qRow,'method','yoshikawa','axes','all','dof',[1 2 3 4 5 6]);
        %IndManipulability(row)=rob.maniplty(qRow,'method','yoshikawa','axes','all','dof',[1 2 3 4 5 6]);
        [m,md] =mpf.DinamicManipulabilty(rob,qRow);
        IndDinamicManipulability(row,1) =m;
        IndDinamicManipulability(row,2) =md;

        matrizOutputCSV(row,1)=string(CSV{row,1}); 
        matrizOutputCSV(row,2)=m;
        matrizOutputCSV(row,3)=md; 
        matrizOutputCSV(row,4)=1/destreza(row);; 
    end 
    
    
    cd 'C:\Users\oscar\OneDrive - Universidad Nacional Autónoma de Honduras\f_Docturado FC-UA\Desarrollo Doctorado\Proyectos\EMG-Kinect\Matlab\Output_Manipulability';

    %Guardar csv
    fileFinal=string(strcat(OutputFile,'\',strcat(nameNewFile,'.csv')));
    T2 = array2table(matrizOutputCSV)
    T2.Properties.VariableNames(1:4) = {'timestamp','Índice de Manipulabilidad','Índice de Manipulabilidad Dinámica','Índice de Condicionamiento Local'}
    writetable(T2,string(strcat(nameNewFile,'.csv')))

    
    f = figure;
    hold on
    subplot(3,1,1);
    plot(matrizOutputCSV(:,2))
    legend('Índice de Manipulabilidad')
    subplot(3,1,2); 
    plot(matrizOutputCSV(:,3))
    legend('Índice de Manipulabilidad Dinámica')
    subplot(3,1,3); 
    plot(matrizOutputCSV(:,4))
    legend('Índice de Condicionamiento Local')
    hold off
    exportgraphics(f,string(strcat(nameNewFile,'.png')))
end
cd ..
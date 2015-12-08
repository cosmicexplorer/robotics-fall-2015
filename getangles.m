%function [output1,output2] = getangles(args)
function getangles
    %theta = args.arg1';
    %final = args.arg2;
    type = 6;
    theta = [1;-0.7;0;1.4;0;0.85;0];
    %theta = [1;-0.377;-0.2;1.4;0.1;0.777;0.067];
    final = [0.3,.5625,.01516];
    finalr = final + [0,0,1.0374-0.8077];
    finalr = finalr';
    Rdes = ROTy(pi/2);

    dt = 0.1;
    w = [0,0,1,0,1,0,1;0,1,0,1,0,1,0;1,0,0,0,0,0,0];
    q = [0,0.069,0.171,0.4334,0.537,0.8077,0.9237;0,0,0,0,0,0,0;...
         0,0.2703,0.2703,0.2014,0.2014,0.1913,0.1913];
    tw = zeros(6,7);
    for i = 1:7
        tw(:,i) = [-cross(w(:,i),q(:,i));w(:,i)];
    end

    g01 = eye(4);
    g02 = [ROTx(-pi/2),[0.0690;0;0.2703];zeros(1,3),1]
    g03 = [ROTy(pi/2),[0.1710;0;0.2703];zeros(1,3),1];
    g04 = [ROTx(-pi/2),[0.4334;0;0.2014];zeros(1,3),1];
    g05 = [ROTy(pi/2),[0.5370;0;0.2014];zeros(1,3),1];
    g06 = [ROTx(-pi/2),[0.8077;0;0.1913];zeros(1,3),1];
    g07 = [ROTy(pi/2),[0.9237;0;0.1913];zeros(1,3),1];
    g0e = [ROTy(pi/2),[1.0374;-0.0022;0.1924];zeros(1,3),1];
    %g0e = [ROTy(pi/2),[1.0237;0;0.1913];zeros(1,3),1];



    framecount = 1;
    q = 1;

    while true
        q=q+1;
        if floor(q/200) == ceil(q/200)
            pause(3);
        end
        for i = 1:7
            ex{i} = expm(hat6(tw(:,i))*theta(i));
        end
        H{1} = ex{1}*g01;
        H{2} = ex{1}*ex{2}*g02;
        H{3} = ex{1}*ex{2}*ex{3}*g03;
        H{4} = ex{1}*ex{2}*ex{3}*ex{4}*g04;
        H{5} = ex{1}*ex{2}*ex{3}*ex{4}*ex{5}*g05;
        H{6} = ex{1}*ex{2}*ex{3}*ex{4}*ex{5}*ex{6}*g06;
        H{7} = ex{1}*ex{2}*ex{3}*ex{4}*ex{5}*ex{6}*ex{7}*g07;
        He = ex{1}*ex{2}*ex{3}*ex{4}*ex{5}*ex{6}*ex{7}*g0e;

        Oe = He(1:3,4);
        O = zeros(3,7);
        J = zeros(6,7);
        JO = zeros(6,7);
        for i=1:7
            O(:,i) = H{i}(1:3,4);
            J(:,i) = [cross(w(:,i),Oe-O(:,i));w(:,i)];
            JO(:,i) = [cross(w(:,i),O(:,7)-O(:,i));w(:,i)];
        end
        %JO(:,5:7)=0;
        %JE(:,1:5)=0;
        DrawRobot(O,Oe,final,finalr);
        movie_frames(framecount) = getframe(gcf);
        framecount = framecount+1;

        CurrR = He(1:3,1:3);
        %Reul = rotm2eul(R,'XYZ');
        %ErrR = dot(Rdes,CurrR')
        ErrR = CurrR\Rdes
        thet = acos((trace(ErrR))/2);
        %me = 1./(2./sin(thet))*[ErrR(3,2)-ErrR(2,3);ErrR(1,3)-ErrR(3,1);ErrR(2,1)-ErrR(1,2)];
        me = vrrotmat2vec(ErrR);
        me = me(1,4)*(me(1,1:3)')
        w_unit = me;
        EdotR = .001*w_unit;
        %Rm = R\Rdes;

        %[r,p,y] = dcm2angle(Rm, 'XYZ')
        %V = (1/cos(p))*[cos(r) sin(r) 0;-sin(r)*cos(p) cos(r)*cos(p) 0;cos(r)*sin(p) sin(r)*sin(p) cos(r)];
        %E = [eye(3,3) zeros(3,3); zeros(3,3) V];
        %J = E*JE;
        err = [final(1)-Oe(1);final(2)-Oe(2);final(3)-Oe(3)];
        %Rerr = [finalr(1)-O(1,7);finalr(2)-O(2,7);finalr(3)-O(3,7)];
        %Rerr = [r;p;y];
        nerr = norm(err);
        %nRerr = norm(Rerr);
        nRerr = norm(EdotR)
        if type == 6
            if (nerr>0.01) && (nRerr>0.01)
                %Edot = [err./nerr*0.02;Rerr./nRerr*0.05];
                Edot = [err./nerr*0.02];
                %EdotR =[Rerr./nRerr*0.02;zeros(3,1)];
            elseif (nerr>0.01) && (nRerr<0.01)
                Edot = [err./nerr*0.02];
                EdotR = zeros(3,1);
            elseif (nerr<0.01) && (nRerr>0.01)
                Edot = zeros(3,1);
            else
                %pause(5);
                break
            end
            qdot = J'*(J*J'+0.01*eye(6,6))^(-1)*[Edot;EdotR];
        elseif type == 5
            if (nerr>0.01) && (nRerr>0.5)
                Edot = [err./nerr*0.02;Rerr./nRerr*0.05];
                %Edot = [err./nerr*0.02;zeros(3,1)];
            elseif (nerr>0.01) && (nRerr<0.5)
                Edot = [err./nerr*0.02;zeros(3,1)];
            elseif (nerr<0.01) && (nRerr>0.5)
                Edot = [zeros(3,1);Rerr./nRerr*0.05];
            else
                %pause(5);
                break
            end
            qdot = J'*(J*J'+0.01*eye(6,6))^(-1)*Edot;
        elseif type == 3
            J = J(1:3,:);
            if nerr > 0.01
                Edot = err./nerr*0.05;
            else
                pause(5);
                break
            end
            qdot = J'*(J*J'+0.01*eye(3,3))^(-1)*Edot;
        else
           if nerr > 0.01
                Edot = [err./nerr*0.05;zeros(3,1)];
            else
                pause(5);
                break
            end
            qdot = J'*(J*J'+0.01*eye(6,6))^(-1)*Edot;
        end
        theta = theta + qdot*dt
    end

     nFrames = length(movie_frames);
     vidObj = VideoWriter('Baxter_Checkers.avi');
     vidObj.Quality = 100;
     vidObj.FrameRate = nFrames/5;
     open(vidObj);
     for i = 1:nFrames
         writeVideo(vidObj,movie_frames(i));
     end
     close(gcf)
     close(vidObj);

    output1 = theta;
    output2 = He;
end

function DrawRobot(O,Oe,final,finalr)

    clf        % Clears the figure
    axis equal % Sets axes scaling equal
    axis([0,1.5,0,1.5,-1,1]); % Defines axes bounds
    view([1,1,1]); %View angle
    hold on;

    % Plots point and lines for links


    for i=1:7
        plot3(O(1,i),O(2,i),O(3,i),'ro');
        if i == 7
            plot3(Oe(1),Oe(2),Oe(3),'ro');
            line([O(1,i),Oe(1)],[O(2,i),Oe(2)],[O(3,i),Oe(3)]);
            plot3(final(1),final(2),final(3),'bo');
            plot3(finalr(1),finalr(2),finalr(3),'go');

        else
            line([O(1,i),O(1,i+1)],[O(2,i),O(2,i+1)],[O(3,i),O(3,i+1)]);
        end
    end

end

function [R] = ROTx(theta)
    R = [1,0,0;0,cos(theta),-sin(theta);0,sin(theta),cos(theta)];
end

function [R] = ROTy(theta)
    R = [cos(theta),0,sin(theta);0,1,0;-sin(theta),0,cos(theta)];
end

function [R] = ROTz(theta)
    R = [cos(theta),-sin(theta),0;sin(theta),cos(theta),0;0,0,1];
end

function Ehat = hat6(twist)
Ehat = [0 -twist(6) twist(5) twist(1);...
        twist(6) 0 -twist(4) twist(2);...
        -twist(5) twist(4) 0 twist(3);...
           0        0      0       0];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Task for Velázquez, Villareal, Bouzas  
%
%   This program displays a spaceship moving around the earth and
%   changing its position at every trial. Participants have to click
%   on the place where they think the spaceship will move next time. After
%   they choose, a red dot indicates the position they clicked on.
%   There are 1200 trials divided up into 4 blocks of 300.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Parameters for the task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear
clc

a=get(0,'screensize');

centerx=a(3)/2;
centery=a(4)/2;

%NUMBER OF BLOCKS
n_block = 4;

%RADIO
rad=330; % 420 used for the real task

%NOISE FOR VELOCITY
sigma_v = repmat(0.07,1, n_block);

%NOISE FOR POSITION
sigma_x = repmat(0, 1, n_block);

%RANDOMIZE ORDER OF BLOCKS

ratio = [0.05,0.5,1,2];
ratio = perms(ratio);
perm = length(ratio(:,1));
number = randi(perm);
ratio = ratio(number,:);

%NOISE FOR OBSERVATIONS.1
sigma_o = sqrt(sigma_v.^2 ./ ratio);

%NUMBER OF TRIALS
n_trials = 300;
num = input('Please input a random 8 digit number ');
randn('seed',sum(clock)+num);

for h=1:n_block
    
    x0=0;
    v0=0;
    
    while 1
        [r,dv,m,V]=getdata(n_trials, sigma_o(h), sigma_x(h), sigma_v(h), x0, v0,num);
        
        if max(V)>0.8 || min(V)<-0.8
            
            continue;
        else
            break;
        end
    end
    
    %%%%%%%%%%%%%%%
    
    figure; clf;
    subplot(2,1,1);hold on;
    scatter(1:n_trials, m, 'b');
    scatter(1:n_trials, r, 'r');
    xlabel('time');
    ylabel('position');
    legend({'position','observation'});
    subplot(2,1,2);hold on;
    scatter(1:n_trials, V, 'b');
    scatter(1:n_trials, dv, 'r');
    xlabel('time');
    ylabel('velocity');
    legend({'velocity','direction'});
    
    %%%%%%%%%%%%%%%
    
    %X coordinate
    r1=centerx + rad.*cos(r);
    %Y coordinates
    r2=centery + rad.*sin(r);
    
    %theta
    block_theta(:,h)=r;
    %actual position X
    block_observationsX(:,h)=r1;
    %actual position Y
    block_observationsY(:,h)=r2;
    %generative mean of theta
    block_position(:,h)=m;
    %velocity of theta
    block_velocity(:,h)=V;
    %velocity switch
    block_vdirection(:,h)=dv;
    
end

figure(5)
scatter(r1,r2);

figure(6)

scatter(1:length(V),V);



%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Screen and keyboard
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Screen('Preference', 'SkipSyncTests', 1);
subjectname = input('Please input the subject name: ','s');
KbName('UnifyKeyNames');
spaceKey = KbName('space'); escKey = KbName('ESCAPE');
Key1=KbName('LeftArrow'); Key2=KbName('RightArrow');

%Colors
gray = [127 127 127 ]; white = [ 255 255 255]; black = [0 0 0];
bgcolor = black; textcolor = white; green=[0,255,0]; yellow=[255,255,0];
red=[255,0,0]; black=[0,0,0]; blue = [30,144,255]; indigo=[75,0,130];
navy=[0 0 128];


%screenrect is the coordinates of the screen (left bottom width hight)
%the screen is specified by OpendWindow  and 0 (main screen)

[mainwin, screenrect] = Screen(0,'OpenWindow');
Screen('BlendFunction', mainwin, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
HideCursor(mainwin);
Screen('FillRect', mainwin, bgcolor);
length1=screenrect(3);
height1=screenrect(4);
center = [screenrect(3)/2 screenrect(4)/2];

%Standarize position on the screen

% X AXIS

xm = center(1); x_4=xm/2; x_8=x_4/2; x_16=x_8/2; x_32=x_16/2;
x_64=x_32/2; x_128=x_64/2;

% Y AXIS

ym = center(2); y_4=ym/2; y_8=y_4/2; y_16=y_8/2; y_32=y_16/2;
y_64=y_32/2; y_128=y_64/2;

%SIZE dots of trajectory
Tsize=3;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Key images
%left_arrow=imread('left_arrow.jpg');
%left_arrow1=Screen('MakeTexture', mainwin, left_arrow);
%right_arrow=imread('right_arrow.jpg');
%right_arrow1=Screen('MakeTexture', mainwin, right_arrow);
%esc=imread('esc.jpg');
%esc1=Screen('MakeTexture', mainwin, esc);

%White right arrow

[right_arrowB map alpha] = imread('right_arrow2.png');
right_arrowB(:,:,4) = alpha;
right_arrow2=Screen('MakeTexture', mainwin, right_arrowB);

%White left arrow
[left_arrowC map alpha]=imread('left_arrow3.png');
left_arrowC(:,:,4) = alpha;
left_arrow3=Screen('MakeTexture', mainwin, left_arrowC);

%Overshadow left arrow
%left_arrowB  = imread('left_arrow2.jpg');
%left_arrow2=Screen('MakeTexture', mainwin, left_arrowB);

%Images task.
[im4 map alpha] = imread('ufo1_red.png');
im4(:,:,4) = alpha;
[im5 map alpha] = imread('ufo1.png');
im5(:,:,4) = alpha;
[im6 map alpha]= imread('Earth.png');
im6(:,:,4) = alpha;
%landscape = imread('space.png');

%MakeTexture
ufo_red=Screen('MakeTexture',mainwin,im4);
ufo=Screen('MakeTexture',mainwin,im5);
earth = Screen('MakeTexture',mainwin,im6);
%landscape1 = Screen('MakeTexture',mainwin,landscape);

%Coordinates of ghost trajectory

T=trajectory(centerx,centery,rad);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Instructions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

n_instructions=13;
i=0;

while i<n_instructions
    
    i=i+1;
    if i==1
        
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['En esta sección se te dará una serie de instrucciones.\n\nUsa las siguientes teclas para navegar en ellas. '] ,x_8,y_4,textcolor);
        DrawFormattedText(mainwin,['Presiona la flecha derecha para continuar'] ,x_8+x_16,y_4+y_8+y_16,textcolor);
        DrawFormattedText(mainwin,['Presiona la flecha izquierda para regresar'] ,x_8+x_16,ym+y_16,textcolor);
        DrawFormattedText(mainwin,['Presiona esc para salir de la tarea'] ,x_8+x_16,ym+y_8+y_16,textcolor);
        Screen('DrawTexture',mainwin, right_arrow1,[],mrect(xm+x_8+x_16,y_4+y_8+y_32+y_64,x_64+x_128));
        Screen('DrawTexture',mainwin, left_arrow1,[],mrect(xm+x_8+x_16,ym+y_32+y_128,x_64+x_128));
        Screen('DrawTexture',mainwin, esc1,[],mrect(xm+x_8+x_16,ym+y_8+y_32+y_128,x_64+x_128));
        Screen('DrawTexture',mainwin, left_arrow2,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
    elseif i==2
        %2
        Screen('TextSize', mainwin,25);
        im = imread('D8.PNG'); D1 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D1,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['En el experimento se mostrará la imagen del planeta tierra en el espacio.'] ,'center',y_16+y_32,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[], mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==3
        %3
        Screen('TextSize', mainwin,25);
        im = imread('D9.PNG'); D2 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D2,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Un objeto volador no Identificado (OVNI) orbitará alrededor de ella.'] ,'center',y_16+y_32,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==4
        Screen('TextSize', mainwin,25);
        im = imread('D10.PNG'); D3 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D3,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Su trayectoria estará indicada con un círculo punteado.'] ,'center',y_16+y_32,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==5
        Screen('TextSize', mainwin,25);
        im = imread('D12.PNG'); D4 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D4,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['¡Y tu tarea es predecir su posición exacta en cada momento!'] ,'center',y_16+y_32,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
    elseif i==6
        
        Screen('TextSize', mainwin,25);
        im = imread('D12.PNG'); D5 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D5,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['El experimento comienza cuando el OVNI se muestra en algún punto de su trayectoria.'] ,'center',y_16+y_32,textcolor);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==7
        
        Screen('TextSize', mainwin,25);
        im = imread('D14.PNG'); D6 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D6,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Después de un tiempo breve, éste desaparecerá.'] ,'center',y_16+y_32,textcolor);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('TextSize', mainwin,25);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
    elseif i==8
        
        Screen('TextSize', mainwin,25);
        im = imread('D16.PNG'); D7 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D7,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Cuando esto ocurra, usa el botón izquierdo de tu cursor para indicar\nla posición donde creas que se va a mover.'] ,'center',y_16+y_32,textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==9
        
        Screen('TextSize', mainwin,25);
        im = imread('D19.PNG'); D7 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D7,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Un punto rojo indicará la posición donde presionaste\ny el OVNI se mostrará en su nueva ubicación.'] ,'center',y_16+y_32,textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==10
        
        Screen('TextSize', mainwin,25);
        im = imread('D19.PNG'); D7 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D7,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Esta secuencia se repite a lo largo de todo el experimento.'] ,'center',y_16+y_32,textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==11
        
        Screen('TextSize', mainwin,25);
        im = imread('D20.PNG'); D7 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D7,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Si predices adecuadamente la posición del OVNI, éste se tornará rojo.\nDe lo contrario, no cambiará de color.'] ,'center',y_16+y_32,textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    elseif i==12
        
        Screen('TextSize', mainwin,25);
        im = imread('D18.PNG'); D7 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D7,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Mientras el OVNI aparezca dentro del círculo rojo, será considerado como un acierto.'] ,'center',y_16+y_32,textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,['Ejemplo:'] ,x_8,y_4,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
        
    else
        
        Screen('TextSize', mainwin,50);
        Screen('DrawTexture', mainwin, landscape1,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['¡Practica un poco!'] ,'center','center',textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,'[Presiona la flecha derecha para iniciar la fase de entrenamiento]' ,'center',ym+y_8,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            i=i-2;
        end
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Practice phase
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%
n_practice=2;
ii=0;
tt=0;
sd_vel(1)=0.3;

while ii<n_practice
    
    ii=ii+1;
    
    if ii==1
        tt=0;
        ShowCursor('CrossHair',mainwin);
                
        while 1
            
            tt=tt+1;
            
            if tt==1
                
                rr(tt)=0;
            else
                sd_vel(tt)=sd_vel(tt-1) + 0.1*randn(1);
                rr(tt)=rr(tt-1)+sd_vel(tt);
                
            end
            
            %X coordinate
            rr1(tt)=centerx + rad.*cos(rr(tt));
            %Y coordinates
            rr2(tt)=centery + rad.*sin(rr(tt));
            
            
            if tt==1
                Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
                Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
                Screen('DrawDots', mainwin, T, Tsize, navy, [], 0);
                Screen('DrawTexture', mainwin, ufo,[],mrect2(rr1(tt),rr2(tt),y_32));
                DrawFormattedText(mainwin,['Fase de entrenamiento'] ,'center',y_16,textcolor);
                Screen('TextSize', mainwin,25);
                Screen(mainwin, 'Flip');
                
                WaitSecs(1);
                Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
                Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
                Screen('DrawDots', mainwin, T, Tsize, navy , [], 0);
                DrawFormattedText(mainwin,['Fase de entrenamiento'] ,'center',y_16,textcolor);
                Screen('TextSize', mainwin,25);
                Screen(mainwin, 'Flip');
                
                ispressed = false;
                
                while ~ispressed
                    
                    [x_p,y1_p,buttons]= GetMouse(mainwin);
                    
                    [d] = distance(x_p, y1_p,centerx, centery);
                    
                    if d > rad+50 || d< rad-50
                        
                        ispressed = 0;
                    else
                        ispressed = buttons(1);
                    end
                    
                    [keyIsDown, secs, keyCode] = KbCheck;
                    
                    if keyIsDown
                        if keyCode(escKey)
                            Screen('CloseAll');
                            return;
                        elseif keyCode(Key2)
                            break;
                        else
                            ;
                        end
                    end
                end
                               
                if keyCode(Key2)
                    break;
                end
                
                X_p(tt) = x_p;
                Y_p(tt) = y1_p;
                
            else
                
                Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
                Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
                Screen('DrawDots', mainwin, T, Tsize , navy , [], 0);
                
                [d]=distance(rr1(tt),rr2(tt), X_p(tt-1), Y_p(tt-1));
                
                if d <= 52
                    
                    Screen('DrawTexture', mainwin, ufo_red,[],mrect2(rr1(tt),rr2(tt),y_32));
                else
                    Screen('DrawTexture', mainwin, ufo,[],mrect2(rr1(tt),rr2(tt),y_32));
                end
                
                Screen('FrameOval',mainwin,red,mrect(x_p,y1_p,20));
                Screen('FillOval',mainwin,red,mrect(x_p,y1_p,7));
                if 20<tt && tt<30
                    DrawFormattedText(mainwin,['¡Sigue practicando!'] ,'center',y_16,textcolor);
                elseif tt>=30
                    DrawFormattedText(mainwin,['Cuando estés listo, presiona la flecha derecha para comenzar el experimento'] ,'center',y_16,textcolor);
                else
                    DrawFormattedText(mainwin,['Fase de entrenamiento'] ,'center',y_16,textcolor);
                end
                Screen('TextSize', mainwin,25);
                Screen(mainwin, 'Flip');
                
                %cambiar 80
                WaitSecs(0.5);
                Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
                Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
                Screen('DrawDots', mainwin, T, Tsize, navy , [], 0);
                Screen('FrameOval',mainwin,red,mrect(x_p,y1_p,20));
                Screen('FillOval',mainwin,red,mrect(x_p,y1_p,7));
                if 20<tt && tt<30
                    DrawFormattedText(mainwin,['¡Sigue practicando!'] ,'center',y_16,textcolor);
                elseif tt>=30
                    DrawFormattedText(mainwin,['Cuando estés listo, presiona la flecha derecha para comenzar el experimento'] ,'center',y_16,textcolor);
                else
                    DrawFormattedText(mainwin,['Fase de entrenamiento'] ,'center',y_16,textcolor);
                end
                Screen('TextSize', mainwin,25);
                Screen(mainwin, 'Flip');
                
                ispressed = false;
                
                while ~ispressed
                    
                    [x_p,y1_p,buttons]= GetMouse(mainwin);
                    
                    [d] = distance(x_p, y1_p,centerx, centery);
                    
                    if d > rad+50 || d< rad-50
                        
                        ispressed = 0;
                    else
                        ispressed = buttons(1);
                    end
                    
                    [keyIsDown, secs, keyCode] = KbCheck;
                    
                    if keyIsDown
                        if keyCode(escKey)
                            Screen('CloseAll');
                            return;
                        elseif keyCode(Key2)
                            break;
                        else
                            ;
                        end
                    end
                    
                end
                
                if keyCode(Key2)
                    break;
                end
                
                X_p(tt) = x_p;
                Y_p(tt) = y1_p;
                
            end
        end
        
    else
        
        Screen('TextSize', mainwin,50);
        Screen('DrawTexture', mainwin, landscape1,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['¿Estás listo?'] ,'center','center',textcolor);
        Screen('TextSize', mainwin,25);
        DrawFormattedText(mainwin,'[Presiona la flecha derecha para comenzar el experimento\n\no izquierda para volver al entrenamiento]' ,'center',ym+y_8,textcolor);
        Screen('DrawTexture',mainwin, left_arrow3,[],mrect(x_4+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen('DrawTexture',mainwin, right_arrow2,[],mrect(xm+x_8,ym+y_4+y_8+y_32,x_64+x_128));
        Screen(mainwin, 'Flip');
        
        while 1
            [secs, keyCode] = KbStrokeWait;
            if keyCode(Key1)
                break;
            elseif keyCode(Key2)
                break;
            elseif keyCode(escKey)
                ShowCursor;
                Screen('CloseAll');
                return;
            else
                continue;
            end
        end
        
        if keyCode(Key1)
            ii=ii-2;
        end
        
    end
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ShowCursor('CrossHair',mainwin);


for j=1:n_block
    
    %actual position X
    r1=block_observationsX(:,j);
    %actual position Y
    r2=block_observationsY(:,j);
    %generative mean
    m=block_position(:,j);
    %velocity
    V=block_velocity(:,j);
    %velocity switch
    dv=block_vdirection(:,j);
    
    for i=1:length(r)
        
        if i==1
            Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
            Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
            Screen('DrawDots', mainwin, T, Tsize, navy, [], 0);
            Screen('DrawTexture', mainwin, ufo,[],mrect2(r1(i),r2(i),y_32));
            
            rt_show(i,j) = Screen(mainwin, 'Flip');
            
            WaitSecs(1);
            Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
            Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
            Screen('DrawDots', mainwin, T, Tsize, navy , [], 0);
            rt_disappear(i,j) = Screen(mainwin, 'Flip');
            
            ispressed = false;
            
            while ~ispressed
                
                [x,y1,buttons]= GetMouse(mainwin);
                
                [d] = distance(x, y1,centerx, centery);
                
                if d > rad+50 || d< rad-50
                    
                    ispressed = 0;
                else
                    ispressed = buttons(1);
                end
                
                [keyIsDown, secs, keyCode] = KbCheck;
                rt_press(i,j) = secs;
                if keyIsDown
                    if keyCode(escKey)
                        Screen('CloseAll');
                        return;
                    else
                        ;
                    end
                end
                
            end
            
            X(i) = x;
            Y(i) = y1;
            aciertos(i)=nan;
        else
            
            Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
            Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
            Screen('DrawDots', mainwin, T, Tsize , navy , [], 0);
            
            [d]=distance(r1(i),r2(i), X(i-1), Y(i-1));
            
            if d <= 52
                
                Screen('DrawTexture', mainwin, ufo_red,[],mrect2(r1(i),r2(i),y_32));
                aciertos(i)=1;
            else
                Screen('DrawTexture', mainwin, ufo,[],mrect2(r1(i),r2(i),y_32));
                aciertos(i)=0;
            end
            
            Screen('FrameOval',mainwin,red,mrect(x,y1,20));
            Screen('FillOval',mainwin,red,mrect(x,y1,7));
            
            rt_show(i,j) = Screen(mainwin, 'Flip');
            
            %cambiar 80
            WaitSecs(0.5);
            Screen('DrawTexture',mainwin,landscape1,[],[0,0,length1,height1]);
            Screen('DrawTexture', mainwin, earth,[],mrect(centerx,centery,ym-y_8-y_16));
            Screen('DrawDots', mainwin, T, Tsize, navy , [], 0);
            Screen('FrameOval',mainwin,red,mrect(x,y1,20));
            Screen('FillOval',mainwin,red,mrect(x,y1,7));
            rt_disappear(i,j) = Screen(mainwin, 'Flip');
            
            ispressed = false;
            
            while ~ispressed
                
                [x,y1,buttons]= GetMouse(mainwin);
                
                [d] = distance(x, y1,centerx, centery);
                
                if d > rad+50 || d< rad-50
                    
                    ispressed = 0;
                else
                    ispressed = buttons(1);
                end
                
                [keyIsDown, secs, keyCode] = KbCheck;
                rt_press(i,j) = secs;
                if keyIsDown
                    if keyCode(escKey)
                        Screen('CloseAll');
                        return;
                    else
                        ;
                    end
                end
                
            end
            
            X(i) = x;
            Y(i) = y1;
        end
        
    end
    
    block_responsesX(:,j)= X;
    block_responsesY(:,j)= Y;
    aciertos_T=round((nansum(aciertos)/(n_trials-1))*100);
    m_aciertos(:,j)=aciertos_T;
    cc=['Porcentaje de aciertos: ',num2str(aciertos_T), '%'];
    
    if j<n_block
        Screen('TextSize', mainwin,30);
        Screen('DrawTexture', mainwin, landscape1,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Puedes tomar un pequeño descanso.\n\nPresiona cualquier tecla para iniciar la siguiente ronda cuando estés listo'] ,'center','center',textcolor);
        DrawFormattedText(mainwin,[cc] ,'center',ym-y_8,green);
        Screen(mainwin, 'Flip');
        [secs, keyCode] = KbStrokeWait;
    else
        ;
    end
    
end
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                           Save variables
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

save(['S_',subjectname],'m_aciertos','sigma_v','sigma_x','sigma_o','block_theta','block_responsesX', 'block_responsesY','block_observationsX','block_observationsY','block_position','block_velocity','block_vdirection','subjectname','rt_show','rt_disappear','rt_press','ratio','length1','height1');

Screen('TextSize', mainwin,30);
Screen('DrawTexture', mainwin, landscape1,[],[0,0,length1,height1]);
DrawFormattedText(mainwin,'Fin del experimento' ,'center','center',textcolor);
Screen('TextSize', mainwin,30);
DrawFormattedText(mainwin,[cc] ,'center',ym-y_8,green);
DrawFormattedText(mainwin,['\n\n\n\n\n\n\n [Presiona cualquier tecla para salir]'] ,'center','center',textcolor);
Screen(mainwin, 'Flip');

%END OF THE TASK
[secs, keyCode] = KbStrokeWait;
Screen('CloseAll')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   Task for Vel�zquez, Villareal, Bouzas  
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
num = 3141592;%input('Please input a random 8 digit number ');
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
    
    %figure; clf;
    %subplot(2,1,1);hold on;
    %scatter(1:n_trials, m, 'b');
    %scatter(1:n_trials, r, 'r');
    %xlabel('time');
    %ylabel('position');
    %legend({'position','observation'});
    %subplot(2,1,2);hold on;
    %scatter(1:n_trials, V, 'b');
    %scatter(1:n_trials, dv, 'r');
    %xlabel('time');
    %ylabel('velocity');
    %legend({'velocity','direction'});
    
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

%figure(5)
%scatter(r1,r2);

%figure(6)

%scatter(1:length(V),V);



%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Screen and keyboard
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Screen('Preference', 'SkipSyncTests', 1);
subjectname = 'MarkTheMagnificent'%input('Please input the subject name: ','s');
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
left_arrow=imread('left_arrow.jpg');
left_arrow1=Screen('MakeTexture', mainwin, left_arrow);
right_arrow=imread('right_arrow.jpg');
right_arrow1=Screen('MakeTexture', mainwin, right_arrow);
esc=imread('esc.jpg');
esc1=Screen('MakeTexture', mainwin, esc);

%White right arrow

[right_arrowB map alpha] = imread('right_arrow2.png');
right_arrowB(:,:,4) = alpha;
right_arrow2=Screen('MakeTexture', mainwin, right_arrowB);

%White left arrow
[left_arrowC map alpha]=imread('left_arrow3.png');
left_arrowC(:,:,4) = alpha;
left_arrow3=Screen('MakeTexture', mainwin, left_arrowC);

%Overshadow left arrow
left_arrowB  = imread('left_arrow.jpg');
left_arrow2=Screen('MakeTexture', mainwin, left_arrowB);

%Images task.
[im4 map alpha] = imread('ufo1_red.png');
im4(:,:,4) = alpha;
[im5 map alpha] = imread('ufo1.png');
im5(:,:,4) = alpha;
[im6 map alpha]= imread('Earth.png');
im6(:,:,4) = alpha;
landscape = imread('space.png');

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
        DrawFormattedText(mainwin,['En esta secci�n se te dar� una serie de instrucciones.\n\nUsa las siguientes teclas para navegar en ellas. '] ,x_8,y_4,textcolor);
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
        DrawFormattedText(mainwin,['En el experimento se mostrar� la imagen del planeta tierra en el espacio.'] ,'center',y_16+y_32,textcolor);
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
    elseif i==3
        %3
        Screen('TextSize', mainwin,25);
        im = imread('D9.PNG'); D2 = Screen('MakeTexture', mainwin, im);
        Screen('DrawTexture', mainwin, D2,[],[0,0,length1,height1]);
        DrawFormattedText(mainwin,['Un objeto volador no Identificado (OVNI) orbitar� alrededor de ella.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Su trayectoria estar� indicada con un c�rculo punteado.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['�Y tu tarea es predecir su posici�n exacta en cada momento!'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['El experimento comienza cuando el OVNI se muestra en alg�n punto de su trayectoria.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Despu�s de un tiempo breve, �ste desaparecer�.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Cuando esto ocurra, usa el bot�n izquierdo de tu cursor para indicar\nla posici�n donde creas que se va a mover.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Un punto rojo indicar� la posici�n donde presionaste\ny el OVNI se mostrar� en su nueva ubicaci�n.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Si predices adecuadamente la posici�n del OVNI, �ste se tornar� rojo.\nDe lo contrario, no cambiar� de color.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['Mientras el OVNI aparezca dentro del c�rculo rojo, ser� considerado como un acierto.'] ,'center',y_16+y_32,textcolor);
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
        DrawFormattedText(mainwin,['�Practica un poco!'] ,'center','center',textcolor);
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

Screen('CloseAll')
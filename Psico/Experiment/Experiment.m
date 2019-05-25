%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   This program displays a spaceship moving around the earth and
%   changing its position at every trial. Participants have to click
%   on the place where they think the spaceship will move next time. After
%   they choose, a red dot indicates the position they clicked on.
%   There are 1200 trials divided up into 4 blocks of 300.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                       Parameters for the task
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%Number of blocks, number of trials per block and radius
n_blocks = 4;
rad      = 330;
n_trials = 300;
randn('seed',time());
subject_name = 'MarkTheMagnificent'; %input('Please input the subject name: ','s');

%Randomize block order. Ratio is a random choice between al
%possible permutations of the set of ratios
ratio  = [0.05, 0.5, 1.0 ,2.0];
ratio  = perms(ratio);
number = randi(length(ratio(:,1)));
ratio  = ratio(number,:);

%Noises for velocity, position and observation
sigma_v = repmat(0.07, 1, n_blocks);
sigma_x = repmat(0.0,  1, n_blocks);
sigma_o = sqrt(sigma_v.^2 ./ ratio);

%Get the center of the screen
scr_size  = get(0,'screensize');
screen_w  = scr_size(3);
screen_h  = scr_size(4);
screen_cx = screen_w/2
screen_cy = screen_h/2
screen_w = 1200
screen_h = 1600
screen_origin_x = 1367

%I think this loop generates all ship positions that will be used
%for the whole experiment. But I'm not sure.
for i=1:n_blocks
    x0=0;
    v0=0;
    %[theta, velocity switch, generative mean of theta and velocity of theta]
    [r,dv,m,V]=get_data(n_trials, sigma_o(i), sigma_x(i), sigma_v(i), x0, v0, 314);
    % I don't know why this condition is necessary
    while max(V)>0.8 || min(V)<-0.8
        [r,dv,m,V]=get_data(n_trials, sigma_o(i), sigma_x(i), sigma_v(i), x0, v0, 314); 
    end
    
    %XY in screen coordinates
    xo = screen_cx + rad.*cos(r);
    yo = screen_cy + rad.*sin(r);
    
    block_theta        (:,i) = r;
    block_observationsX(:,i) = xo;
    block_observationsY(:,i) = yo;
    block_position     (:,i) = m;
    block_velocity     (:,i) = V;
    block_vdirection   (:,i) = dv;
end
trajectory = trajectory(screen_cx, screen_cy, rad);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                          Screen and keyboard settings
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

KbName('UnifyKeyNames');
key_space  = KbName('space');
key_escape = KbName('ESCAPE');
key_left   = KbName('LeftArrow');
key_right  = KbName('RightArrow');

%Colors
color_gray   = [127 127 127];
color_white  = [255 255 255];
color_black  = [  0   0   0];
color_green  = [  0 255   0];
color_yellow = [255 255   0];
color_red    = [255   0   0];
color_black  = [  0   0   0];
color_blue   = [ 30 144 255];
color_indigo = [ 75   0 130];
color_navy   = [  0   0 128];
color_bg     = color_black;
color_text   = color_white;

%Standarize position on the screen
% X AXIS
xm    = screen_w/2;
x_4   = xm/2;
x_8   = x_4/2;
x_16  = x_8/2;
x_32  = x_16/2;
x_64  = x_32/2;
x_128 = x_64/2;

ym    = screen_h/2;
y_4   = ym/2;
y_8   = y_4/2;
y_8   = y_4/2;
y_16  = y_8/2;
y_32  = y_16/2;
y_64  = y_32/2;
y_128 = y_64/2;
%SIZE dots of trajectory
dots_size = 3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                               Images
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Key images
[img_l_arrow, map, alpha_l_arrow] = imread('Figures/left_arrow.png' );
[img_r_arrow, map, alpha_r_arrow] = imread('Figures/right_arrow.png');
[img_ufo_w  , map, alpha_ufo_w  ] = imread('Figures/ufo_white.png');
[img_ufo_r  , map, alpha_ufo_r  ] = imread('Figures/ufo_red.png');
[img_earth  , map, alpha_earth  ] = imread('Figures/Earth.png');
img_l_arrow(:,:,4) = alpha_l_arrow;
img_r_arrow(:,:,4) = alpha_r_arrow;
img_ufo_w  (:,:,4) = alpha_ufo_w  ;
img_ufo_r  (:,:,4) = alpha_ufo_r  ;
img_earth  (:,:,4) = alpha_earth  ;
img_escape  = imread('Figures/escape.jpg');
img_demo_03 = imread('Figures/Demo03.png');
img_demo_04 = imread('Figures/Demo04.png');
img_demo_05 = imread('Figures/Demo05.png');
img_demo_07 = imread('Figures/Demo07.png');
img_demo_08 = imread('Figures/Demo08.png');
img_demo_09 = imread('Figures/Demo09.png');
img_demo_11 = imread('Figures/Demo11.png');
img_demo_12 = imread('Figures/Demo12.png');

%Open window and set background color to black
Screen('Preference', 'SkipSyncTests', 1);
[main_window, screen_rect] = Screen(0,'OpenWindow');
Screen('BlendFunction', main_window, GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
HideCursor(main_window);
Screen('FillRect', main_window, color_bg);

tex_l_arrow = Screen('MakeTexture', main_window, img_l_arrow);
tex_r_arrow = Screen('MakeTexture', main_window, img_r_arrow);
tex_ufo_w   = Screen('MakeTexture', main_window, img_ufo_w  );
tex_ufo_r   = Screen('MakeTexture', main_window, img_ufo_r  );
tex_earth   = Screen('MakeTexture', main_window, img_earth  );
tex_escape  = Screen('MakeTexture', main_window, img_escape );
tex_demo_03 = Screen('MakeTexture', main_window, img_demo_03);
tex_demo_04 = Screen('MakeTexture', main_window, img_demo_04);
tex_demo_05 = Screen('MakeTexture', main_window, img_demo_05);
tex_demo_07 = Screen('MakeTexture', main_window, img_demo_07);
tex_demo_08 = Screen('MakeTexture', main_window, img_demo_08);
tex_demo_09 = Screen('MakeTexture', main_window, img_demo_09);
tex_demo_11 = Screen('MakeTexture', main_window, img_demo_11);
tex_demo_12 = Screen('MakeTexture', main_window, img_demo_12);

SM_Definitions;
sm_state = SM_DEMO_INIT;

while sm_state != SM_DEMO_END
    Screen('TextSize', main_window, 25);
    switch sm_state
      case SM_DEMO_INIT
        DrawFormattedText(main_window, ['En esta sección se te dará ' ...
                            'una serie de instrucciones.'] , ...
                          screen_origin_x + x_8,y_4,color_text);
        DrawFormattedText(main_window, ['Usa las siguientes teclas para navegar en ellas.'],screen_origin_x+x_8,y_4+y_16, color_text);
        DrawFormattedText(main_window ,['Presiona la flecha derecha ' ...
                            'para continuar'] , screen_origin_x+x_8+x_16,y_4+y_8+y_16,color_text);
        DrawFormattedText(main_window ,['Presiona la flecha izquierda para regresar'] ,screen_origin_x+x_8+x_16,ym+y_16,color_text);
        DrawFormattedText(main_window ,['Presiona esc para salir de la tarea'] ,screen_origin_x+x_8+x_16,ym+y_8+y_16,color_text);
        Screen('DrawTexture',main_window, tex_r_arrow,[],mrect(screen_origin_x+xm+x_8+x_16, y_4+y_8+y_32+y_64,x_64+x_128));
        Screen('DrawTexture',main_window, tex_l_arrow,[],mrect(screen_origin_x+xm+x_8+x_16,ym+y_32+y_128,x_64+x_128));
        Screen('DrawTexture',main_window, tex_escape ,[],mrect(screen_origin_x+xm+x_8+x_16,ym+y_8+y_32+y_128,x_64+x_128));
        Screen(main_window, 'Flip');
        [secs, key_code] = KbStrokeWait;
        if key_code(key_right)
            sm_state = SM_DEMO_END;
        end 
      otherwise
        sm_state = SM_DEMO_END;
    end
    if key_code(key_escape)
        sm_state = SM_DEMO_END;
    end
end

Screen('CloseAll');
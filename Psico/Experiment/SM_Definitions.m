SM_DEMO_INIT           = 1 ;
SM_DEMO_SHOW_EARTH     = 2 ;
SM_DEMO_SHOW_UFO       = 3 ;
SM_DEMO_SHOW_PATH      = 4 ;
SM_DEMO_EXPLAIN_TASK   = 5 ;
SM_DEMO_EXPLAIN_START  = 6 ;
SM_DEMO_DISAPPEAR_UFO  = 7 ;
SM_DEMO_TELL_CLICK     = 8 ;
SM_DEMO_SHOW_CLICK_POS = 9 ;
SM_DEMO_EXPLAIN_REPEAT = 10;
SM_DEMO_UFO_COLOR      = 11;
SM_DEMO_EXPLAIN_CIRCLE = 12;
SM_DEMO_GO_AHEAD       = 13;
SM_DEMO_END            = 14;

%Standarize position on the screen
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

SMD_INIT_Txt1_t = ['En esta sección se te dará una serie de instrucciones.'];
SMD_INIT_Txt2_t = ['Usa las siguientes teclas para navegar en ellas.'      ];
SMD_INIT_Txt3_t = ['Presiona la flecha derecha para continuar'             ];
SMD_INIT_Txt4_t = ['Presiona la flecha izquierda para regresar'            ];
SMD_INIT_Txt5_t = ['Presiona esc para salir de la tarea'                   ];

SMD_INIT_Txt1_x = screen_origin_x + x_8     ;
SMD_INIT_Txt2_x = screen_origin_x + x_8     ;
SMD_INIT_Txt3_x = screen_origin_x + x_8+x_16;
SMD_INIT_Txt4_x = screen_origin_x + x_8+x_16;
SMD_INIT_Txt5_x = screen_origin_x + x_8+x_16;

SMD_INIT_Txt1_y = y_4         ;
SMD_INIT_Txt2_y = y_4 + y_16  ;
SMD_INIT_Txt3_y = y_4+y_8+y_16;
SMD_INIT_Txt4_y = ym+y_16     ;
SMD_INIT_Txt5_y = ym+y_8+y_16 ;

SMD_SHOW_EARTH_Txt1_t = ['En el experimento se mostrará la imagen del planeta tierra en el espacio.'];
SMD_SHOW_EARTH_Txt1_x = 'center';
SMD_SHOW_EARTH_Txt1_y = y_16 + y_32;

SMD_SHOW_UFO_Txt1 = ['Un objeto volador no Identificado (OVNI) orbitará alrededor de ella.'];

SMD_EXPLAIN_TASK_Txt1  = ['¡Y tu tarea es predecir su posición exacta en cada momento!'];

SMD_EXPLAIN_START_Txt1 = ['El experimento comienza cuando el OVNI se muestra en algún punto de su trayectoria.'];

SMD_TELL_CLICK_Txt1 = ['Cuando esto ocurra, usa el botón izquierdo de tu cursor para indicar']
SMD_TELL_CLICK_Txt2 = ['la posición donde creas que se va a mover.'];

SMD_SHOW_CLICK_POS_Txt1 = ['Un punto rojo indicará la posición donde presionaste'];
SMD_SHOW_CLICK_POS_Txt2 = ['y el OVNI se mostrará en su nueva ubicación.'];

SMD_EXPLAIN_REPEAT_Txt1 = ['Esta secuencia se repite a lo largo de todo el experimento.'];

SMD_UFO_COLOR_Txt1 = ['Si predices adecuadamente la posición del OVNI, éste se tornará rojo.'];
SMD_UFO_COLOR_Txt2 = ['De lo contrario, no cambiará de color.'];

SMD_EXPLAIN_CIRCLE_Txt1 = ['Mientras el OVNI aparezca dentro del círculo rojo, será considerado como un acierto.'];

SM_PRACTICE_INIT        = 0
SM_PRACTICE_SHOW_UFO    = 1;
SM_PRACTICE_WAIT_CLICK  = 2;
SM_PRACTICE_SHOW_RESULT = 3;
SM_PRACTICE_END         = 4;


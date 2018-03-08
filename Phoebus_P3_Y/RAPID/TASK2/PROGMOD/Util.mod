MODULE Util
    PERS tasks task_list{2} := [ ["T_ROB1"], ["T_ROB2"]];
    
    LOCAL CONST bool DISABLE_HAND := FALSE;
    LOCAL CONST num HAND_OPEN_POS := 160;
    LOCAL CONST num HAND_DEFAULT_FORCE := 100;
           
    PROC open_hand(\num max_wait_time, \num target_pos)
        VAR num target;
        
        target := HAND_OPEN_POS;
        IF Present(target_pos) THEN
            target := target_pos;
        ENDIF
        
        IF NOT DISABLE_HAND THEN
            IF Present(target_pos) OR Hand_GetActualPos() < 150 THEN
                !Hand_JogOutward;
                IF Present(max_wait_time) THEN
                    Hand_MoveTo target, \NoWait;
                    WaitTime max_wait_time;
                ELSE
                    Hand_MoveTo target;
                ENDIF
                
            ENDIF
        ENDIF
    ENDPROC
    
    PROC close_hand(\num force)
        VAR num hold_force;
        
        IF Present(force) THEN
            hold_force := force;
        ELSE
            hold_force := HAND_DEFAULT_FORCE;
        ENDIF
        
        IF NOT DISABLE_HAND THEN
            Hand_GripInward \targetPosition:=0, \expectedForce:=hold_force;
        ENDIF
    ENDPROC
    
    PROC hand_regrip(num force)
        VAR num cur_pos;
        
        cur_pos := Hand_GetActualPos();
        IF cur_pos = 0 THEN
            cur_pos := 5;
        ENDIF
        
        Hand_MoveTo cur_pos;
        close_hand \force:=force;
    ENDPROC
    
    !=======================================
    ! Proc for PP to routine
    PROC a0_reset_from_error()
        CONST jointtarget to_init := [[6.49587,-52.3484,-12.0455,35.5865,84.451,74.7478],[30.5815,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget init_pos_:=[[38.7008,-85.1794,63.5496,100.933,12.7709,95.4244],[127.495,9E+09,9E+09,9E+09,9E+09,9E+09]];

        SingArea \Wrist;
        
        open_hand;
        MoveAbsJ to_init, v100, fine, tool1;
        MoveAbsJ init_pos_, v100, fine, tool1;
    ENDPROC
    
    PROC aa_hand_close()
        close_hand;
    ENDPROC
    
    PROC aa_hand_open()
        open_hand;
    ENDPROC
    
    PROC aa_temp_proc()
        hand_regrip 200;
    ENDPROC
    
    PROC aa_go_home_rob2()
        CONST jointtarget home_pos := [[170.373,45.4504,61.0777,293.228,135.373,183.355],[170.271,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveAbsJ home_pos, v1500, fine, tool1;
    ENDPROC
    
    PROC aa_go_ax3_rob2()
        CONST jointtarget path_p1 := [[170.487,33.0946,37.5553,293.416,135.488,185.563],[170.286,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget path_p2 := [[146.745,-15.3806,69.0363,204.539,105.951,185.549],[8.58678,9E+09,9E+09,9E+09,9E+09,9E+09]];
        Hand_Initialize \holdForce:=HAND_DEFAULT_FORCE, \Calibrate;
        ! HandCalibrateServer;
        Hand_MoveTo 160;
        MoveAbsJ path_p1, v1500, fine, tool1;
        MoveAbsJ path_p2, v1500, fine, tool1;
    ENDPROC
    
    PROC aa_go_ready_rob2()
        CONST jointtarget path_p1 := [[146.778,3.79745,1.9535,193.903,106.514,184.781],[8.61216,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget path_p2 := [[2.06349,-98.8958,48.8278,134.595,-73.9858,53.7862],[116.707,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveAbsJ path_p1, v1500, fine, tool1;
        MoveAbsJ path_p2, v1500, fine, tool1;
    ENDPROC
    
ENDMODULE
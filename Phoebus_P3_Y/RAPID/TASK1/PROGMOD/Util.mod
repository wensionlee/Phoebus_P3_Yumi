MODULE Util
    PERS tasks task_list{2} := [ ["T_ROB1"], ["T_ROB2"]];
    
    LOCAL CONST bool DISABLE_HAND := FALSE;
    LOCAL CONST num HAND_OPEN_POS := 160;
    LOCAL CONST num HAND_DEFAULT_FORCE := 100;
    
    PROC open_hand(\num max_wait_time, \num target_pos)
        VAR num target;
        
        ! get target pos
        target := HAND_OPEN_POS;
        IF Present(target_pos) THEN
            target := target_pos;
        ENDIF
        
        ! check if need move
        IF NOT DISABLE_HAND THEN
            IF Present(target_pos) OR Hand_GetActualPos() < 150 THEN
                ! move
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
        CONST jointtarget to_init := [[-31.897,-46.3973,-5.9733,-9.95711,76.4649,98.9142],[-32.1968,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget init_pos_:=[[-30.6203,-94.5968,55.174,-91.9155,17.2035,-6.24286],[-119.844,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
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
    
    PROC aa_go_home_rob1()
        CONST jointtarget home_pos := [[170.46,45.4945,56.1483,293.556,135.698,185.998],[170.491,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveAbsJ home_pos, v1500, fine, tool1;
    ENDPROC
    
    PROC aa_go_ax3_rob1()
        CONST jointtarget path_p1 := [[170.46,35.1921,32.4763,261.945,107.288,146.089],[170.491,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget path_p2 := [[170.46,35.1923,61.0162,193.522,107.288,146.089],[8.68253,9E+09,9E+09,9E+09,9E+09,9E+09]];
        !HandCalibrateServer;
        Hand_Initialize \holdForce:=HAND_DEFAULT_FORCE, \Calibrate;
        Hand_MoveTo(HAND_OPEN_POS);
        MoveAbsJ path_p1, v1500, fine, tool1;
        MoveAbsJ path_p2, v1500, fine, tool1;
    ENDPROC
    
    PROC aa_go_ready_rob1()
        CONST jointtarget path_p1 := [[117.071,-19.1603,24.9451,193.522,107.288,146.09],[8.68264,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST jointtarget path_p2 := [[-7.73346,-98.0486,31.7355,39.3585,87.4945,-134.703],[-120.766,9E+09,9E+09,9E+09,9E+09,9E+09]];
        MoveAbsJ path_p1, v1500, fine, tool1;
        MoveAbsJ path_p2, v1500, fine, tool1;
    ENDPROC
ENDMODULE
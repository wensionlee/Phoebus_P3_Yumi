MODULE MainModule
    PERS bool R_BODY_READY;
    PERS bool R_BODY_SPINED;
    PERS bool R_HAND_LEAVE;
    PERS bool R_CAP_FINISH;
    PERS bool R_BODY_GRIPPED;
    PERS bool R_BODY_GRIP_SUCCESS;
    
    PERS bool L_PILOT_FINISH;
    PERS bool L_HOLDER_READY;
    PERS bool L_BODY_PICKED;
    PERS bool L_BODY_READY;
    PERS bool L_HAND_LEAVE;

    VAR syncident END_MOVE_ON;
    VAR syncident END_MOVE_OFF;
    LOCAL CONST num END_MOVE_ID1 := 10;
    LOCAL CONST num END_MOVE_ID2 := 20;
    
    VAR syncident SPIN_MOVE_ON;
    VAR syncident SPIN_MOVE_OFF;
    LOCAL CONST num SPIN_ID_DO := 10;
    LOCAL CONST num SPIN_ID_UNDO := 20;
    LOCAL CONST num SPIN_ID_FAILED1 := 30;
    LOCAL CONST num SPIN_ID_FAILED2 := 40;
    
    
    LOCAL VAR bool HOLDER_COMBINE_SUCCESS;
    
   ! =================== Camera var =========================
    LOCAL CONST num TI_Pilot_U := 1;
    LOCAL CONST num TI_Pilot_D := 2;
    LOCAL CONST num TI_Holder := 3;
    LOCAL VAR CJ_Job job_;
    LOCAL VAR CJ_Tool ptools_{3};
    LOCAL VAR cj_ObjPos obj_pos_{3};
    LOCAL VAR bool obj_pos_updated_{3};
    LOCAL VAR string holder_absence_;
    
    ! =================== Init position ======================
    LOCAL CONST robtarget init_pos_:=[[-19.94,234.02,284.85],[0.0331142,-0.717023,-0.69535,0.0356439],[0,1,1,11],[92.4305,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    LOCAL CONST num pilot_pos_scale_ := 0.97561;
    LOCAL CONST num holder_pos_scale_ := 0.95071;
    LOCAL CONST robtarget init_pilot_u_robtarget_:=[[193.36,424.51,132.12],[0.00296032,0.71032,0.703872,-0.00183421],[-1,1,0,11],[147.418,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_pilot_u_objpos_ := [100.5,-10.6,358.531067];
    LOCAL CONST robtarget init_pilot_d_robtarget_:=[[76.61,436.49,131.79],[0.00277157,0.702463,-0.711716,0.000580868],[-1,1,-2,11],[92.6196,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_pilot_d_objpos_ := [-41.5,-14.342066,358.386292];
    LOCAL CONST robtarget init_holder_robtarget_:=[[161.52,432.03,163.10],[0.000442375,1,-0.000829869,0.000861587],[-1,1,-3,11],[132.1,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_holder_objpos_ := [26.9,-2.6,359.215118];
    
    
   
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !GLOBAL VARIABLES
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
   
    VAR num x;

    !//Robot configuration
    PERS tooldata currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS wobjdata currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
!    CONST jointtarget home_pos := [[170.373,45.4504,61.0777,293.228,135.373,183.355],[170.271,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PERS speeddata currentSpeed;
    PERS zonedata currentZone;

    !//PC communication
    VAR socketdev clientSocket;
    VAR socketdev serverSocket;
    VAR num instructionCode;
    VAR num params{10};
    VAR num nParams;

    PERS string ipController:="192.168.125.1";
    !robot default IP
    !PERS string ipController:= "127.0.0.1"; !local IP for testing in simulation
    VAR num serverPort:=5000;

    !//Motion of the robot
    VAR robtarget cartesianTarget;
    VAR jointtarget jointsTarget;
    VAR bool moveCompleted;
    !Set to true after finishing a Move instruction.

    !//Buffered move variables
    CONST num MAX_BUFFER:=512;
    VAR num BUFFER_POS:=0;
    VAR robtarget bufferTargets{MAX_BUFFER};
    VAR speeddata bufferSpeeds{MAX_BUFFER};

    !//External axis position variables
    VAR extjoint externalAxis;

    !//Circular move buffer
    VAR robtarget circPoint;

    !//Correct Instruction Execution and possible return values
    VAR num ok;
    VAR bool should_send_res;
    CONST num SERVER_BAD_MSG:=0;
    CONST num SERVER_OK:=1;

    !//Robot Constants
    CONST jointtarget jposHomeYuMiL:=[[0,-130,30,0,40,0],[135,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PERS tasks tasklistArms{2}:=[["T_ROB_L"],["T_ROB_R"]];
    VAR syncident Sync_Start_Arms;
    VAR syncident Sync_Stop_Arms;
    CONST confdata L_CONF := [-1,-1,0,11];
    VAR string str_data;
     VAR string str_data1;
    CONST robtarget p11:=[[412.45,63.43,230.73],[0.0264407,-0.023321,-0.705188,0.708144],[-1,-1,0,0],[-91.1388,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p21:=[[79.78,477.73,302.50],[0.427727,-0.692755,0.521034,0.256254],[0,-2,1,11],[102.529,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p31:=[[53.54,554.74,302.54],[0.426881,-0.6917,0.519561,0.263404],[0,-2,1,11],[102.532,9E+09,9E+09,9E+09,9E+09,9E+09]];
!    CONST robtarget p11:=[[-140.63,360.74,583.76],[0.215646,-0.695059,0.360686,0.583349],[0,-2,0,11],[108.39,9E+09,9E+09,9E+09,9E+09,9E+09]]; 
    
    

    LOCAL PROC aa_TechPos ()
        MoveL init_pilot_u_robtarget_,v1000, fine, tool1;
        MoveL init_pilot_d_robtarget_,v1000, fine, tool1;
        SingArea \Wrist;
        MoveL init_holder_robtarget_,v1000, fine, tool1;
    ENDPROC
    
    LOCAL PROC InitCamareVar()
        ! job
        cj_InitCJJobRecord job_;
        job_.cam_id := 2;
        job_.job_name := "PenLeft.job";
        job_.ip_addr := "192.168.10.209";
        job_.port := 23;
        job_.user_name := "admin";
        job_.user_passwd := "";
        job_.max_trigger_time := 10000;
        
        ! Tool pilot up
        cj_InitCJToolRecord ptools_{TI_Pilot_U};
        ptools_{TI_Pilot_U}.tool_name := "PenPilotUp";
        ptools_{TI_Pilot_U}.enabled_tags := "PenPilotUP";
        ptools_{TI_Pilot_U}.need_trigger := TRUE;
        
        ! Tool pilot down
        cj_InitCJToolRecord ptools_{TI_Pilot_D};
        ptools_{TI_Pilot_D}.tool_name := "PenPilotDown";
        ptools_{TI_Pilot_D}.enabled_tags := "PenPilotDown";
        ptools_{TI_Pilot_D}.need_trigger := TRUE;
        
        ! Tool holder
        cj_InitCJToolRecord ptools_{TI_Holder};
        ptools_{TI_Holder}.tool_name := "PenHolder";
        ptools_{TI_Holder}.enabled_tags := "PenHolderCase|hole1|hole2|hole3|hole4|hole5|hole6|hole7|hole8|hole9|hole10";
        ptools_{TI_Holder}.need_trigger := TRUE;

    ENDPROC
    
    LOCAL PROC GetObjPos()
        VAR bool result;
        VAR string str_out{1};
        VAR num data_num;


        result := cj_TriggerTools(job_, ptools_);


        FOR i FROM 1 TO Dim(ptools_, 1) DO
            IF ptools_{i}.need_trigger THEN
                data_num := cj_GetToolData(job_, ptools_{i}, str_out, \max_data_len:=1);
                IF NOT data_num = 0 THEN
                     result := cj_StrToObjPos(str_out{1}, obj_pos_{i});
                     obj_pos_updated_{i} := TRUE;
                ENDIF
            ENDIF
        ENDFOR

    ENDPROC

    LOCAL PROC WaitObjPos(num obj_id)
        ! Init
        move_init;
        ptools_{obj_id}.need_trigger := TRUE;
        obj_pos_updated_{obj_id} := FALSE;
        
        WaitRob \InPos;
        ! try get obj pos
        GetObjPos;
        ! retry if need
        IF NOT obj_pos_updated_{obj_id} THEN
            TPWrite "Could not get obj pos, retry...";
            WHILE obj_pos_updated_{obj_id} = FALSE DO
                WaitTime(2);
                GetObjPos;
            ENDWHILE
        ENDIF
        
        ptools_{obj_id}.need_trigger := FALSE;
    ENDPROC
    
    LOCAL FUNC bool HasOneObjPosUpdated(num obj_ids{*})
        FOR i FROM 1 TO Dim(obj_ids, 1) DO
            IF obj_pos_updated_{obj_ids{i}} THEN
                RETURN TRUE;
            ENDIF
        ENDFOR
        
        RETURN FALSE;
    ENDFUNC
    
    LOCAL PROC WaitOneObjPos(num obj_ids{*})
        ! Init
        move_init;
        FOR i FROM 1 TO Dim(obj_ids, 1) DO
            ptools_{obj_ids{i}}.need_trigger := TRUE;
            obj_pos_updated_{obj_ids{i}} := FALSE;
        ENDFOR

        
        ! try get obj pos
        WaitRob \InPos;
        GetObjPos;
 
        ! retry if need
        IF NOT HasOneObjPosUpdated(obj_ids) THEN
            TPWrite "Could not get obj pos, retry...";
            WHILE NOT HasOneObjPosUpdated(obj_ids) DO
                WaitTime(2);
                GetObjPos;
            ENDWHILE
        ENDIF
        
        FOR i FROM 1 TO Dim(obj_ids, 1) DO
            ptools_{obj_ids{i}}.need_trigger := FALSE;
        ENDFOR
    ENDPROC
    
    LOCAL PROC WaitHolder()
        VAR string str_out {1};
        VAR num data_num;
        
        ptools_{TI_Holder}.need_trigger := TRUE;
        ! GetObjPos;
        
        WHILE TRUE DO
            data_num := cj_GetToolData(job_, ptools_{TI_Holder}, str_out, \beg_result_pos:=2, \max_data_len:=1);
            IF NOT data_num = 0 THEN
                IF StrMatch(str_out{1}, 1, "1") <= StrLen(str_out{1}) THEN
                    holder_absence_ := str_out{1};
                    ptools_{TI_Holder}.need_trigger := FALSE;
                    RETURN;
                ENDIF
            ENDIF

            TPWrite "Could not get holder pos, retry...";
            GetObjPos;
        ENDWHILE
        
    ENDPROC
    
    
    PROC main()
        VAR bool result;
        
!        WHILE TRUE DO
!        ENDWHILE
        ConfL \Off;
        SingArea \Wrist;
        !a0_reset_from_error;
        InitCamareVar;
        result := cj_Init(job_, ptools_);
        
        WHILE TRUE DO
            combine_pen;
        ENDWHILE
        
    ENDPROC
    
    LOCAL PROC move_init()
        MoveJ init_pos_, v1000, fine, tool1;
    ENDPROC
    
    LOCAL PROC reset_bool_flag()
        L_PILOT_FINISH := FALSE;
        L_HOLDER_READY := FALSE;
        L_BODY_PICKED := FALSE;
        L_BODY_READY := FALSE;
        L_HAND_LEAVE := FALSE;
    ENDPROC
    
    LOCAL PROC combine_pen()
        VAR bool result;
        VAR num status;
    
        ! Init
        reset_bool_flag;
        ptools_{TI_Holder}.need_trigger := FALSE;
        ptools_{TI_Pilot_U}.need_trigger := FALSE;
        ptools_{TI_Pilot_D}.need_trigger := FALSE;
  
        ! combine pilot
        WaitOneObjPos([TI_Pilot_U, TI_Pilot_D]);
        IF obj_pos_updated_{TI_Pilot_U} THEN
            pick_pilot_up;
        ELSEIF obj_pos_updated_{TI_Pilot_D} THEN
            pick_pilot_down;
        ENDIF
        
        combine_pilot;
        
        ! combine holder
        WaitObjPos(TI_Holder);
        WaitHolder;
        pick_holder;
        combine_holder;
        
        move_init;
        ! wait end
        SyncMoveOn END_MOVE_ON, task_list;
            MoveL Offs(init_pos_, 0, 0, -30) \ID:=END_MOVE_ID1, v1000, fine, tool1;
            reset_bool_flag;
            MoveL init_pos_ \ID:=END_MOVE_ID2, v1000, fine, tool1;
        SyncMoveOff END_MOVE_OFF;
    ENDPROC

    LOCAL PROC pick_pilot_up()
        VAR robtarget pick_pos;
        VAR num x_off;
        VAR num y_off;
        VAR num a_off;

        ! pick object
        x_off := (obj_pos_{TI_Pilot_U}.x - init_pilot_u_objpos_.x) * pilot_pos_scale_;
        y_off := (obj_pos_{TI_Pilot_U}.y - init_pilot_u_objpos_.y) * pilot_pos_scale_;
        a_off := obj_pos_{TI_Pilot_U}.angle- init_pilot_u_objpos_.angle;
       
        pick_pos := Offs(init_pilot_u_robtarget_, x_off, y_off, 0);
     
        pick_pos := RelTool(pick_pos, 0, 0, 0 \Rz:=a_off);
        
        open_hand \target_pos:=55;
        MoveL Offs(pick_pos, 0, 0, 100), vmax, z20, tool1;
        MoveL pick_pos, v500, fine, tool1;
        close_hand;
        MoveL Offs(pick_pos, 155, 0, 0), v200, z5, tool1;
        MoveL Offs(pick_pos, 155, 0, 100), v1000, z20, tool1;
        
    ENDPROC

    LOCAL PROC pick_pilot_down()
        CONST robtarget to_pick_pos:=[[11.68,306.94,232.19],[0.0486431,-0.0280322,-0.998421,0.00238224],[0,1,-1,11],[92.4459,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget out_pos:=[[166.30,380.98,272.57],[0.0326295,0.226672,0.961245,-0.153504],[0,0,0,11],[-172.498,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR robtarget pick_pos;
        VAR num x_off;
        VAR num y_off;
        VAR num a_off;

        ! pick object
        x_off := (obj_pos_{TI_Pilot_D}.x - init_pilot_d_objpos_.x) * pilot_pos_scale_;
        y_off := (obj_pos_{TI_Pilot_D}.y - init_pilot_d_objpos_.y) * pilot_pos_scale_;
        a_off := obj_pos_{TI_Pilot_D}.angle- init_pilot_d_objpos_.angle;
        pick_pos := Offs(init_pilot_d_robtarget_, x_off, y_off, 0);
        pick_pos := RelTool(pick_pos, 0, 0, 0 \Rz:=a_off);
        
        open_hand \target_pos:=55;
        MoveL to_pick_pos, vmax, z20, tool1;
        MoveL Offs(pick_pos, 0, 0, 30), vmax, z20, tool1;
        MoveL pick_pos, v500, fine, tool1;
        close_hand;
        MoveL Offs(pick_pos, -150, 0, 0), v200, z5, tool1;
        MoveL Offs(pick_pos, -150, 0, 100), v1000, z20, tool1;
        MoveJ out_pos, v1500, z20, tool1;
        
    ENDPROC
    
    LOCAL PROC pick_holder()
        CONST robtarget to_pick_pos:=[[82.44,355.08,267.51],[0.000531081,-0.26961,0.96297,-0.000702183],[0,0,-1,11],[106.072,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR robtarget pick_pos;
        VAR num first_holder;
        VAR num hole_x_off;
        VAR num hole_y_off;
        VAR num case_x_off;
        VAR num case_y_off;
        
        first_holder := StrMatch(holder_absence_, 1, "1");
        IF first_holder > 10 THEN
            TPWrite "Could not find holder, please reset pp";
            WHILE TRUE DO
                ! dead loop
            ENDWHILE
        ENDIF
        
        case_x_off := (obj_pos_{TI_Holder}.x - init_holder_objpos_.x) * holder_pos_scale_;
        case_y_off := (obj_pos_{TI_Holder}.y - init_holder_objpos_.y) * holder_pos_scale_;
        
        ! reference hole is hole 3
        IF first_holder > 5 THEN
            hole_x_off := -50;
            hole_y_off := (3 - (first_holder - 5)) * 27;
        ELSE
            hole_x_off := 0;
            hole_y_off := (3 - first_holder) * 27;
        ENDIF
        
        pick_pos := Offs(init_holder_robtarget_, case_x_off + hole_x_off, case_y_off + hole_y_off, 0);
        
        open_hand;
        ConfL \On;
        MoveL to_pick_pos, vmax, z20, tool1;
        MoveL Offs(pick_pos, 0, 0, 50), vmax, z20, tool1;
        MoveL pick_pos, v500, fine, tool1;
        ConfL \Off;
        close_hand;
        MoveL Offs(pick_pos, 0, 0, 50), v1000, z20, tool1;
    UNDO
        ConfL \Off;
    ENDPROC
    
    LOCAL PROC combine_pilot()
        CONST robtarget to_wait_pos:=[[410.47,159.72,439.07],[0.534571,0.567127,0.626579,-0.000882242],[-1,0,0,11],[-169.313,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_pos:=[[391.33,-85.32,471.21],[0.723445,0.690383,-0.000491051,0.000652106],[-1,0,0,11],[165.173,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos:=[[391.73,-87.05,418.33],[0.720698,0.69325,-0.000409404,0.000589926],[-1,0,1,11],[165.184,9E+09,9E+09,9E+09,9E+09,9E+09]];

        ! move to combine pos
        MoveL to_wait_pos, v1500, z50, tool1;
        MoveL wait_pos, v1500, fine, tool1;
        
        ! combine
        WaitUntil (R_BODY_READY = TRUE);
        MoveL combine_pos, v50, fine, tool1;
        open_hand;
        
        MoveL to_wait_pos, v1500, z50, tool1;
        L_PILOT_FINISH := TRUE;
        move_init;
        
    ERROR
        TPWrite "err";
    ENDPROC
    
    LOCAL PROC combine_holder()
 
        CONST robtarget to_wait_pos:=[[344.71,219.96,275.59],[0.493562,0.869608,-0.00734886,0.0112189],[-1,0,-3,11],[-170.324,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_pos:=[[393.38,59.49,230.49],[0.490599,0.53066,0.505697,-0.471153],[-1,-1,-3,11],[-168.059,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos1:=[[395.70,20.72,230.50],[0.490633,0.530633,0.505722,-0.471121],[-1,-1,-3,11],[-168.053,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos2:=[[395.69,20.74,231.20],[0.499651,0.522085,0.496792,-0.480598],[-1,-1,-3,11],[-168.056,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos3:=[[395.68,2.52,231.20],[0.499645,0.5221,0.496775,-0.480606],[-1,-1,-3,11],[-168.058,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos_ready:=[[395.69,1.8,230.4],[0.497604,0.500661,0.500703,-0.501026],[-1,-1,-3,11],[-168.047,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        CONST robtarget spin_beg_pos := combine_pos_ready;
        VAR robtarget spin_end_pos := [[395.70,-1,230.40],[0.505705,0.508761,0.492481,-0.492838],[-1,-1,1,11],[-168.047,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_failed_pos_move := [[537.51,13.77,134.26],[0.489641,0.492939,0.508379,-0.508739],[-2,-1,-3,11],[-157.199,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_failed_pos_spin := [[537.52,13.76,134.25],[0.705763,0.707696,-0.0243831,0.0217027],[-2,-1,-2,11],[-157.233,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        CONST robtarget body_pick_pos:=[[394.77,-7.98,211.81],[0.000317753,-1,-0.000738117,-0.000344675],[-2,1,0,11],[-176.608,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget try_drop_holder_pos := [[467.22,119.78,168.04],[0.71461,0.699515,-0.00166773,0.00312769],[-1,-1,2,11],[-170.325,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_cap_pos:=[[462.13,-58.99,142.93],[0.000754405,-0.00454344,0.709494,-0.704697],[-2,-1,0,11],[-154.734,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        CONST robtarget release_pos := [[488.31,454.82,108.08],[0.0157331,-0.764597,0.644066,0.0180238],[-1,0,-2,11],[-166.74,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget to_back_pos := [[306.30,430.93,307.14],[0.0156531,-0.76461,0.644052,0.0180347],[-1,0,-2,11],[-135.405,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        CONST num step_per_spin := 0.6;
        CONST num step_num := 3;
        
        spin_end_pos.trans := spin_beg_pos.trans;
        
        ! wait combine
        MoveL to_wait_pos, v1000, z50, tool1;
        MoveL wait_pos, v1000, fine, tool1;
        WaitUntil(R_BODY_READY = TRUE);
        
        ! combine
        MoveL combine_pos1, v20, z0, tool1;
        hand_regrip 200;
        
        ConfL \On;
        SingArea \Wrist;
        MoveL combine_pos2, v20, z0, tool1;
        MoveL combine_pos3, v20, z0, tool1;
        MoveL combine_pos_ready, v20, fine, tool1;
        WaitRob \InPos;
        L_HOLDER_READY := TRUE;
        
        !codes for modify position
        !MoveL spin_end_pos, vmax, fine, tool1;
        !MoveL spin_beg_pos, vmax, fine, tool1;
        !MoveL spin_failed_pos_move, vmax, fine, tool1; 
        !MoveL spin_failed_pos_spin, vmax, fine, tool1; 
        
        ! hold the pen body
        SyncMoveOn SPIN_MOVE_ON, task_list;
            FOR i FROM 1 TO (step_num - 1) DO
                MoveL Offs(spin_end_pos, 0, -step_per_spin, 0) \ID:=SPIN_ID_DO,vmax, fine, tool1;
                MoveL spin_beg_pos \ID:=SPIN_ID_UNDO,vmax, fine, tool1;
!                IF i = 1 THEN
!                    hand_regrip 200;
!                ENDIF
                WaitUntil R_BODY_GRIPPED;
                
                ! Check if right hand gripped success
                IF NOT R_BODY_GRIP_SUCCESS THEN
                    GOTO SPIN_FAILED;
                ENDIF
                
            ENDFOR
            
            GOTO SPIN_LAST;
            
        SPIN_FAILED:
            open_hand;
            MoveL spin_failed_pos_move \ID:=SPIN_ID_FAILED1, v200, fine, tool1;
            MoveL spin_failed_pos_spin \ID:=SPIN_ID_FAILED2, v200, fine, tool1;
            MoveL spin_end_pos \ID:=SPIN_ID_UNDO, v200, fine, tool1;
            HOLDER_COMBINE_SUCCESS := FALSE;
            GOTO SPIN_END;
                
        SPIN_LAST:
            MoveL Offs(spin_end_pos, 0, -step_per_spin, 0) \ID:=SPIN_ID_DO,vmax, fine, tool1;
            HOLDER_COMBINE_SUCCESS := TRUE;
            GOTO SPIN_END;
            
        SPIN_END:
            ! nothing
            
        SyncMoveOff SPIN_MOVE_OFF;
        
        ConfL \Off;
        
        IF NOT HOLDER_COMBINE_SUCCESS THEN
            a0_reset_from_error;
            RETURN;
        ENDIF
        
        WaitUntil(R_BODY_SPINED = TRUE);
        !Hand_MoveTo(30);
        open_hand;
        MoveL wait_pos, v100, fine, tool1;
        !open_hand;
        MoveL body_pick_pos, v100, fine, tool1;
        close_hand \force:=50;
        IF Hand_GetActualPos() < 10 THEN
            ! Failed
            open_hand;
            ! continue
        ENDIF
        
        L_BODY_PICKED := TRUE;
        
        ! wait hand leave and spin
        WaitUntil(R_HAND_LEAVE = TRUE);
        
        ! Rotate pen to make some failed holder drop
        ConfL \On;
        FOR i FROM 1 TO 2 DO
            MoveL Offs(try_drop_holder_pos, 0, 0, 30), v1000, z0, tool1;
            MoveL try_drop_holder_pos, v1000, z0, tool1;
        ENDFOR
        MoveL Offs(try_drop_holder_pos, 0, 0, 150), v1000, z1, tool1;
        WaitTime 1;
        
        ! wait to combine
        MoveL Offs(wait_cap_pos, 0, 50, 50), v200, z5, tool1;
        MoveL Offs(wait_cap_pos, 0, 0, 50), v200, z5, tool1;
        
        ConfL \Off;
        MoveL wait_cap_pos, v100, fine, tool1;
        WaitRob \InPos;
        L_BODY_READY := TRUE;
        
        ! wait cap 
        WaitUntil(R_CAP_FINISH = TRUE);
        
        ! release pen
        MoveL Offs(wait_cap_pos, 0, 0, 50), v1000, z20, tool1;
        MoveL Offs(release_pos, 0,0,50), v1000, z10, tool1;
        MoveL release_pos, v200, fine, tool1;
        open_hand;
        MoveL Offs(release_pos, 0,0,50), v1000, z10, tool1;
        ! back up init pos
        !MoveL to_wait_pos, v1000, z50, tool1;
        WaitRob \InPos;
        L_HAND_LEAVE := TRUE;
        
        MoveL to_back_pos, v1500, z10, tool1;
        move_init;
    UNDO
        ConfL \Off;
    ENDPROC
    
    
   


    
!    VAR string str_data;
    

    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !LOCAL METHODS
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    !//Method to parse the message received from a PC
    !// If correct message, loads values on:
    !// - instructionCode.
    !// - nParams: Number of received parameters.
    !// - params{nParams}: Vector of received params.
    
   
   
    PROC ParseMsg(string msg)
        !//Local variables
        VAR bool auxOk;
        VAR num ind:=1;
        VAR num newInd;
        VAR num length;
        VAR num indParam:=1;
        VAR string subString;
        VAR bool end:=FALSE;

        !//Find the end character
        length:=StrMatch(msg,1,"#");
        IF length>StrLen(msg) THEN
            !//Corrupt message
            nParams:=-1;
        ELSE
            !//Read Instruction code
            newInd:=StrMatch(msg,ind," ")+1;
            subString:=StrPart(msg,ind,newInd-ind-1);
            auxOk:=StrToVal(subString,instructionCode);
            ! ASG: set instructionCode here!
            IF auxOk=FALSE THEN
                !//Impossible to read instruction code
                nParams:=-1;
            ELSE
                ind:=newInd;
                !//Read all instruction parameters (maximum of 8)
                WHILE end=FALSE DO
                    newInd:=StrMatch(msg,ind," ")+1;
                    IF newInd>length THEN
                        end:=TRUE;
                    ELSE
                        subString:=StrPart(msg,ind,newInd-ind-1);
                        auxOk:=StrToVal(subString,params{indParam});
                        indParam:=indParam+1;
                        ind:=newInd;
                    ENDIF
                ENDWHILE
                nParams:=indParam-1;
            ENDIF
        ENDIF
    ENDPROC
    
    FUNC bool isPoseReachable(robtarget pose, PERS tooldata tool, PERS wobjdata wobj)
        VAR bool reachable := True;
        VAR jointtarget joints;
        
        joints := CalcJointT(pose, tool, \WObj:=wobj);
        RETURN reachable;

        ERROR
            reachable := FALSE;
            TRYNEXT;
    ENDFUNC
    
    FUNC bool isJointsReachable(jointtarget joints, PERS tooldata tool, PERS wobjdata wobj)
        VAR bool reachable := True;
        VAR robtarget pose;
        
        pose := CalcRobT(joints, tool \Wobj:=wobj);
        cartesianTarget := pose;
        RETURN reachable;

        ERROR
            reachable := FALSE;
            TRYNEXT;            
    ENDFUNC
        
    !//Handshake between server and client:
    !// - Creates socket.
    !// - Waits for incoming TCP connection.
    PROC ServerCreateAndConnect(string ip,num port)
        VAR string clientIP;

        SocketCreate serverSocket;
        SocketBind serverSocket,ip,port;
        SocketListen serverSocket;

        !! ASG: while "current socket status of clientSocket" IS NOT EQUAL TO the "client connected to a remote host"
        WHILE SocketGetStatus(clientSocket)<>SOCKET_CONNECTED DO
            SocketAccept serverSocket,clientSocket\ClientAddress:=clientIP\Time:=WAIT_MAX;
            !//Wait 0.5 seconds for the next reconnection
            WaitTime 0.5;
        ENDWHILE
    ENDPROC

    !//Parameter initialization
    !// Loads default values for
    !// - Tool.
    !// - WorkObject.h
    !// - Zone.
    !// - Speed.
    PROC Initialize()
        currentTool:=[TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
        currentWobj:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[0,0,0],[1,0,0,0]]];
        currentSpeed:=[1000,1000,1000,1000];
        !currentZone:=[FALSE,0.3,0.3,0.3,0.03,0.3,0.03]; 
        currentZone:=fine;
        !z0

        !Find the current external axis values so they don't move when we start
        jointsTarget:=CJointT();
        externalAxis:=jointsTarget.extax;
    ENDPROC

    FUNC string FormateRes(string clientMessage)
        VAR string message;
        
        message:=NumToStr(instructionCode,0);
        message:=message+" "+NumToStr(ok,0);
        message:=message+" "+ clientMessage;

        RETURN message;        
    ENDFUNC 
    
    PROC main_myo()
     VAR robtarget p1test;
      VAR num StartBit1;
      VAR num LenBit1;
      VAR num  EndBit1;
      
      VAR num StartBit2;
      VAR num LenBit2;
      VAR num  EndBit2;
      
      VAR num StartBit3;
      VAR num LenBit3;
      VAR num  EndBit3;
      
      VAR num StartBit4;
      VAR num LenBit4;
      VAR num  EndBit4;
        
        VAR string  s1;
        VAR string  s2;
        VAR string  s3;
         VAR string  s4;
         
        VAR   bool flag1;
        
       VAR num delta_x;
       VAR num delta_y;
       VAR num delta_z;
      VAR num hand;
    VAR progdisp progdisp1;
        
        !//Local variables
        VAR string receivedString;
        !//Received string
        VAR string sendString;
        !//Reply string
        VAR string addString;
        !//String to add to the reply.
        VAR bool connected;
        !//Client connected
        VAR bool reconnected;
        !//Reconnect after sending ack
        VAR bool reconnect;
        !//Drop and reconnection happened during serving a command
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;
         VAR num i;
        !//Motion configuration
        ConfL\Off;
        SingArea\Wrist;
        moveCompleted:=TRUE;
        MoveL p11, v50, z50, tool1;
        !//Initialization of WorkObject, Tool, Speed and Zone
        Initialize;
        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort;
        
        connected:=TRUE;
        reconnect:=FALSE;
        
        !//Server Loop
        WHILE TRUE DO
            
            !//For message sending post-movement
            should_send_res:=TRUE;
            
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Has communication dropped after receiving a command?
            addString:="";
           !?????????????


            !//Wait for a command
            SocketReceive clientSocket \Str := str_data;
            TPWrite str_data ;   
          !     str_data:="-2.28,0.439,1.333\0D";
        StartBit1:=1;
       EndBit1:=StrFind(str_data,StartBit1,",");
       LenBit1:=EndBit1-StartBit1;
       
        StartBit2:=EndBit1+1;
       EndBit2:=StrFind(str_data,StartBit2,",");
       LenBit2:=EndBit2-StartBit2;
       
        StartBit3:=EndBit2+1;
       EndBit3:=StrFind(str_data,StartBit3,",");
       LenBit3:=EndBit3-StartBit3;
       
       StartBit4:=EndBit3+1;
       EndBit4:=StrFind(str_data,StartBit4,"\0D");
       LenBit4:=EndBit4-StartBit4;
       
       
       s1:=StrPart(str_data,StartBit1,LenBit1);
       s2:=StrPart(str_data,StartBit2,LenBit2);
       s3:=StrPart(str_data,StartBit3,LenBit3);
       s4:=StrPart(str_data,StartBit4,LenBit4);
       
         flag1:=StrToVal(s1,delta_x);
         flag1:=StrToVal(s2,delta_y);
         flag1:=StrToVal(s3,delta_z);
         flag1:=StrToVal(s4,hand); 
         
        TPWrite "delta_x:"\Num:=delta_x;
        TPWrite "delta_y:"\Num:=delta_y;
        TPWrite "delta_z:"\Num:=delta_z;
        TPWrite "hand:"\Num:=hand;
        !p1test:=[[delta_x,delta_y,delta_z],[0.427727,-0.692755,0.521034,0.256254],[0,-2,1,11],[102.529,9E+09,9E+09,9E+09,9E+09,9E+09]];
     !FOR i FROM 1 TO 5 DO
       ! MoveL p11, v50, z50, tool1;
        !WaitTime 2;
        
        MoveL Offs(p11,delta_x,delta_y,delta_z), v50, z50, tool1;
         IF hand=1 THEN
               Hand_GripOutward;
                
           
            ELSE IF hand=0
                 Hand_GripInward;
         ENDIF
        !WaitTime 2;
        !MoveL p21, v50, z50, tool1;
        
        !MoveL p1test, v50, z50, tool1;
        
!        MoveL p21, v50, z50, tool1;
!        MoveL p11, v1000, z50, tool1;
     !ENDFOR
 !      main_1;reconnected:=FALSE;

            !//Execution of the command
            !---------------------------------------------------------------------------------------------------------------
            TEST instructionCode
            CASE 0:
                !Ping
                IF nParams=0 THEN
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 1:
                !Cartesian Move
               
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        ClkReset clock1;
                        ClkStart clock1;
                        
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        ClkStop clock1;
                        reg1:=ClkRead(clock1);
                        addString:=NumToStr(reg1,5);
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 2:
                !Joint Move
                IF nParams=7 THEN
                    externalAxis.eax_a:=params{7};
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;
                    ClkReset clock1;
                    ClkStart clock1;
                    MoveAbsJ jointsTarget,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    ClkStop clock1;
                    reg1:=ClkRead(clock1);
                    addString:=NumToStr(reg1,5);
                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 3:
                !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams=0 THEN
                    cartesianPose:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                    addString:=NumToStr(cartesianPose.trans.x,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.y,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.z,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q1,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q2,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q3,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q4,3);
                    !End of string	
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 4:
                !Get Joint Coordinates
                IF nParams=0 THEN
                    jointsPose:=CJointT();
                    addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_6,2)+" ";
                    addString:=addString+NumToStr(jointsTarget.extax.eax_a,2);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 5:
                !Cartesian Move, nonlinear movement
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        ClkReset clock1;
                        ClkStart clock1;
                        MoveJ cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        ClkStop clock1;
                        reg1:=ClkRead(clock1);
                        addString:=NumToStr(reg1,5);
                        moveCompleted:=TRUE;
                   ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 6:
                !Set Tool
                IF nParams=7 THEN
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 7:
                !Set Work Object
                IF nParams=7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 8:
                !Set Speed of the Robot
                IF nParams=4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
                    ok:=SERVER_OK;
                ELSEIF nParams=2 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 9:
                !Set zone data
                IF nParams=4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep:=TRUE;
                        currentZone.pzone_tcp:=0.0;
                        currentZone.pzone_ori:=0.0;
                        currentZone.zone_ori:=0.0;
                    ELSE
                        currentZone.finep:=FALSE;
                        currentZone.pzone_tcp:=params{2};
                        currentZone.pzone_ori:=params{3};
                        currentZone.zone_ori:=params{4};
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 11:
                !Cartesian Move (synchronized)
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;

                        SyncMoveOn Sync_Start_Arms,tasklistArms;
                        MoveL cartesianTarget\ID:=11,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        SyncMoveOff Sync_Stop_Arms;

                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 12:
                !Joint Move (synchronized)
                IF nParams=7 THEN
                    externalAxis.eax_a:=params{7};
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;

                    SyncMoveOn Sync_Start_Arms,tasklistArms;
                    MoveAbsJ jointsTarget\ID:=12,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    SyncMoveOff Sync_Stop_Arms;

                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 13:
                !Relative Cartesian Move
                IF nParams=3 THEN
                    cartesianTarget:=Offs(CRobT(),params{1},params{2},params{3});

                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                    
                ELSEIF nParams=6 THEN
                    cartesianTarget:=RelTool(CRobT(),params{1},params{2},params{3},\Rx:=params{4}\Ry:=params{5}\Rz:=params{6});

                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 14:
                !ContactL //NOTE: NOT IMPLEMENTED ON PYTHON
                ! Desired Torque Stated
                IF nParams=8 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    ok:=SERVER_OK;
!                    ContactL\DesiredTorque:=params{8},cartesianTarget,v100,\Zone:=currentZone,currentTool,\WObj:=currentWobj;
                    ! Desired Torque Not Stated
                    ! Instruction will only raise the collision detection level and not supervise the internal torque level
                ELSEIF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    ok:=SERVER_OK;
!                    ContactL cartesianTarget,v100,\Zone:=currentZone,currentTool,\WObj:=currentWobj;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 20:
                !Gripper Close
                IF nParams=0 THEN
                    Hand_GripInward;
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripInward;!\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripInward;!\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 21:
                !Gripper Open
                IF nParams=0 THEN
                    Hand_GripOutward;
                    ok:=SERVER_OK;

                ELSEIF nParams=1 THEN
                    Hand_GripOutward;!\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripOutward;!\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripOutward;!\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 22:
                ! Initialize gripper with specified values

                ! calibrate only
                IF nParams=0 THEN
                    Hand_Initialize\Calibrate;
                    ok:=SERVER_OK;

                    ! set maxSpeed, holdForce, physicalLimit (0-25 mm), and calibrate                    
                ELSEIF nParams=3 THEN
                    Hand_Initialize;!\maxSpd:=params{1}\holdForce:=params{2}\phyLimit:=params{3}\Calibrate;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 23:
                ! Set Max Speed
                IF nParams=1 THEN
                    Hand_SetMaxSpeed params{1};
                    ! between 0-20 mm/s 
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 24:
                ! Set gripping force 
                IF nParams=1 THEN
                    Hand_SetHoldForce params{1};
                    ! between 0-20 Newtons
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 25:
                ! Move the gripper to a specified position 
                IF nParams=1 THEN
                    Hand_MoveTo params{1};
                    ! between 0-25 mm or 0-phyLimit if phyLimit is set in CASE 22
                    ok:=SERVER_OK;

                ELSEIF nParams=2 THEN
                    Hand_MoveTo params{1}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 26:
                !Get Gripper Width
                IF nParams=0 THEN
                    addString:=NumToStr(Hand_GetActualPos(),2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 29:
                ! Stop any action of the gripper (motors will lose power)
                IF nParams=0 THEN
                    Hand_Stop;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 30:
                !Add Cartesian Coordinates to buffer
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        L_CONF,
                                        externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        IF BUFFER_POS<MAX_BUFFER THEN
                            BUFFER_POS:=BUFFER_POS+1;
                            bufferTargets{BUFFER_POS}:=cartesianTarget;
                            bufferSpeeds{BUFFER_POS}:=currentSpeed;
                        ENDIF
                        ok:=SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 31:
                !Clear Cartesian Buffer
                IF nParams=0 THEN
                    BUFFER_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 32:
                !Get Buffer Size)
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 33:
                !Execute moves in cartesianBuffer as linear moves
                IF nParams=0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO
                        MoveL bufferTargets{i},bufferSpeeds{i},currentZone,currentTool\WObj:=currentWobj;
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 34:
                !External Axis move
                IF nParams=6 THEN
                    externalAxis:=[params{1},params{2},params{3},params{4},params{5},params{6}];
                    jointsTarget:=CJointT();
                    jointsTarget.extax:=externalAxis;
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;
                    MoveAbsJ jointsTarget,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
                TPWrite "Execution started";
            CASE 35:
                !Specify circPoint for circular move, and then wait on toPoint
                IF nParams=7 THEN
                    circPoint:=[[params{1},params{2},params{3}],
                                [params{4},params{5},params{6},params{7}],
                                [0,0,0,0],
                                externalAxis];
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 36:
                !specify toPoint, and use circPoint specified previously
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    MoveC circPoint,cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 40:
                !returns 1 if given pose is reachable. 0 other wise.
                IF nParams=7 THEN
                    cartesianTarget := [[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [-1,-1,0,11],
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        addString := "1";
                    ELSE
                        TPWrite "not reachable";
                        addString := "0";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 41:
                !returns 1 if given joint configuration is reachable. 0 other wise.
                IF nParams=7 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    IF isJointsReachable(jointsTarget, currentTool, currentWobj) THEN
                        addString := "1";
                    ELSE
                        addString := "0";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 98:
                !returns current robot info: serial number, robotware version, and robot type
                IF nParams=0 THEN
                    addString:=GetSysInfo(\SerialNo)+"*";
                    addString:=addString+GetSysInfo(\SWVersion)+"*";
                    addString:=addString+GetSysInfo(\RobotType);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 99:
                !Close Connection
                IF nParams=0 THEN
                    reconnect:=TRUE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 100:
                ! LEFT ARM: Send robot to home    
                IF nParams=0 THEN
                    MoveAbsJ jposHomeYuMiL\NoEOffs,currentSpeed,fine,tool0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            DEFAULT:
                ok:=SERVER_BAD_MSG;
            ENDTEST
            !---------------------------------------------------------------------------------------------------------------
            !Compose the acknowledge string to send back to the client
            IF connected and reconnected=FALSE and SocketGetStatus(clientSocket)=SOCKET_CONNECTED and should_send_res THEN                                
                IF reconnect THEN
                    connected:=FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    reconnect:=FALSE;
                ELSE
                    SocketSend clientSocket\Str:=FormateRes(addString);
                ENDIF
            ENDIF
        ENDWHILE
        
    ERROR 
        ok:=SERVER_BAD_MSG;
        should_send_res:=FALSE;
        
        TEST ERRNO
            CASE ERR_HAND_WRONG_STATE:
                ok := SERVER_OK;
                should_send_res:=TRUE;
                RETRY;
            CASE ERR_SOCK_CLOSED:
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                should_send_res:=TRUE;
                RETRY;
                 
            CASE ERR_HAND_NOTCALIBRATED:
                SocketSend clientSocket\Str:=FormateRes( "ERR_HAND_NOTCALIBRATED: "+NumToStr(ERRNO,0));
                
                ! Gripper not calibrated.
                Hand_Initialize\Calibrate;
                RETRY;

!            CASE ERR_COLL_STOP:
!                SocketSend clientSocket\Str:=FormateRes("ERR_COLL_STOP: "+NumToStr(ERRNO,0));

!                StopMove\Quick;
!                ClearPath;
!                StorePath;
!                !MotionSup\Off;
!                cartesianTarget:=Offs(CRobT(),0,0,50);
!                MoveL cartesianTarget,v300,fine,currentTool\WObj:=currentWobj;
!                !MotionSup\On;
!                RestoPath;
                
!                !StartMoveRetry;
!                !RETRY;
!                !TRYNEXT;
                
            CASE ERR_ROBLIMIT:
                ! Position is reachable but at least one axis is outside joint limit or limits exceeded for at least one coupled joint (function CalcJoinT)
                SocketSend clientSocket\Str:=FormateRes("ERR_ROBLIMIT: "+NumToStr(ERRNO,0));
                RETRY;
                
            CASE ERR_OUTOFBND:
                ! The position (robtarget) is outisde the robot's working area for function CalcJoinT.
                SocketSend clientSocket\Str:=FormateRes("ERR_OUTSIDE_REACH: "+NumToStr(ERRNO,0));
                RETRY;
                
            DEFAULT:
                SocketSend clientSocket\Str:=FormateRes("Default Error: "+NumToStr(ERRNO,0));
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                RETRY;
        ENDTEST
  

 
       
    ENDPROC
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////
    !//SERVER: Main procedure
    !/////////////////////////////////////////////////////////////////////////////////////////////////////////

    PROC main_1()
        VAR progdisp progdisp1;
        
        !//Local variables
        VAR string receivedString;
        !//Received string
        VAR string sendString;
        !//Reply string
        VAR string addString;
        !//String to add to the reply.
        VAR bool connected;
        !//Client connected
        VAR bool reconnected;
        !//Reconnect after sending ack
        VAR bool reconnect;
        !//Drop and reconnection happened during serving a command
        VAR robtarget cartesianPose;
        VAR jointtarget jointsPose;
         
        !//Motion configuration
        ConfL\Off;
        SingArea\Wrist;
        moveCompleted:=TRUE;

        !//Initialization of WorkObject, Tool, Speed and Zone
        Initialize;
        !//Socket connection
        connected:=FALSE;
        ServerCreateAndConnect ipController,serverPort;
        
        connected:=TRUE;
        reconnect:=FALSE;
        
        !//Server Loop
        WHILE TRUE DO
            
            !//For message sending post-movement
            should_send_res:=TRUE;
            
            !//Initialization of program flow variables
            ok:=SERVER_OK;
            !//Has communication dropped after receiving a command?
            addString:="";
           


            !//Wait for a command
            SocketReceive clientSocket \Str := str_data;
            TPWrite str_data ;
            
            
        
            
!         SocketReceive clientSocket\Str:=receivedString\Time:=WAIT_MAX;
            
!            ParseMsg receivedString;

            !//Correctness of executed instruction.
            reconnected:=FALSE;

            !//Execution of the command
            !---------------------------------------------------------------------------------------------------------------
            TEST instructionCode
            CASE 0:
                !Ping
                IF nParams=0 THEN
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 1:
                !Cartesian Move
               
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        ClkReset clock1;
                        ClkStart clock1;
                        
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        ClkStop clock1;
                        reg1:=ClkRead(clock1);
                        addString:=NumToStr(reg1,5);
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 2:
                !Joint Move
                IF nParams=7 THEN
                    externalAxis.eax_a:=params{7};
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;
                    ClkReset clock1;
                    ClkStart clock1;
                    MoveAbsJ jointsTarget,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    ClkStop clock1;
                    reg1:=ClkRead(clock1);
                    addString:=NumToStr(reg1,5);
                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 3:
                !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams=0 THEN
                    cartesianPose:=CRobT(\Tool:=currentTool\WObj:=currentWObj);
                    addString:=NumToStr(cartesianPose.trans.x,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.y,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.trans.z,2)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q1,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q2,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q3,3)+" ";
                    addString:=addString+NumToStr(cartesianPose.rot.q4,3);
                    !End of string	
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 4:
                !Get Joint Coordinates
                IF nParams=0 THEN
                    jointsPose:=CJointT();
                    addString:=NumToStr(jointsPose.robax.rax_1,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_2,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_3,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_4,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_5,2)+" ";
                    addString:=addString+NumToStr(jointsPose.robax.rax_6,2)+" ";
                    addString:=addString+NumToStr(jointsTarget.extax.eax_a,2);
                    !End of string
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 5:
                !Cartesian Move, nonlinear movement
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        ClkReset clock1;
                        ClkStart clock1;
                        MoveJ cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        ClkStop clock1;
                        reg1:=ClkRead(clock1);
                        addString:=NumToStr(reg1,5);
                        moveCompleted:=TRUE;
                   ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 6:
                !Set Tool
                IF nParams=7 THEN
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 7:
                !Set Work Object
                IF nParams=7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 8:
                !Set Speed of the Robot
                IF nParams=4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
                    ok:=SERVER_OK;
                ELSEIF nParams=2 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 9:
                !Set zone data
                IF nParams=4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep:=TRUE;
                        currentZone.pzone_tcp:=0.0;
                        currentZone.pzone_ori:=0.0;
                        currentZone.zone_ori:=0.0;
                    ELSE
                        currentZone.finep:=FALSE;
                        currentZone.pzone_tcp:=params{2};
                        currentZone.pzone_ori:=params{3};
                        currentZone.zone_ori:=params{4};
                    ENDIF
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 11:
                !Cartesian Move (synchronized)
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;

                        SyncMoveOn Sync_Start_Arms,tasklistArms;
                        MoveL cartesianTarget\ID:=11,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        SyncMoveOff Sync_Stop_Arms;

                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 12:
                !Joint Move (synchronized)
                IF nParams=7 THEN
                    externalAxis.eax_a:=params{7};
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;

                    SyncMoveOn Sync_Start_Arms,tasklistArms;
                    MoveAbsJ jointsTarget\ID:=12,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    SyncMoveOff Sync_Stop_Arms;

                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 13:
                !Relative Cartesian Move
                IF nParams=3 THEN
                    cartesianTarget:=Offs(CRobT(),params{1},params{2},params{3});

                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                    
                ELSEIF nParams=6 THEN
                    cartesianTarget:=RelTool(CRobT(),params{1},params{2},params{3},\Rx:=params{4}\Ry:=params{5}\Rz:=params{6});

                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        ok:=SERVER_OK;
                        moveCompleted:=FALSE;
                        MoveL cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                        moveCompleted:=TRUE;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 14:
                !ContactL //NOTE: NOT IMPLEMENTED ON PYTHON
                ! Desired Torque Stated
                IF nParams=8 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    ok:=SERVER_OK;
!                    ContactL\DesiredTorque:=params{8},cartesianTarget,v100,\Zone:=currentZone,currentTool,\WObj:=currentWobj;
                    ! Desired Torque Not Stated
                    ! Instruction will only raise the collision detection level and not supervise the internal torque level
                ELSEIF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       L_CONF,
                                       externalAxis];
                    ok:=SERVER_OK;
!                    ContactL cartesianTarget,v100,\Zone:=currentZone,currentTool,\WObj:=currentWobj;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 20:
                !Gripper Close
                IF nParams=0 THEN
                    Hand_GripInward;
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripInward;!\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripInward;!\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 21:
                !Gripper Open
                IF nParams=0 THEN
                    Hand_GripOutward;
                    ok:=SERVER_OK;

                ELSEIF nParams=1 THEN
                    Hand_GripOutward;!\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! holdForce range = 0 - 20 N, targetPos = 0 - 25 mm, posAllowance = tolerance of gripper closure value
                ELSEIF nParams=2 THEN
                    Hand_GripOutward;!\holdForce:=params{1}\targetPos:=params{2};
                    ok:=SERVER_OK;

                    ! Program won't wait until gripper completion or failure to move on.
                ELSEIF nParams=3 THEN
                    Hand_GripOutward;!\holdForce:=params{1}\targetPos:=params{2}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 22:
                ! Initialize gripper with specified values

                ! calibrate only
                IF nParams=0 THEN
                    Hand_Initialize\Calibrate;
                    ok:=SERVER_OK;

                    ! set maxSpeed, holdForce, physicalLimit (0-25 mm), and calibrate                    
                ELSEIF nParams=3 THEN
                    Hand_Initialize;!\maxSpd:=params{1}\holdForce:=params{2}\phyLimit:=params{3}\Calibrate;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 23:
                ! Set Max Speed
                IF nParams=1 THEN
                    Hand_SetMaxSpeed params{1};
                    ! between 0-20 mm/s 
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 24:
                ! Set gripping force 
                IF nParams=1 THEN
                    Hand_SetHoldForce params{1};
                    ! between 0-20 Newtons
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 25:
                ! Move the gripper to a specified position 
                IF nParams=1 THEN
                    Hand_MoveTo params{1};
                    ! between 0-25 mm or 0-phyLimit if phyLimit is set in CASE 22
                    ok:=SERVER_OK;

                ELSEIF nParams=2 THEN
                    Hand_MoveTo params{1}\NoWait;
                    ok:=SERVER_OK;

                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

                !---------------------------------------------------------------------------------------------------------------
            CASE 26:
                !Get Gripper Width
                IF nParams=0 THEN
                    addString:=NumToStr(Hand_GetActualPos(),2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 29:
                ! Stop any action of the gripper (motors will lose power)
                IF nParams=0 THEN
                    Hand_Stop;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 30:
                !Add Cartesian Coordinates to buffer
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        L_CONF,
                                        externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        IF BUFFER_POS<MAX_BUFFER THEN
                            BUFFER_POS:=BUFFER_POS+1;
                            bufferTargets{BUFFER_POS}:=cartesianTarget;
                            bufferSpeeds{BUFFER_POS}:=currentSpeed;
                        ENDIF
                        ok:=SERVER_OK;
                    ELSE
                        ok := SERVER_BAD_MSG;
                        addString := "Unreachable Pose";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 31:
                !Clear Cartesian Buffer
                IF nParams=0 THEN
                    BUFFER_POS:=0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 32:
                !Get Buffer Size)
                IF nParams=0 THEN
                    addString:=NumToStr(BUFFER_POS,2);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 33:
                !Execute moves in cartesianBuffer as linear moves
                IF nParams=0 THEN
                    FOR i FROM 1 TO (BUFFER_POS) DO
                        MoveL bufferTargets{i},bufferSpeeds{i},currentZone,currentTool\WObj:=currentWobj;
                    ENDFOR
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 34:
                !External Axis move
                IF nParams=6 THEN
                    externalAxis:=[params{1},params{2},params{3},params{4},params{5},params{6}];
                    jointsTarget:=CJointT();
                    jointsTarget.extax:=externalAxis;
                    ok:=SERVER_OK;
                    moveCompleted:=FALSE;
                    MoveAbsJ jointsTarget,currentSpeed,currentZone,currentTool\Wobj:=currentWobj;
                    moveCompleted:=TRUE;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
                TPWrite "Execution started";
            CASE 35:
                !Specify circPoint for circular move, and then wait on toPoint
                IF nParams=7 THEN
                    circPoint:=[[params{1},params{2},params{3}],
                                [params{4},params{5},params{6},params{7}],
                                [0,0,0,0],
                                externalAxis];
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 36:
                !specify toPoint, and use circPoint specified previously
                IF nParams=7 THEN
                    cartesianTarget:=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    MoveC circPoint,cartesianTarget,currentSpeed,currentZone,currentTool\WObj:=currentWobj;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 40:
                !returns 1 if given pose is reachable. 0 other wise.
                IF nParams=7 THEN
                    cartesianTarget := [[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [-1,-1,0,11],
                                       externalAxis];
                    IF isPoseReachable(cartesianTarget, currentTool, currentWobj) THEN
                        addString := "1";
                    ELSE
                        TPWrite "not reachable";
                        addString := "0";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 41:
                !returns 1 if given joint configuration is reachable. 0 other wise.
                IF nParams=7 THEN
                    jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}],externalAxis];
                    IF isJointsReachable(jointsTarget, currentTool, currentWobj) THEN
                        addString := "1";
                    ELSE
                        addString := "0";
                    ENDIF
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 98:
                !returns current robot info: serial number, robotware version, and robot type
                IF nParams=0 THEN
                    addString:=GetSysInfo(\SerialNo)+"*";
                    addString:=addString+GetSysInfo(\SWVersion)+"*";
                    addString:=addString+GetSysInfo(\RobotType);
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            CASE 99:
                !Close Connection
                IF nParams=0 THEN
                    reconnect:=TRUE;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
            CASE 100:
                ! LEFT ARM: Send robot to home    
                IF nParams=0 THEN
                    MoveAbsJ jposHomeYuMiL\NoEOffs,currentSpeed,fine,tool0;
                    ok:=SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF
                !---------------------------------------------------------------------------------------------------------------
            DEFAULT:
                ok:=SERVER_BAD_MSG;
            ENDTEST
            !---------------------------------------------------------------------------------------------------------------
            !Compose the acknowledge string to send back to the client
            IF connected and reconnected=FALSE and SocketGetStatus(clientSocket)=SOCKET_CONNECTED and should_send_res THEN                                
                IF reconnect THEN
                    connected:=FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;
                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected:=TRUE;
                    reconnected:=TRUE;
                    reconnect:=FALSE;
                ELSE
                    SocketSend clientSocket\Str:=FormateRes(addString);
                ENDIF
            ENDIF
        ENDWHILE
        
    ERROR 
        ok:=SERVER_BAD_MSG;
        should_send_res:=FALSE;
        
        TEST ERRNO
            CASE ERR_HAND_WRONG_STATE:
                ok := SERVER_OK;
                should_send_res:=TRUE;
                RETRY;
            CASE ERR_SOCK_CLOSED:
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                should_send_res:=TRUE;
                RETRY;
                 
            CASE ERR_HAND_NOTCALIBRATED:
                SocketSend clientSocket\Str:=FormateRes( "ERR_HAND_NOTCALIBRATED: "+NumToStr(ERRNO,0));
                
                ! Gripper not calibrated.
                Hand_Initialize\Calibrate;
                RETRY;

!            CASE ERR_COLL_STOP:
!                SocketSend clientSocket\Str:=FormateRes("ERR_COLL_STOP: "+NumToStr(ERRNO,0));

!                StopMove\Quick;
!                ClearPath;
!                StorePath;
!                !MotionSup\Off;
!                cartesianTarget:=Offs(CRobT(),0,0,50);
!                MoveL cartesianTarget,v300,fine,currentTool\WObj:=currentWobj;
!                !MotionSup\On;
!                RestoPath;
                
!                !StartMoveRetry;
!                !RETRY;
!                !TRYNEXT;
                
            CASE ERR_ROBLIMIT:
                ! Position is reachable but at least one axis is outside joint limit or limits exceeded for at least one coupled joint (function CalcJoinT)
                SocketSend clientSocket\Str:=FormateRes("ERR_ROBLIMIT: "+NumToStr(ERRNO,0));
                RETRY;
                
            CASE ERR_OUTOFBND:
                ! The position (robtarget) is outisde the robot's working area for function CalcJoinT.
                SocketSend clientSocket\Str:=FormateRes("ERR_OUTSIDE_REACH: "+NumToStr(ERRNO,0));
                RETRY;
                
            DEFAULT:
                SocketSend clientSocket\Str:=FormateRes("Default Error: "+NumToStr(ERRNO,0));
                connected:=FALSE;
                !//Closing the server
                SocketClose clientSocket;
                SocketClose serverSocket;
                !//Reinitiate the server
                ServerCreateAndConnect ipController,serverPort;
                reconnected:=TRUE;
                connected:=TRUE;
                RETRY;
        ENDTEST
    ENDPROC
    


ENDMODULE

MODULE MainModule1
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
    
    LOCAL VAR bool BODY_COMBINE_SUCCESS;
    
    ! =================== Camera var =========================
    LOCAL CONST num TI_Cap := 1;
    LOCAL CONST num TI_Body_U := 2;
    LOCAL CONST num TI_Body_D := 3;
    LOCAL VAR CJ_Job job_;
    LOCAL VAR CJ_Tool ptools_{3};
    LOCAL VAR cj_ObjPos obj_pos_{3};
    LOCAL VAR bool obj_pos_updated_{3};
    
    ! =================== Init position ======================
    LOCAL CONST robtarget init_pos_:=[[-7.92,-281.29,257.07],[0.000253616,-0.708004,-0.706209,-0.000224018],[-1,-2,-1,11],[-93.6182,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    LOCAL CONST num pos_scale_ := 0.946969;
    LOCAL CONST robtarget init_cap_robtarget_:=[[199.69,-516.79,157.26],[0.00125898,0.706923,0.70729,-0.000635735],[0,-1,0,11],[-173.244,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_cap_objpos_ := [92.2,-91.1,-5.911484];
    !LOCAL CONST robtarget init_body_u_robtarget_:=[[176.23,-436.35,195.27],[0.00308931,-0.709011,0.00313078,0.705185],[0,-1,1,11],[134.856,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST robtarget init_body_u_robtarget_:=[[179.18,-440.35,193.67],[0.00309015,-0.709007,0.00309541,0.705189],[0,-1,1,11],[134.868,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_body_u_objpos_ := [86.4,-11.4,-358.309967];
    LOCAL CONST robtarget init_body_d_robtarget_:=[[122.57,-434.21,186.49],[0.00419306,-0.704552,0.00478445,-0.709625],[0,0,-1,11],[-131.571,9E+09,9E+09,9E+09,9E+09,9E+09]];
    LOCAL CONST CJ_ObjPos init_body_d_objpos_ := [-14.460936,3.674437,-357.674103];

    
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
    VAR num serverPort:=5001;

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
    CONST robtarget p11:=[[372.64,339.11,289.11],[0.477004,-0.527842,0.612727,-0.344118],[0,-2,0,11],[-127.116,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p21:=[[79.78,477.73,302.50],[0.427727,-0.692755,0.521034,0.256254],[0,-2,1,11],[102.529,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p31:=[[53.54,554.74,302.54],[0.426881,-0.6917,0.519561,0.263404],[0,-2,1,11],[102.532,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget p41:=[[410.62,10.55,225.62],[0.0336807,-0.0388925,-0.698789,-0.713476],[-1,-2,0,1001],[97.7122,9E+09,9E+09,9E+09,9E+09,9E+09]];
!    CONST robtarget p11:=[[-140.63,360.74,583.76],[0.215646,-0.695059,0.360686,0.583349],[0,-2,0,11],[108.39,9E+09,9E+09,9E+09,9E+09,9E+09]];


    LOCAL PROC aa_TechPos ()
        MoveL init_cap_robtarget_,v1000, fine, tool1;
        MoveL init_body_u_robtarget_,v1000, fine, tool1;
        MoveL init_body_d_robtarget_,v1000, fine, tool1;
    ENDPROC
    
    LOCAL PROC InitCamareVar()
        ! job
        cj_InitCJJobRecord job_;
        job_.cam_id := 1;
        job_.job_name := "PenRight.job";
        job_.ip_addr := "192.168.10.208";
        job_.port := 23;
        job_.user_name := "admin";
        job_.user_passwd := "";
        job_.max_trigger_time := 10000;   
        
        ! Tool cap
        cj_InitCJToolRecord ptools_{TI_Cap};
        ptools_{TI_Cap}.tool_name := "PenCap";
        ptools_{TI_Cap}.enabled_tags := "PenCap";
        ptools_{TI_Cap}.need_trigger := TRUE;
        
        ! Tool body up
        cj_InitCJToolRecord ptools_{TI_Body_U};
        ptools_{TI_Body_U}.tool_name := "PenBodyUp";
        ptools_{TI_Body_U}.enabled_tags := "PenBodyUp";
        ptools_{TI_Body_U}.need_trigger := TRUE;
        
        ! Tool body down
        cj_InitCJToolRecord ptools_{TI_Body_D};
        ptools_{TI_Body_D}.tool_name := "PenBodyDown";
        ptools_{TI_Body_D}.enabled_tags := "PenBodyDown";
        ptools_{TI_Body_D}.need_trigger := TRUE;
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
        
        ! try get obj pos
        WaitRob \InPos;
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
    
    PROC main()
        VAR bool result;
        
!        WHILE TRUE DO
!        ENDWHILE
        ConfL \Off;
        SingArea \Wrist;
        InitCamareVar;
        !a0_reset_from_error;
        result := cj_Init(job_, ptools_);
        
        WHILE TRUE DO
            combine_pen;
        ENDWHILE
        
	ENDPROC
    
    LOCAL PROC move_init()
        MoveJ init_pos_, v1000, fine, tool1;
    ENDPROC
    
    LOCAL PROC reset_bool_flag()
        R_BODY_READY := FALSE;
        R_BODY_SPINED := FALSE;
        R_HAND_LEAVE := FALSE;
        R_CAP_FINISH := FALSE;
    ENDPROC
    
    LOCAL PROC combine_pen()
        VAR bool result;
        VAR num status;

        ! Init
        reset_bool_flag;
        ptools_{TI_Body_U}.need_trigger := FALSE;
        ptools_{TI_Body_D}.need_trigger := FALSE;
        ptools_{TI_Cap}.need_trigger := FALSE;

        ! combine body
        WaitOneObjPos([TI_Body_U, TI_Body_D]);
        IF obj_pos_updated_{TI_Body_U} THEN
            pick_body_up;
        ELSEIF obj_pos_updated_{TI_Body_D} THEN
            pick_body_down;
        ENDIF

        combine_body;
        
        IF BODY_COMBINE_SUCCESS THEN
            ! combine cap
            WaitObjPos(TI_Cap);
            combine_cap;
        ENDIF
        
        move_init;
        SyncMoveOn END_MOVE_ON, task_list;
            MoveL Offs(init_pos_, 0, 0, -30) \ID:=END_MOVE_ID1, v1000, fine, tool1;
            reset_bool_flag;
            MoveL init_pos_ \ID:=END_MOVE_ID2, v1000, fine, tool1;
        SyncMoveOff END_MOVE_OFF;
        
    ENDPROC
    
    LOCAL PROC pick_body_up()
        VAR robtarget pick_pos;
        VAR num x_off;
        VAR num y_off;
        
         ! pick object
        x_off := (obj_pos_{TI_Body_U}.x - init_body_u_objpos_.x) * pos_scale_;
        y_off := (obj_pos_{TI_Body_U}.y - init_body_u_objpos_.y) * pos_scale_;
        pick_pos := Offs(init_body_u_robtarget_, x_off, y_off, 0);
        
        open_hand;
        MoveL Offs(pick_pos, 50, 0, 80), vmax, z10, tool1;
        MoveL Offs(pick_pos, 50, 0, 0), v1000, z10, tool1;
        MoveL pick_pos, v500, fine, tool1;
        
        close_hand \force:=150;
        MoveL Offs(pick_pos, 60, 0, 0), v50, z0, tool1;
        MoveL Offs(pick_pos, 60, 0, 50), v1000, z10, tool1;
        
    ENDPROC
    
    LOCAL PROC pick_body_down()
        VAR robtarget pick_pos;
        VAR num x_off;
        VAR num y_off;
        
         ! pick object
        x_off := (obj_pos_{TI_Body_D}.x - init_body_d_objpos_.x) * pos_scale_;
        y_off := (obj_pos_{TI_Body_D}.y - init_body_d_objpos_.y) * pos_scale_;
        pick_pos := Offs(init_body_d_robtarget_, x_off, y_off, 0);
        
        open_hand;
        MoveL Offs(pick_pos, -50, 0, 80), vmax, z10, tool1;
        MoveL Offs(pick_pos, -50, 0, 0), v1000, z10, tool1;
        MoveL pick_pos, v500, fine, tool1;
        
        close_hand \force:=150;
        MoveL Offs(pick_pos, -50, 0, 0), v1000, z10, tool1;
        MoveL Offs(pick_pos, -50, 0, 50), v1000, z10, tool1;
        
    ENDPROC
    
    LOCAL PROC aaa_test()
        CONST robtarget wait_holder_pos:=[[396.30,-32.73,229.50],[0.499893,-0.499739,0.50028,0.50009],[0,0,-3,11],[164.821,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        VAR robtarget temp_pos := wait_holder_pos;
        VAR num a;
        a := EulerZYX(\X, temp_pos.rot);
        a := EulerZYX(\Y, temp_pos.rot);
        a := EulerZYX(\Z, temp_pos.rot);
        temp_pos.rot := OrientZYX(-180, 90, 90);
        
    ENDPROC
    LOCAL PROC combine_body()
        VAR robtarget pick_pos;
        CONST robtarget to_wait_pos:=[[279.05,-233.44,360.00],[0.774907,-0.578073,-0.0927669,0.238216],[0,0,0,11],[107.511,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_pilot_pos:=[[395.20,-69.44,357.86],[0.708287,0.00120865,-0.000346227,-0.705925],[1,0,-1,11],[161.333,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_pilot_pos_spin:=[[395.20,-69.44,357.85],[0.703369,-0.000351052,-0.00122631,0.710824],[1,0,-3,11],[161.334,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget shake_pilot_pos1:=[[395.20,-69.44,357.85],[0.701055,-0.0587824,0.0574237,0.708357],[0,0,-3,11],[161.334,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget shake_pilot_pos2:=[[395.20,-69.44,357.85],[0.702243,0.0386072,-0.0397791,0.709777],[1,0,-3,11],[162.409,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_holder_pos:=[[396.30,-32.73,229.50],[0.499893,-0.499739,0.50028,0.50009],[0,0,-3,11],[164.821,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_beg_pos:=[[396.29,-32.75,229.51],[0.501881,-0.501755,0.498253,0.498099],[0,0,-3,11],[164.821,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_end_pos:=[[396.30,-32.74,229.50],[0.481295,-0.481152,0.518185,0.518009],[0,0,1,11],[164.821,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_failed_pos_move:=[[539.23,-32.28,135.61],[0.49921,-0.499089,0.500902,0.500797],[1,0,-3,11],[164.825,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget spin_failed_pos_spin:=[[539.23,-32.28,135.61],[0.00645186,-0.00662918,-0.707069,-0.707085],[1,0,-2,11],[164.824,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget to_back_pos1:=[[396.30,-103.17,229.50],[0.481291,-0.481158,0.51818,0.518012],[0,0,1,11],[164.821,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget to_back_pos2:=[[261.02,-121.63,262.78],[0.341697,-0.591058,0.62354,0.380911],[-1,0,-3,11],[133.204,9E+09,9E+09,9E+09,9E+09,9E+09]];
        
        CONST num shake_num := 2;
        CONST num step_per_spin := 0.6;
        CONST num step_num := 3;
        
        
        ! move to combine pos
        ConfL \On;
        MoveL to_wait_pos, v1500, z50, tool1;
        MoveL wait_pilot_pos, v1500, fine, tool1;
        R_BODY_READY := TRUE;
        
        ! Wait pilot finsh 
        WaitUntil(L_PILOT_FINISH = TRUE);

        R_BODY_READY := FALSE;
        WaitTime(0.3);
        MoveL wait_pilot_pos_spin, vmax, fine, tool1;
        ! shake
        FOR i FROM 1 TO shake_num DO
            MoveL shake_pilot_pos1, vmax, z0, tool1;
            MoveL shake_pilot_pos2, vmax, z0, tool1;
        ENDFOR
        MoveL wait_pilot_pos_spin, vmax, fine, tool1;
        WaitTime 1;

        ! wait holder
        MoveL wait_holder_pos, v100, fine, tool1;
        R_BODY_READY := TRUE;
        WaitUntil(L_HOLDER_READY = TRUE);
        
        ! spin
        ! first spin
        SingArea \Wrist;
        
        !codes for modify position
        !MoveL spin_end_pos, vmax, fine, tool1;
        !MoveL spin_beg_pos, vmax, fine, tool1;
        !MoveL spin_failed_pos_move, vmax, fine, tool1; 
        !MoveL spin_failed_pos_spin, vmax, fine, tool1; 
        
        ! hold the pen body
        SyncMoveOn SPIN_MOVE_ON, task_list;
            FOR i FROM 1 TO (step_num - 1) DO
                MoveL Offs(spin_end_pos, 0, step_per_spin, 0) \ID:=SPIN_ID_DO,vmax, fine, tool1;
                !open_hand \max_wait_time:= 0.5 \target_pos:=30;
                open_hand \max_wait_time:= 0.5;
                R_BODY_GRIPPED := FALSE;
                ! MoveL spin_end_pos,vmax, fine, tool1;
                MoveL spin_beg_pos \ID:=SPIN_ID_UNDO,vmax, fine, tool1;
                close_hand \force:=200;
                
                IF Hand_GetActualPos() < 30 THEN
                    R_BODY_GRIP_SUCCESS := TRUE;
                    R_BODY_GRIPPED := TRUE;
                ELSE
                    R_BODY_GRIP_SUCCESS := FALSE;
                    R_BODY_GRIPPED := TRUE;
                    GOTO SPIN_FAILED;
                ENDIF
                
                
            ENDFOR
            GOTO SPIN_LAST;
        
        SPIN_FAILED:
            open_hand;
            MoveL spin_failed_pos_move \ID:=SPIN_ID_FAILED1, v200, fine, tool1;
            MoveL spin_failed_pos_spin \ID:=SPIN_ID_FAILED2, v200, fine, tool1;
            MoveL spin_end_pos \ID:=SPIN_ID_UNDO, v200, fine, tool1;
            BODY_COMBINE_SUCCESS := FALSE;
            GOTO SPIN_END;
            
        SPIN_LAST:
            MoveL Offs(spin_end_pos, 0, step_per_spin, 0) \ID:=SPIN_ID_DO,vmax, fine, tool1;
            BODY_COMBINE_SUCCESS := TRUE;
            GOTO SPIN_END;
            
        SPIN_END:
            ! nothing
        SyncMoveOff SPIN_MOVE_OFF;
        
        ConfL \Off;
        
        IF NOT BODY_COMBINE_SUCCESS THEN
            a0_reset_from_error;
            RETURN;
        ENDIF

        R_BODY_SPINED := TRUE;
        
        ! wait left hand hold pen
        WaitUntil(L_BODY_PICKED = TRUE);
        open_hand;
        
        MoveL to_back_pos1, v1500, z50, tool1;
        MoveL to_back_pos2, v1500, z50, tool1;
        R_HAND_LEAVE := TRUE;
        
        move_init;
    UNDO
        ConfL \Off;
    ENDPROC

    LOCAL PROC combine_cap()
         VAR robtarget pick_pos;
        CONST robtarget to_wait_pos:=[[317.02,-273.72,265.46],[0.00122608,0.706886,0.707327,-0.000594339],[0,-2,0,11],[-173.192,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget wait_pos:=[[464.59,-36.85,227.66],[0.0185697,-0.704876,-0.708927,0.0151395],[-1,-1,1,11],[116.54,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget combine_pos:=[[464.57,-36.85,166.41],[0.0186012,-0.704899,-0.708903,0.0151471],[-2,-1,1,11],[116.547,9E+09,9E+09,9E+09,9E+09,9E+09]];
        CONST robtarget to_back_pos:=[[366.84,-169.96,227.66],[0.018577,-0.704867,-0.708935,0.015186],[-1,-1,1,11],[116.541,9E+09,9E+09,9E+09,9E+09,9E+09]];

        VAR num x_off;
        VAR num y_off;

        ! pick object
        x_off := (obj_pos_{TI_Cap}.x - init_cap_objpos_.x) * pos_scale_;
        y_off := (obj_pos_{TI_Cap}.y - init_cap_objpos_.y) * pos_scale_;
        pick_pos := Offs(init_cap_robtarget_, x_off, y_off, 0);
        
        MoveL Offs(pick_pos, 0, 0, 60), v1500, fine, tool1;
        open_hand;
        MoveL pick_pos, v100, fine, tool1;
        close_hand;
        MoveL Offs(pick_pos,0,0,100), v100, z50, tool1;
        
        ! combine
        MoveL to_wait_pos, v1500, z50, tool1;
        MoveL wait_pos, v1500, fine, tool1;
        WaitUntil(L_BODY_READY);
        MoveL Offs(combine_pos, 0, 0, 10), v100, z10, tool1;
        MoveL combine_pos, v10, fine, tool1;
        !hand_regrip 5;
        Hand_Stop;
        MoveL wait_pos, v1500, z0, tool1; 
        MoveL to_back_pos, v1500, z0, tool1;
        
        WaitRob \InPos;
        R_CAP_FINISH := TRUE;
        open_hand \max_wait_time:=0.3;
        move_init;
        
    ENDPROC
    
    
    
  !start  

  
   
    
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
    
    PROC main_myo_r()
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
        MoveL p41, v50, z50, tool1;
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
       EndBit3:=StrFind(str_data,StartBit3,"\0D");
       LenBit3:=EndBit3-StartBit3;
       
!       StartBit4:=EndBit3+1;
!       EndBit4:=StrFind(str_data,StartBit4,"\0A");
!       LenBit4:=EndBit4-StartBit4;
       
       
       s1:=StrPart(str_data,StartBit1,LenBit1);
       s2:=StrPart(str_data,StartBit2,LenBit2);
       s3:=StrPart(str_data,StartBit3,LenBit3);
!       s4:=StrPart(str_data,StartBit4,LenBit4);
       
         flag1:=StrToVal(s1,delta_x);
         flag1:=StrToVal(s2,delta_y);
         flag1:=StrToVal(s3,delta_z);
          
         
        TPWrite "delta_x:"\Num:=delta_x;
        TPWrite "delta_y:"\Num:=delta_y;
        TPWrite "delta_z:"\Num:=delta_z;
        
        !p1test:=[[delta_x,delta_y,delta_z],[0.427727,-0.692755,0.521034,0.256254],[0,-2,1,11],[102.529,9E+09,9E+09,9E+09,9E+09,9E+09]];
     !FOR i FROM 1 TO 5 DO
       ! MoveL p11, v50, z50, tool1;
        !WaitTime 2;
        
        MoveL Offs(p11,delta_x,delta_y,delta_z), v50, z50, tool1;
!         IF s4="1"THEN
!               Hand_GripOutward;
                
           
!            ELSE IF s4="0"
!                 Hand_GripInward;
!         ENDIF
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
import rclpy
import DR_init

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
#VELOCITY, ACC = 20, 20

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("stack_block", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import(
            set_digital_output,
            get_digital_input,
            set_tool,
            set_tcp,
            movej,
            movel,
            wait,
            get_current_posx,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_BASE,
            set_tool_shape,
            DR_TOOL,
            set_ref_coord,
            set_desired_force,
            task_compliance_ctrl,
            check_force_condition,
            release_compliance_ctrl,
            release_force,
            DR_MV_MOD_REL,
            move_periodic,
            parallel_axis,
            mwait,
            get_current_posj,
            amovel,
            stop,
            DR_SSTOP,
        )
        from DR_common2 import posx, posj

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return

    def gripper_close():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        print("그립퍼 닫힘")
        wait(0.5)
    def gripper_open():
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        print("그립퍼 열림")
        wait(0.5)


    def z_axis_alignment(axes_data, joint_data):
        vect = [0, 0, -1]
        parallel_axis(vect, DR_AXIS_Z, DR_BASE)
        mwait()
        print("move to z-axis alignment")
        print(get_current_posj(), joint_data)
        print(get_current_posx()[0], axes_data)
        # move_by_line(axes_data, joint_data)

    def check_gear_position_for_pick():
        gripper_close()
        wait(0.1)
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        mwait()
        amovel(move_down_for_gear_check, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        while True:
            print("기어 포지션 체크를 위해 내려가는 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            if fcon1 == 0: #외력이 감지가 되었을 때
                stop(DR_SSTOP) #로봇 멈춤
                mwait()
                print("기어 포지션 체크 끝")
                print(f"현재 포지션: {get_current_posx()[0]}")
                movel(move_up_for_gear, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
            wait(1)

    def gear_pick():
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        mwait()
        set_ref_coord(DR_BASE)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        fd = [0,0,-20,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        while True:
            print("내려가기 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            if fcon1 == 0:
                release_compliance_ctrl()
                mwait(0.2)
                print("내려가기 끝")
                movel(move_up_for_gear, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                gripper_open()
                movel(move_down_for_gear, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                gripper_close()
                break
            wait(1)


    def check_gear_position_for_release():
        wait(0.1)
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        amovel(move_down_for_gear_check, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
        while True:
            print("기어 포지션 체크를 위해 내려가는 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            if fcon1 == 0: #외력이 감지가 되었을 때
                stop(DR_SSTOP) #로봇 멈춤
                mwait()
                print("기어 포지션 체크 끝")
                movel(move_up_for_gear_release, v=30, a=60, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                current_goal_position = get_current_posx()[0]
                print(f"현재 포지션: {current_goal_position}")
                return current_goal_position
            wait(1)


    def gear_release(current_goal_position):
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        mwait()
        set_ref_coord(DR_BASE)
        task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        fd = [0,0,-20,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        while True:
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            if fcon1 == 0:
                release_compliance_ctrl()
                mwait()
                print("기어 충돌 힘 제어 풀기 완료")
                move_periodic(amp =[0,0,1,0,0,5], period=[0,0,1.0,0,0,1.0], atime=0.2, repeat=3, ref=DR_TOOL)
                mwait(0.5)
                print("기어 회전 완료")
                pos_check, _ = get_current_posx(ref=DR_BASE)
                if pos_check[2] <= current_goal_position[2]-20: #만약 충분히 들어갔다
                    release_compliance_ctrl()
                    mwait()
                    gripper_open()
                    break
                else: #만약 충분히 들어가지 않았다. 힘제어로 다시 내려가
                    print()
                    set_ref_coord(DR_BASE)
                    task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
                    set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
                    mwait()
            wait(1)


    set_tool("Tool Weight_2FG")
    set_tcp("2FG_TCP")

    while rclpy.ok():
        point_home = posj(0,0,90,0,90,0)
        point_gear_1 = posx(302.35,204.13,150,0,180,0) #기어 포인트 1위에 도착
        point_gear_2 = posx(215.00,145.02,150,0,180,0)
        point_gear_3 = posx(310.44,98.31,150,0,180,0)
        point_sungear = posx(276.81,148.73,150,0,180,0)
        goal_gear_1 = posx(301.72, -93.63, 150, 0, -180, 0)
        goal_gear_2 = posx(216.70,-153.78,150,0,180,0)
        goal_gear_3 = posx(308.72,-198.04,150,0,180,0)
        goal_sungear = posx(274.45,-148.91,150,0,180,0)

        move_down_for_gear = posx(0,0,20,0,0,0)
        move_up_for_gear = posx(0,0,-10,0,0,0)
        move_down_for_gear_check = posx(0,0,150,0,0,0)
        move_up_for_gear_release = posx(0,0,-25,0,0,0)

        gripper_open()
        movej(point_home, v=30, a=60)
        print("home_point이동 완료")
        movel(point_gear_1,v=30,a=60,ref=DR_BASE)
        print("point1 도착 완료")
        check_gear_position_for_pick()
        gear_pick()
        print("기어 집기 완료")
        movel(point_gear_1,v=30,a=60,ref=DR_BASE)
        #########기어 집고 위로 이동
        movel(goal_gear_1,v=30,a=60,ref=DR_BASE)
        print("goal1 도착 완료")
        #########기어 놓을 위치 확인
        currnet_goal_pos = check_gear_position_for_release()
        gear_release(currnet_goal_pos)
        print("기어 놓기 완료")
        movel(goal_gear_1,v=30,a=60,ref=DR_BASE)
        movej(point_home, v=30, a=60)
        break

if __name__ == "__main__":
    main()

import rclpy
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0ACC9"
VELOCITY, ACC = 40, 80

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("zenga_pick_and_place", namespace=ROBOT_ID)
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
            DR_TOOL,
            set_ref_coord,
            set_desired_force,
            task_compliance_ctrl,
            check_force_condition,
            release_compliance_ctrl,
            release_force,
            DR_MV_MOD_REL,
            parallel_axis,
            mwait,
            get_current_posj,
            DR_MV_MOD_ABS,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing _ROBOT2: {e}")
        return

    def gripper_close():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        print("그리퍼 닫힘")
        wait(0.5)

    def gripper_open():
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        print("그리퍼 열림")
        wait(0.5)

    def z_axis_alignment():
        vect = [0, 0, -1]
        parallel_axis(vect, DR_AXIS_Z, DR_BASE)
        mwait(0.5)
        print("move to z-axis alignment")

    def check_cup_position_and_pick(pic_point):
        move_up_for_cup = posx(0,0,-10,0,0,0)
        move_down_for_cup = posx(0,0,20,0,0,0)
        move_up_for_cup_pick = posx(0,0,-120,0,0,0)
        gripper_close()
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        set_ref_coord(DR_BASE)#기준 좌표계를 Base로 변경
        task_compliance_ctrl([2000,2000,500,100,100,100])
        fd = [0,0,-30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        print("컵을 집기 위해 내려가기 시작")
        while True:
            print("컵 집기 포지션 체크를 위해 내려가는 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            pos_first, _ = get_current_posx(ref=DR_BASE)
            print(f"현재 pos값: {pos_first}")
            if fcon1 == 0: #외력이 감지가 되었을 때
                release_compliance_ctrl()
                mwait(1)
                print("힘 제어 풀기 완료")
                print(f"pic_point: {pic_point}")
                print("정렬 시작")
                pos_second, _ = get_current_posx(ref=DR_BASE)
                if pos_second[:1] != pic_point[:1] and pos_second[2:] != pic_point[2:]: #pos_second[2]즉 Z축을 제외한 나머지를 pic_point와 일치시킨다.
                    new_position = posx(pic_point[0],pic_point[1],pos_second[2],pic_point[3],pic_point[4],pic_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                    movel(move_up_for_cup, vel=VELOCITY, acc=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL) #위로 10만큼 offset
                    mwait(1)
                    movel(new_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                    print("정렬 완료")
                    mwait(1)
                print(f"현재 포지션: {get_current_posx()[0]}")
                movel(move_up_for_cup, v=VELOCITY, a=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                mwait(1)
                gripper_open()
                movel(move_down_for_cup, v=VELOCITY, a=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                mwait(1)
                gripper_close()
                print("컵 집기 끝")
                movel(move_up_for_cup_pick, v=VELOCITY, a=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                print("컵을 들어")
                break
            wait(1)

    def check_cup_position_and_release(goal_point):
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        set_ref_coord(DR_BASE)#기준 좌표계를 Base로 변경
        task_compliance_ctrl([2000,2000,500,100,100,100])
        fd = [0,0,-30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        print("컵을 놓기 위해 내려가기 시작")
        while True:
            print("컵 놓기 포지션 체크를 위해 내려가는 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 5, ref=DR_BASE)
            if fcon1 == 0: #외력이 감지가 되었을 때
                print("컵 포지션 체크 끝")
                release_compliance_ctrl()
                mwait(1)
                print("힘 제어 풀기 완료")
                print(f"현재 포지션: {get_current_posx()[0]}")
                print(f"pic_point: {goal_point}")
                #while True:
                print("정렬 시작")
                pos_second, _ = get_current_posx(ref=DR_BASE)
                if pos_second[:1] != goal_point[:1] and pos_second[2:] != goal_point[2:]: #pos_second[2]즉 Z축을 제외한 나머지를 pic_point와 일치시킨다.
                    new_position = posx(goal_point[0],goal_point[1],pos_second[2],goal_point[3],goal_point[4],goal_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                    movel(move_up_for_cup, vel=VELOCITY, acc=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL) #위로 10만큼 offset
                    mwait(1)
                    movel(new_position, vel=VELOCITY, acc=ACC, ref=DR_BASE)
                    print("정렬 완료")
                    mwait(1)
                gripper_open()
                print("컵 놓기 끝")
                mwait(1)
                current_xy_position = get_current_posx()[0]
                current_xy_position = [current_xy_position[0], current_xy_position[1], 200, current_xy_position[3],current_xy_position[4],current_xy_position[5]]
                print(current_xy_position)
                movel(current_xy_position,v=VELOCITY, a=ACC,ref=DR_BASE,mod=DR_MV_MOD_ABS)
                print("놓고 올라오기 완료")
                break
            wait(1)
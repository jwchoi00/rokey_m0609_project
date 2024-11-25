import rclpy
import math
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0ACC9"
VELOCITY, ACC = 100, 150
CUP_VEL, CUP_ACC = 40, 40
TOOL_NAME='Tool'
TOOL_WEIGHT='Gripper'

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("project", namespace=ROBOT_ID)
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
            set_robot_mode,
            get_tool,
            get_tcp,
            fkin,
            DR_FC_MOD_ABS,
        )
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing _ROBOT2: {e}")
        return

#######################################################################################

    def tool_setting(tool_name,tool_weight):
        set_robot_mode(0)
        set_tool(tool_name)
        set_tcp(tool_weight)
        set_robot_mode(1)
        print('tool name: {0}, tool weight: {1}'.format(get_tool(),get_tcp()))

    def gripper_close():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        print("그리퍼 닫힘")
        wait(3)

    def gripper_open():
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        print("그리퍼 열림")
        wait(3)

    def z_axis_alignment(axes_data, joint_data):
        vect = [0, 0, -1]
        parallel_axis(vect, DR_AXIS_Z, DR_BASE)
        mwait(0.5)
        print("move to z-axis alignment")
        print(get_current_posj(), joint_data)
        print(get_current_posx()[0], axes_data)
        # move_by_line(axes_data, joint_data)

    def alignment_for_pick_cup(current_point, pick_point): #
        move_up_for_cup = posx(0,0,-10,0,0,0)
        if current_point[:1] != pick_point[:1] and current_point[2:] != pick_point[2:]: #current_point[2]즉 Z축을 제외한 나머지를 pick_point 일치시킨다.
                    new_position = posx(pick_point[0],pick_point[1],current_point[2]+6,pick_point[3],pick_point[4],pick_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                    movel(move_up_for_cup, vel=20, acc=40, ref=DR_TOOL, mod=DR_MV_MOD_REL) #위로 10만큼 offset
                    mwait()
                    movel(new_position, vel=20, acc=40, ref=DR_BASE)

    def check_cup_position_and_pick(pick_point):
        move_up_for_cup = posx(0,0,-10,0,0,0)
        move_down_for_cup = posx(0,0,28,0,0,0)
        move_up_for_cup_pick = posx(0,0,-120,0,0,0)
        gripper_close()
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        set_ref_coord(DR_BASE)#기준 좌표계를 Base로 변경
        task_compliance_ctrl([2000,2000,500,100,100,100])
        fd = [-2,0,-50,0,0,0] #외력을 base 기준 좌표계 기준으로 -2,와 -50으로 힘을 주어 내려갈 때 축이 들어지는 것을 방지
        fctrl_dir=[1,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        #print("컵을 집기 위해 내려가기 시작")
        while True:
            #print("컵 집기 포지션 체크를 위해 내려가는 중")
            fcon1 = check_force_condition(DR_AXIS_Z, min = 3, ref=DR_BASE)
            if fcon1 == 0: #외력이 감지가 되었을 때
                release_compliance_ctrl()
                mwait()
                pos_second, _ = get_current_posx(ref=DR_BASE)
                alignment_for_pick_cup(pos_second, pick_point) #집기 직전 position 수정
                movel(move_up_for_cup, v=20, a=40, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                mwait(1)
                gripper_open()
                movel(move_down_for_cup, v=20, a=40, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                mwait(1)
                gripper_close()
                print("컵 집기 끝")
                movel(move_up_for_cup_pick, v=VELOCITY, a=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                print("컵을 들어")
                break
            wait(1)

    def check_cup_position_and_release(goal_point):
        move_up_for_cup = posx(0,0,-10,0,0,0)
        move_up_for_cup_pick = posx(0,0,-120,0,0,0)
        z_axis_alignment(get_current_posx()[0],get_current_posj()) #z축 정렬
        set_ref_coord(DR_BASE)#기준 좌표계를 Base로 변경
        task_compliance_ctrl([500,500,100,100,100,100])
        fd = [-2,0,-50,0,0,0]
        fctrl_dir=[1,0,1,0,0,0]
        set_desired_force(fd, dir=fctrl_dir, mod=DR_FC_MOD_REL)
        print("컵을 놓기 위해 내려가기 시작")
        while True:
            fcon1 = check_force_condition(DR_AXIS_Z, min = 3, ref=DR_BASE)
            if fcon1 == 0: #외력이 감지가 되었을 때
                #print("컵 포지션 체크 끝")
                release_compliance_ctrl()
                mwait()
                pos_second, _ = get_current_posx(ref=DR_BASE)
                alignment_for_pick_cup(pos_second,goal_point)
                gripper_open()
                print("컵 놓기 끝")
                mwait()
                movel(move_up_for_cup_pick, v=VELOCITY, a=ACC, ref=DR_TOOL, mod=DR_MV_MOD_REL)
                print("놓고 올라오기 완료")
                break
            wait(1)

    def equilateral_triangle_center(pos, cup_length=80, x_direction_sign=1, y_direction_sign=1):
        # 중심 좌표 계산 (centroid calculation)
        center_x = pos[0] + cup_length*x_direction_sign
        center_y = pos[1] + cup_length*math.tan(math.radians(30))*y_direction_sign
        return_pos_list = [center_x, center_y, 400 ,0, -180, 0]
        return return_pos_list


    def calculate_triangle_points(pos_start, cup_length=80, cup_count=6, x_direction_sign=1, y_direction_sign=1):
        if x_direction_sign not in {1, -1} or y_direction_sign not in {1, -1}:
            raise ValueError("Direction signs must be 1 (increase) or -1 (decrease).")

        x, y, z, roll, pitch, yaw = pos_start

        if cup_count == 1:
            points = [list(pos_start)]  # pos_start를 리스트로 변환하여 추가
        if cup_count >= 3:
            x_offset = x_direction_sign * cup_length * math.cos(math.radians(60))
            y_offset = y_direction_sign * cup_length * math.sin(math.radians(60))
            points.append([round(x + x_direction_sign * cup_length, 3), round(y, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
            points.append([round(x + x_offset, 3), round(y + y_offset, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
        if cup_count == 6:
            x_offset = x_direction_sign * cup_length * math.cos(math.radians(60))
            y_offset = y_direction_sign * cup_length * math.sin(math.radians(60))
            points.append([round(x + 2 * x_direction_sign * cup_length, 3), round(y, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
            points.append([round(x + x_offset + x_direction_sign * cup_length, 3), round(y + y_offset, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
            points.append([round(x + 2 * x_offset, 3), round(y + 2 * y_offset, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])

        return points
    
    def calculate_triangle_points(pos_start, cup_length=80, cup_count=6, x_direction_sign=1, y_direction_sign=1):
            if x_direction_sign not in {1, -1} or y_direction_sign not in {1, -1}:
                raise ValueError("Direction signs must be 1 (increase) or -1 (decrease).")

            x, y, z, roll, pitch, yaw = pos_start

            if cup_count == 1:
                points = [list(pos_start)]  # pos_start를 리스트로 변환하여 추가
            if cup_count == 3:
                x_offset = x_direction_sign * cup_length * math.cos(math.radians(60))
                y_offset = y_direction_sign * cup_length * math.sin(math.radians(60))
                points.append([round(x - x_direction_sign * cup_length*1/2, 3), round(y - y_direction_sign * cup_length*1/2*math.tan(math.radians(30)), 3),round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                points.append([round(x + x_direction_sign * cup_length*1/2, 3), round(y - y_direction_sign * cup_length*1/2*math.tan(math.radians(30)), 3),round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                points.append([round(x,3), round(y +y_direction_sign * (cup_length*1/2*math.tan(math.radians(60)) - cup_length*1/2*math.tan(math.radians(30))), 3),round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
            if cup_count == 6:
                points.append([round(x - x_direction_sign * cup_length, 3), round(y - y_direction_sign * cup_length*math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                points.append([round(x , 3), round(y - y_direction_sign * cup_length*math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                points.append([round(x + x_direction_sign * cup_length, 3), round(y - y_direction_sign * cup_length*math.tan(math.radians(30)), 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                # points.append([round(x + x_offset + x_direction_sign * cup_length, 3), round(y + y_offset, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])
                # points.append([round(x + 2 * x_offset, 3), round(y + 2 * y_offset, 3), round(z, 3), round(roll, 3), round(pitch, 3), round(yaw, 3)])

            return points
    
    def reverse_cup(position_setting,pick_for_revers, pick, drop):# 필요 데이터. 첫 포지션, 컵을 뒤집을 위치, 컵을 잡을 위치, 컵을 놓을 위치
        gripper_open()
        print("position_setting : ",position_setting)
        movej(position_setting,vel=CUP_VEL, acc=CUP_ACC)
        print("pick_for_revers : ",pick_for_revers)
        movej(pick_for_revers,vel=CUP_VEL, acc=CUP_ACC)
        gripper_close()
        pick_for_revers_up = fkin(pick_for_revers, DR_BASE)
        pick_for_revers_up[2] += 100#posx
        print("pick_for_revers_up : ",pick_for_revers_up)
        movel(pick_for_revers_up,vel=CUP_VEL, acc=CUP_ACC)
        pick_for_revers_up[5] -= 180
        print("pick_for_revers_up : ",pick_for_revers_up)
        movel(pick_for_revers_up,vel=CUP_VEL, acc=CUP_ACC)
        pick_for_revers_up[2] -= 100#posx
        print("pick_for_revers_up : ",pick_for_revers_up)
        movel(pick_for_revers_up,vel=CUP_VEL, acc=CUP_ACC)
        gripper_open()
        pick_for_revers_up[1] -= 100#posx
        print("pick_for_revers_up : ",pick_for_revers_up)
        movel(pick_for_revers_up,vel=CUP_VEL, acc=CUP_ACC)
        print("home : [0,0,90,0,90,0]")
        movej([0,0,90,0,90,0],vel=CUP_VEL, acc=CUP_ACC)
        pick[2] += 70
        print("pick : ",pick)
        movel(pick,vel=CUP_VEL, acc=CUP_ACC)
        pick[2] -= 70
        print("pick : ",pick)
        movel(pick,vel=CUP_VEL, acc=CUP_ACC)
        gripper_close()
        pick[2] += 70
        print("pick : " ,pick)
        movel(pick,vel=CUP_VEL, acc=CUP_ACC)
        print(" drop : ",drop)
        movel(drop,vel=CUP_VEL, acc=CUP_ACC)
        gripper_open()
######################################################################################################

    try:
        #while rclpy.ok():
        #######시작
        tool_setting(TOOL_NAME,TOOL_WEIGHT)
        #컵 잡을 위치 설정

        point_home = posx(410.25,235.35,262,0,-180,0)

        #컵 놓을 위치 설정
        start_position = posx(424.87, -75.56, 250, 0, -180, 0)
        first_floor_points = calculate_triangle_points(start_position, cup_length=80, cup_count=6, x_direction_sign=-1, y_direction_sign=1)
        second_floor_start_position = posx(start_position[0]-40, start_position[1]+(40*math.tan(math.radians(30))), 250, 0, -180, 0)
        center_point = equilateral_triangle_center(start_position,cup_length=80, x_direction_sign=-1, y_direction_sign =1)
        third_floor_point = posx(center_point)


        gripper_open()
        movej(posj(0,0,90,0,90,0),v=20,acc=20) #시작 지점

        position_setting = posj(-22.80, 29.13, 110.09, 72.14, 104.59, -51.48)
        pick_for_revers = posj(-14.53, 27.23, 113.56, 78.44, 99.05, -51.68)
        pick = posx(410.25, 235.35, 262.00, 47.21, 179.93, 46.78)
        drop = posx(277.43, -222.86, 57.95, 46.98, 179.93, 46.55)
        reverse_cup(position_setting, pick_for_revers, pick, drop)

        movel(point_home, v=50, a= 100,ref=DR_BASE) #홈 위치로 이동
        #뒤집은 컵 놓기

        #1층 컵 쌓기 시작
        print(first_floor_points)
        print("1층 컵 쌓기 시작")
        for index in range(0, len(first_floor_points)):
            print(f"current_index: {index}")
            current_cup_position = get_current_posx()[0]
            mwait()
            check_cup_position_and_pick(current_cup_position)
            print(f"currennt cup cup position_1:{current_cup_position}")
            print("cup pick End")
            #print(posx(first_floor_points[index]))
            movel(posx(first_floor_points[index]),v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            ###################첫번째 포인트 도착
            mwait()
            print("go to first point")
            current_cup_position = get_current_posx()[0]
            print(f"currennt cup cup position_2:{current_cup_position}")
            ####################컵 놓기 시작
            check_cup_position_and_release(current_cup_position)
            mwait()
            ####################컵 놓기 끝 컵 잡는 위치로 복귀
            movel(point_home, v=50, a= 100) #홈 위치로 이동
            mwait()
            current_cup_position = get_current_posx()[0]
            print(f"currennt cup cup position_3: {current_cup_position}")

        second_floor_points = calculate_triangle_points(second_floor_start_position, cup_length=80, cup_count=3, x_direction_sign=-1, y_direction_sign=1)
        print(second_floor_points)
        print("2층 컵 쌓기 시작")
        #2층 컵 쌓기 시작
        for index in range(0, len(second_floor_points)):
            print(f"current_index: {index}")
            current_cup_position,_ = get_current_posx()
            check_cup_position_and_pick(current_cup_position)
            print(f"currennt cup cup position_1:{current_cup_position}")
            print("cup pick End")
            movel(posx(second_floor_points[index]),v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)
            mwait()
            ###################첫번째 포인트 도착
            print("go to first point")
            current_cup_position,_ = get_current_posx()
            #print(f"currennt cup cup position_2:{current_cup_position}")
            ####################컵 놓기 시작
            check_cup_position_and_release(second_floor_points[index])
            mwait()
            ####################컵 놓기 끝 컵 잡는 위치로 복귀
            movel(point_home, v=50, a= 100,ref=DR_BASE) #홈 위치로 이동
            mwait()
            current_cup_position = get_current_posx()[0]
            print(f"currennt cup cup position_3: {current_cup_position}")

        print("3층 컵 쌓기 시작")
        #3층 컵 쌓기
        print(f"current_index: {index}")
        current_cup_position = get_current_posx()[0]
        check_cup_position_and_pick(current_cup_position)
        print(f"currennt cup cup position_1:{current_cup_position}")
        print("cup pick End")
        movel(posx(current_cup_position[0],current_cup_position[1],400,current_cup_position[3],current_cup_position[4],current_cup_position[5]),v=VELOCITY, a=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS)
        print("집고 들기 완료")
        movel(posx(third_floor_point),v=VELOCITY, a=ACC, ref=DR_BASE, mod=DR_MV_MOD_ABS)
        ###################첫번째 포인트 도착
        print("go to thrid point")
        current_cup_position = get_current_posx()[0]
        print(f"currennt cup cup position_2:{current_cup_position}")
        ####################컵 놓기 시작
        check_cup_position_and_release(third_floor_point)
        ####################컵 놓기 끝 컵 잡는 위치로 복귀
        movel(posx(current_cup_position[0],current_cup_position[1],400,current_cup_position[3],current_cup_position[4],current_cup_position[5]),v=VELOCITY, a=ACC,ref=DR_BASE, mod=DR_MV_MOD_ABS) #컵 잡는 위치로 복귀
        current_cup_position = get_current_posx()[0]
        print(f"currennt cup cup position_3: {current_cup_position}")
        wait(1)
        print("탑 쌓기 끝")

        #################마지막 컵 잡기
        upcup_pos = [277.42,-222.85,77.97,47.70,180,47.26]# z = 57.97
        delta_z = [0,0,-20,0,0,0]
        des_pos=[343.53,-29.69,360.03,109.65,-180,103.84] #ori z = 285 (그립퍼 폭이 0인 상태에서 측정한 높이)
        height= 285 # 탑의 높이 약 342mm
        #movel([0,0,10,0,0,0],vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_REL) # 반대로 뒤집어 진 위치로 z축 다운
        movel(upcup_pos,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_ABS) # 반대로 뒤집어 진 위치 + z 20 지점으로 이동
        movel(delta_z,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_REL) # 반대로 뒤집어 진 위치로 z축 다운
        gripper_close()
        movel([0,0,height,0,0,0],vel=VELOCITY,acc=ACC,ref=DR_BASE,mod=DR_FC_MOD_REL) # 바로 이동하면 부딪히니까 탑 높이까지 올려 이동
        movel(des_pos,vel=VELOCITY,acc=ACC,ref=DR_BASE) # 탑이 쌓여있는 위치로 이동
        check_cup_position_and_release(des_pos)
        print("!!!!!!!!!!!!!!!끝!!!!!!!!!!!!!!")












    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # 자원 정리
        rclpy.shutdown()

if __name__ == "__main__":
    main()
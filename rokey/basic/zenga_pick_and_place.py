import rclpy
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 30, 30

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("zenga_pick_and_place", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        import DSR_ROBOT2 as DS
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        return

    def gripper_close():
        DS.set_digital_output(1,ON)
        DS.set_digital_output(2,OFF)
        print("그리퍼 닫힘")
        DS.wait(0.5)
        #DS.mwait(0.5)

    def gripper_open():
        DS.set_digital_output(1,OFF)
        DS.set_digital_output(2,ON)
        print("그리퍼 열림")
        DS.wait(0.5)
        #DS.mwait(0.5)
        
    def z_axis_alignment(axes_data, joint_data):
        vect = [0, 0, -1]
        DS.parallel_axis(vect, DS.DR_AXIS_Z, DS.DR_BASE)
        DS.mwait()
        print("move to z-axis alignment")
        print(DS.get_current_posj(), joint_data)
        print(DS.get_current_posx()[0], axes_data)
        # move_by_line(axes_data, joint_data)

    def zenga_block_pick(pic_point):
        gripper_close()
        print("Z축 정렬 시작")
        z_axis_alignment(DS.get_current_posx()[0],DS.get_current_posj())
        print("Z축 정렬 끝")
        pos_first = []
        #DS.set_ref_coord(DS.DR_TOOL)
        DS.set_ref_coord(DS.DR_BASE)#기준 좌표계를 Base로 변경
        DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        #fd = [0,0,30,0,0,0]
        fd = [0,0,-30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        DS.set_desired_force(fd, dir=fctrl_dir, mod=DS.DR_FC_MOD_REL)
        print("블록을 집기 위해 내려가기 시작")
        while True:
            fron1 = -1
            fcon1 = DS.check_force_condition(DS.DR_AXIS_Z, min = 5, ref=DS.DR_BASE)
            pos_first, _ = DS.get_current_posx(ref=DS.DR_BASE)
            print(f"현재 pos값: {pos_first}")
            if fcon1 == 0 and pos_first[2] >= 5: #힘이 인식되었고 현재 z축의 좌표가 5보다 클 때 = 블록이 있다.
                print("현재 위치가 0이상이고 블록이 감지됨 힘 제어 해제")
                DS.release_compliance_ctrl()
                DS.mwait()
                print("힘 제어 풀기 완료")
                print(f"pic_point: {pic_point}")

                while True:
                    print("정렬 시작")
                    pos_second, _ = DS.get_current_posx(ref=DS.DR_BASE)
                    if pos_second[:1] != pic_point[:1] and pos_second[2:] != pic_point[2:]: #pos_second[2]즉 Z축을 제외한 나머지를 pic_point와 일치시킨다.
                        new_position = posx(pic_point[0],pic_point[1],pos_second[2],pic_point[3],pic_point[4],pic_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                        DS.movel(move_up, vel=VELOCITY, acc=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL) #위로 10만큼 offset
                        DS.mwait(0.5)
                        #new_position = posx(pic_point[0],pic_point[1],pos_second[2],pic_point[3],pic_point[4],pic_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                        DS.movel(new_position, vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                        #print("정렬 진행 중")
                        #DS.mwait()
                        print("정렬 완료")
                        DS.mwait()
                        break
                    DS.wait(0.1)

                print("블록 집기 시작")
                DS.movel(move_up, v=VELOCITY, a=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL)
                gripper_open()
                DS.movel(move_down, v=VELOCITY, a=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL)
                gripper_close()
                print("블록 집기 성공")
                break
            elif fcon1 == 0 and pos_first[2] <5: #힘이 인식되었고 현재 z축의 좌표가 5보다 작을 때 더이상 옮길 블록이 없다.
                rclpy.shutdown() #ros program 종료
            DS.wait(0.5)

    def zenga_block_release(goal_point):
        #DS.set_ref_coord(DS.DR_TOOL)
        DS.set_ref_coord(DS.DR_BASE)
        z_axis_alignment(DS.get_current_posx()[0],DS.get_current_posj())
        DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        #fd = [0,0,30,0,0,0]
        fd = [0,0,-30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        DS.set_desired_force(fd, dir=fctrl_dir, mod=DS.DR_FC_MOD_REL)
        print("블록을 놓기 위해 내려가기 시작")
        while True:
            fron1 = -1
            fcon1 = DS.check_force_condition(DS.DR_AXIS_Z, min = 5, ref=DS.DR_BASE)
            if fcon1 == 0:
                print("블록 내려놓을 위치 도착")
                DS.release_compliance_ctrl()
                DS.mwait()
                print("힘 제어 풀기 완료")

                while True:
                    print("정렬 시작")
                    print(f"Goal point: {goal_point}")
                    pos_second, _ = DS.get_current_posx(ref=DS.DR_BASE)
                    if pos_second[:1] != goal_point[:1] and pos_second[2:] != goal_point[2:]: #pos_second[2]즉 Z축을 제외한 나머지를 pic_point와 일치시킨다.
                        new_position = posx(goal_point[0],goal_point[1],pos_second[2],goal_point[3],goal_point[4],goal_point[5]) #현재 좌표에서 Z를 제외한 나머지 값을 보정함으로서 블록과 정렬시킴
                        DS.movel(move_up, vel=VELOCITY, acc=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL) #위로 10만큼 offset
                        DS.mwait(0.5)
                        DS.movel(new_position, vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                        #print("정렬 진행 중")
                        #DS.mwait(1)
                        print("정렬 완료")
                        DS.mwait()
                        break
                    DS.wait(0.1)
                gripper_open()
                print("블록 놓기 성공")
                break
            DS.wait(0.5)

    #DS.set_robot_mode(0)
    #DS.add_tcp("G1",[0,0,228,0,0,0])
    #DS.wait(0.2)
    #DS.add_tool("G1_gripper", 0.82, [0,0,0],[0,0,0,0,0,0])
    #DS.wait(0.2)#transition_time만큼 기다려야
    #print(DS.get_tcp())
    #print(DS.get_tool())
    #DS.set_robot_mode(1)

    try:
        while rclpy.ok():
            #######시작
            point_home = posj(0,0,90,0,90,0) #홈 위치로 이동
            move_down = posx(0,0,32,0,0,0) #젠가를 잡기 위해 내려감
            move_up = posx(0,0,-10,0,0,0) #젠가를 잡기 위해 올라감
            #zenga_block_pick_length_l = posx(277.760,-49.5,150.0, 81.91, -180.0, 84.55) #로봇 기준 가장 왼쪽 세로 블록
            #zenga_block_pick_length_m = posx(277.760,-74.5,150.0, 136.93, -180.0, 137.89) # 로봇 기준 중간 세로 블록
            #zenga_block_pick_length_r = posx(277.760,-98.5,150.0, 157.83, 180.0, 158.79) # 로봇 기준 가장 오른쪽 세로 블록
            #zenga_block_pick_width_f = posx(301.99, -70.960, 150.0, 165.93, 180, 75.81) # 로봇 기준 가장 멀리 있는 가로 블록
            #zenga_block_pick_width_m = posx(277.01, -69.980, 150.0, 165.49, -180, 75.38) # 로봇 기준 중간에 있는 가로 블록
            #zenga_block_pick_width_c = posx(253.03, -69.970, 150.0, 160.53, -180, 70.41) # 로봇 기준 가장 가까이 있는 가로 블록
            #zenga_block_goal_length_r = posx(273.97, 46.99, 150.0, 8.24, -180, 10)#로봇 기준 가장 오른쪽에 있는 세로 블록
            #zenga_block_goal_length_m = posx(273.99, 72.99, 150.0, 16.15, -180, 18.79)#로봇 기준 가장 오른쪽에 있는 세로 블록
            #zenga_block_goal_length_l = posx(275.00, 96.00, 150.0, 19.08, -180, 21.72)#로봇 기준 가장 오른쪽에 있는 세로 블록
            #zenga_block_goal_width_f = posx(300.0, 74.0, 150.0, 2.52, 180, -86.05) # 로봇 기준 가장 멀리 있는 가로 블록
            #zenga_block_goal_width_m = posx(275.0, 74.0, 150.0, 2.52, -180, -86.05) # 로봇 기준 중간에 있는 가로 블록
            #zenga_block_goal_width_c = posx(250.0, 74.0, 150.0, 2.52, 179.99, -86.05) # 로봇 기준 가장 가까이 있는 가로 블록
            ###################################################################################################
            zenga_block_pick_length_l = posx(277.760,-49.5,150.0, 0, -180.0, 0) #로봇 기준 가장 왼쪽 세로 블록
            zenga_block_pick_length_m = posx(277.760,-74.5,150.0, 0, -180.0, 0) # 로봇 기준 중간 세로 블록
            zenga_block_pick_length_r = posx(277.760,-99.5,150.0, 0, -180.0, 0) # 로봇 기준 가장 오른쪽 세로 블록
            zenga_block_pick_width_f = posx(302.760, -74.5, 150.0, 0, -180, -90) # 로봇 기준 가장 멀리 있는 가로 블록
            zenga_block_pick_width_m = posx(277.760, -74.5, 150.0, 0, -180, -90) # 로봇 기준 중간에 있는 가로 블록
            zenga_block_pick_width_c = posx(252.760, -74.5, 150.0, 0, -180, -90) # 로봇 기준 가장 가까이 있는 가로 블록
            zenga_block_goal_length_r = posx(273.97, 49.0, 150.0, 0, -180, 0)#로봇 기준 가장 오른쪽에 있는 세로 블록
            zenga_block_goal_length_m = posx(273.97, 74.0, 150.0, 0, -180, 0)#로봇 기준 가장 오른쪽에 있는 세로 블록
            zenga_block_goal_length_l = posx(273.97, 99.0, 150.0, 0, -180, 0)#로봇 기준 가장 오른쪽에 있는 세로 블록
            zenga_block_goal_width_f = posx(298.97, 74.0, 150.0, 0, -180, -90) # 로봇 기준 가장 멀리 있는 가로 블록
            zenga_block_goal_width_m = posx(273.97, 74.0, 150.0, 0, -180, -90) # 로봇 기준 중간에 있는 가로 블록
            zenga_block_goal_width_c = posx(248.97, 74.0, 150.0, 0, -180, -90) # 로봇 기준 가장 가까이 있는 가로 블록
            ###################################################################################################
            zenga_block_pick_list = [zenga_block_pick_length_l,zenga_block_pick_length_m,zenga_block_pick_length_r,zenga_block_pick_width_f,zenga_block_pick_width_m,zenga_block_pick_width_c]
            zenga_block_goal_list = [zenga_block_goal_length_r,zenga_block_goal_length_m,zenga_block_goal_length_l,zenga_block_goal_width_f,zenga_block_goal_width_m,zenga_block_goal_width_c]
            gripper_open()
            gripper_close()
            DS.movej(point_home, v=50, a= 100)

            for index in range(len(zenga_block_pick_list)):
                DS.movel(zenga_block_pick_list[index], vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                zenga_block_pick(zenga_block_pick_list[index])
                #pos_first, _ = DS.get_current_posx(ref=DS.DR_BASE)
                #print(f"현재 pos_z값: {pos_first}")
                print(f"current pick position {zenga_block_pick_list[index]}")
                #DS.mwait(0.5)
                DS.movel(zenga_block_pick_list[index], vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                DS.mwait(0.5)
                #print(f"current pick position {zenga_block_pick_list[index]}")
                print("블록 집고 들기 완료")
                ###################놓기 과정 시작
                DS.movel(zenga_block_goal_list[index], vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                zenga_block_release(zenga_block_goal_list[index])
                DS.mwait(0.5)
                DS.movel(zenga_block_goal_list[index], vel=VELOCITY, acc=ACC, ref=DS.DR_BASE)
                DS.mwait(0.5)
                print(f"current goal position {zenga_block_goal_list[index]}")
                print("블록 놓기 완료")
                
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # 자원 정리
        rclpy.shutdown()

if __name__ == "__main__":
    main()
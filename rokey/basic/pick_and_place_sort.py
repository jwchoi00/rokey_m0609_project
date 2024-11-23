
import rclpy
import DR_init
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

ON, OFF = 1, 0

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("stack_block", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        import DSR_ROBOT2 as DS
        from DR_common2 import posx, posj
    except ImportError as e:
        node.get_logger().error(f"Error importing DSR_ROBOT2: {e}")
        return

    global current_block_size
    
    
    def gripper_close():
        DS.set_digital_output(1,ON)
        DS.set_digital_output(2,OFF)
        #wait_digital_input(1,ON)
        print("그리퍼 열림")
        DS.mwait(0.5)
    def gripper_open():
        DS.set_digital_output(1,OFF)
        DS.set_digital_output(2,ON)
        DS.mwait(0.5)
        print("그리퍼 닫힘")
        #wait_digital_input(2,ON)

    def get_pattern(pos1, pos2, pos3, pos4, row, column, unzip=False):
        pos_list = [pos1, pos2, pos3, pos4]
        return_pos_list = []
        vertical = pos3[0] - pos1[0]  # Difference in x-coordinates between pos1 and pos3
        height = pos2[1] - pos1[1]   # Difference in y-coordinates between pos1 and pos2
        # Generate positions
        for r in range(row):
            for c in range(column):
                new_pos = [
                    pos1[0] + (vertical * r / (row - 1) if row > 1 else 0),  # Increment in x
                    pos1[1] + (height * c / (column - 1) if column > 1 else 0),  # Increment in y
                    pos1[2], pos1[3], pos1[4], pos1[5]  # Keep other values unchanged
                ]
                #print(f"current_position: {r},{c}")
                return_pos_list.append(new_pos)
        if unzip:
            reordered_list = []
            for c in range(column):
                for r in range(row):
                    reordered_list.append(return_pos_list[r * column + c])  # Traverse by column first
            return reordered_list
        return return_pos_list

    def check_size(Z):
        block_size = 0 #1=작은거, 2=중간거, 3=큰거
        if Z > 104:
            block_size = 0
        elif Z > 94:
            block_size = 1
        else:
            block_size = 2
        return block_size

    def block_pick():
        global current_block_size
        pos_first = []
        pos_second = []
        DS.set_ref_coord(DS.DR_TOOL)
        pos_first, sol = DS.get_current_posx(ref=DS.DR_BASE)
        print(f"현재 pos_z값: {pos_first[2]}")
        DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        fd = [0,0,30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        DS.set_desired_force(fd, dir=fctrl_dir, mod=DS.DR_FC_MOD_REL)
        while True:
            fron1 = -1
            fcon1 = DS.check_force_condition(DS.DR_AXIS_Z, min = 5, ref=DS.DR_BASE)
            if fcon1 == 0:
                print("내려가기 끝")
                DS.release_compliance_ctrl()
                DS.mwait()
                #DS.release_force()
                print("집을 때 힘 제어 풀기")
                pos_second, sol = DS.get_current_posx(ref=DS.DR_BASE)
                Z_Gap = pos_first[2] - pos_second[2]
                print(Z_Gap)
                current_block_size = check_size(Z_Gap)
                print("Z-Gap check")
                DS.movel(move_up, v=VELOCITY, a=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL)
                gripper_open()
                DS.movel(move_down, v=VELOCITY, a=ACC, ref=DS.DR_TOOL, mod=DS.DR_MV_MOD_REL)
                gripper_close()
                break
            DS.wait(0.5)

    def block_release():
        DS.set_ref_coord(DS.DR_TOOL)
        DS.task_compliance_ctrl(stx=[500, 500, 500, 100, 100, 100])
        fd = [0,0,30,0,0,0]
        fctrl_dir=[0,0,1,0,0,0]
        DS.set_desired_force(fd, dir=fctrl_dir, mod=DS.DR_FC_MOD_REL)
        while True:
            fron1 = -1
            fcon1 = DS.check_force_condition(DS.DR_AXIS_Z, min = 10, ref=DS.DR_BASE)
            if fcon1 == 0:
                DS.release_compliance_ctrl()
                #DS.mwait()
                #DS.release_force()
                print("놓을 때 힘 제어 풀기")
                gripper_open()
                break
            DS.wait(0.5)

    DS.set_tool("Tool Weight_2FG")
    DS.set_tcp("2FG_TCP")
    try:
        while rclpy.ok():
            #######시작
            point_home = posj(0,0,90,0,90,0)
            move_down = posx(0,0,32,0,0,0)
            move_up = posx(0,0,-10,0,0,0)
            pick_point_list = []
            goal_point_list = []
            gripper_open()
            DS.movej(point_home, v=50, a= 100)
            gripper_close()

            #pos1 = [399.62,-1.00,150,174.43,179.48,172.93]
            #pos2 = [399.29,-103.01,150,174.47,179.48,172.97]
            #pos3 = [498.ACC,-103.00,150,174.42,179.48,172.92]
            #pos4 = [500.28,0.01,150,174.41,179.48,172.92]

            pos1 = [402.00,-1.00,150,0,180,0]
            pos2 = [402.00,-103.01,150,0,180,0]
            pos3 = [500.28,-103.00,150,0,180,0]
            pos4 = [500.28,0.01,150,0,180,0]
            row = 3
            column = 3
            current_point_number = 0
            for pallet_index in get_pattern(pos1, pos2, pos3, pos4, row, column):
                Pallet_Pose = pallet_index
                pick_point_list.append(Pallet_Pose)
            pos5 = [404.00,149.62,150,0,180,0]
            pos6 = [404.00,49.88,150,0,180,0]
            pos7 = [502.91,49.88,150,0,180,0]
            pos8 = [502.91,149.88,150,0,180,0]
            row = 3
            column = 3
            current_point_number = 0
            for pallet_index in get_pattern(pos5, pos6, pos7, pos8, row, column, unzip=True):
                Pallet_Pose = pallet_index
                goal_point_list.append(Pallet_Pose)
            total_count = len(pick_point_list)
            check_goal_list = [False for _ in range(total_count)]

            goal_index = 0
            while current_point_number!=total_count:
                print(check_goal_list)
                index = int(current_point_number%total_count)
                DS.movel(pick_point_list[index], v = VELOCITY, a= ACC, ref=DS.DR_BASE)
                print(pick_point_list[index])
                print("point에 도착")
                gripper_close()
                block_pick()
                print("point끝")
                DS.movel(pick_point_list[index], v = VELOCITY, a= ACC, ref=DS.DR_BASE)
                print("집기 끝")
                #############################지금 블록 사이즈 파악 후 이동
                print(f"current_block_size: {current_block_size}")
                goal_index = current_block_size * row  # 배치 위치
                if check_goal_list[goal_index]==True:
                    goal_index += 1
                    if check_goal_list[goal_index]==True:
                        goal_index += 1
                print(f"goal_index: {goal_index}")
                # 이동 시 goal_point_list에서 위치를 선택하여 목표 위치로 이동
                DS.movel(goal_point_list[goal_index], v=VELOCITY, a=ACC, ref=DS.DR_BASE)
                print("블록 놓기 시작")
                block_release()
                print("블록 놓기 끝")
                #gripper_close()
                check_goal_list[goal_index] = True
                DS.movel(goal_point_list[goal_index], v=VELOCITY, a=ACC, ref=DS.DR_BASE)
                print("1회 루프 끝")
                current_point_number += 1
                if index == total_count-1:
                    is_from_pick_to_goal = True
                    current_point_number = 0
                    break
                DS.wait(0.5)
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        # 자원 정리
        rclpy.shutdown()

if __name__ == "__main__":
    main()
import rclpy
import DR_init

ON,OFF=1,0
VEL,ACC=30,30
TOOL_NAME='Tool'
TOOL_WEIGHT='Gripper'
ROBOT_ID='dsr01'
ROBOT_MODEL='m0609'

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def main():
    rclpy.init()
    node = rclpy.create_node('sungeer_move',namespace=ROBOT_ID)
    DR_init.__dsr__node=node
    

    from DSR_ROBOT2 import (set_robot_mode,
                            set_tool,
                            set_tcp,
                            get_tool,
                            get_tcp,
                            set_digital_output,
                            get_digital_input,
                            get_tool_force,
                            movel,
                            movej,
                            move_periodic,
                            mwait,
                            wait,
                            task_compliance_ctrl,
                            set_desired_force,
                            check_force_condition,
                            release_compliance_ctrl,
                            DR_BASE,
                            DR_TOOL,
                            DR_FC_MOD_REL,
                            DR_FC_MOD_ABS,
                            DR_AXIS_Z,
                            )
    
    def tool_setting(tool_name,tool_weight):
        set_robot_mode(0)
        set_tool(tool_name)
        set_tcp(tool_weight)
        set_robot_mode(1)
        print('tool name: {0}, tool weight: {1}'.format(get_tool(),get_tcp()))
    
    def wait_for_input(pin):
        while not get_digital_input(pin):
            print("아직 못잡음")
            wait(0.5)
        print('물체 잡음')

    def close():
        set_digital_output(1,ON)
        set_digital_output(2,OFF)
        wait_for_input(1)

    
    def open():
        task_compliance_ctrl(stx=[500,500,500,100,100,100])
        set_desired_force(fd=[0,0,-10,0,0,0],dir=[0,0,1,0,0,0],mod=DR_FC_MOD_REL)
        while not check_force_condition(axis=DR_AXIS_Z,ref=DR_BASE,max=10):
            pass
        release_compliance_ctrl()
        mwait()
        set_digital_output(1,OFF)
        set_digital_output(2,ON)
        wait_for_input(2)


################### main start
    tool_setting(TOOL_NAME,TOOL_WEIGHT)
    upcup_pos = [277.42,-222.85,77.97,47.70,180,47.26]# z = 57.97
    delta_z = [0,0,-20,0,0,0]
    des_pos=[343.53,-29.69,342.03,109.65,-180,103.84] #ori z = 285 (그립퍼 폭이 0인 상태에서 측정한 높이)
    height= 285 # 탑의 높이 약 342mm
    movel([0,0,10,0,0,0],vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_REL) # 반대로 뒤집어 진 위치로 z축 다운
    movel(upcup_pos,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_ABS) # 반대로 뒤집어 진 위치 + z 20 지점으로 이동
    movel(delta_z,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_REL) # 반대로 뒤집어 진 위치로 z축 다운
    close()
    movel([0,0,height,0,0,0],vel=VEL,acc=ACC,ref=DR_BASE,mod=DR_FC_MOD_REL) # 바로 이동하면 부딪히니까 탑 높이까지 올려 이동
    movel(des_pos,vel=VEL,acc=ACC,ref=DR_BASE) # 탑이 쌓여있는 위치로 이동
    open()

    ### 높이 테스트
    # movel(upcup_pos,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_ABS) # 반대로 뒤집어 진 위치 + z 20 지점으로 이동
    # movel(delta_z,vel=30,acc=30,ref=DR_BASE,mod=DR_FC_MOD_REL) # 반대로 뒤집어 진 위치로 z축 다운
    # movel([0,0,height,0,0,0],vel=VEL,acc=ACC,ref=DR_BASE,mod=DR_FC_MOD_REL) # 바로 이동하면 부딪히니까 탑 
    # movel(des_pos,vel=VEL,acc=ACC,ref=DR_BASE) # 탑이 쌓여있는 위치로 이동
    

    #movej([0,0,90,0,90,0],vel=VEL,acc=ACC) #HOME POS

    rclpy.shutdown()